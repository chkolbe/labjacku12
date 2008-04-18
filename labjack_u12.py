#!/usr/bin/python
#
# Labjack U12 driver
# (c) 2008 Robert Jordens <jordens@phys.ethz.ch> 
# 
# This driver is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2, or (at your option)
# any later version.
# 
# This driver is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with this driver; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.

import usb, time, random
import numpy as np


class LabjackU12(object):
    id_vendor = 0x0cd5
    id_product = 0x0001
    id_interface = 0
    id_configuration = 0

    @classmethod
    def find_all(cls):
        for bus in usb.busses():
            for dev in bus.devices:
                if (dev.idVendor, dev.idProduct) == (
                        cls.id_vendor, cls.id_product):
                    yield cls(dev)
    
    def __init__(self, usbdev):
        self.dev = usbdev
        self.open()
        self.init_read()
        self.caldata = self.calibration()

    def open(self):
        self.handle = self.dev.open()
        # self.handle.reset()
        self.handle.setConfiguration(1)
        self.interface = \
            self.dev.configurations[self.id_configuration
                    ].interfaces[self.id_interface][0]
        self.ep_in = self.interface.endpoints[0]
        self.ep_out = self.interface.endpoints[1]
        assert self.ep_in.address == 0x81
        assert self.ep_out.address == 0x02
        assert self.ep_in.type == usb.ENDPOINT_TYPE_INTERRUPT
        assert self.ep_out.type == usb.ENDPOINT_TYPE_INTERRUPT
        try:
            self.handle.detachKernelDriver(self.id_interface)
        except usb.USBError:
            pass
        self.handle.claimInterface(self.interface)

    def init_read(self):
        assert self.write((0,)*8) == 8
        try:
            return self.read(8)
        except usb.USBError:
            time.sleep(0.02)
            pass
    
    def __del__(self):
        self.close()

    def close(self):
        self.handle.releaseInterface()
        del self.handle

    def feature_read(self):
        return self.handle.controlMsg(
            requestType=usb.TYPE_CLASS|usb.RECIP_INTERFACE|usb.ENDPOINT_IN,
            request=usb.REQ_CLEAR_FEATURE,
            value=(0x03<<8)+0x00, index=0, buffer=128, 
            timeout=5000)

    def write(self, buf, tmo=100):
        return self.handle.interruptWrite(
                self.ep_out.address, buf, tmo)

    def read(self, siz, tmo=100):
        return self.handle.interruptRead(
                self.ep_in.address, siz, tmo)

    def writeread(self, w, tmo=100):
        assert self.write(w, tmo) == 8
        r = self.read(8, tmo)
        return tuple(v&0xff for v in r)

    def read_mem(self, ad):
        assert ad >= 0 & ad <= 8188
        w = (0,0,0,0,0,80) + divmod(ad, 0x100)
        r = self.writeread(w)
        assert r[0] == 80
        assert w[6:] == r[6:]
        return r[1:5]

    def serial(self):
        r = self.read_mem(0)
        return sum(ri << 8*i for i, ri in enumerate(r[::-1]))

    def calibration(self):
        a = np.array([self.read_mem(0x100+(0x010*j)) for j in range(8)])
        b = np.array([self.read_mem(0x180+(0x010*j)) for j in range(4)])
        return np.concatenate((a[:,1], a[:,3], b[:,1]))

    def local_id(self):
        return self.read_mem(8)[3]

    def reset(self):
        return self.write((0,0,0,0, 0,95,0,0))

    def reenumerate(self):
        return self.write((0,0,0,0, 0,64,0,0))

    def firmware_version(self):
        r = self.writeread((1,0,0,0, 0,83,0,0))
        return r[0]+r[1]/100.

    def digital_io(self, conf_d, conf_io, state_d, state_io, update_digital):
        assert 0 <= conf_d <= 0xffff
        assert 0 <= conf_io <= 0xf
        conf_d ^= 0xffff
        conf_io ^= 0xf
        assert 0 <= state_d <= 0xffff
        assert 0 <= state_io <= 0xf

        w = divmod(conf_d, 0x100) + divmod(state_d, 0x100) + (
            (conf_io << 4) | state_io, 87, update_digital, 0)
        print w
        r = self.writeread(w, tmo=1000)
        
        assert r[0] == 87 or r[0] == 119
        state_d = (r[1] << 8) + r[2]
        state_io = r[3] >> 4
        return state_d, state_io

    def analog_output(self, conf_d, conf_io, state_d, state_io, ao0, ao1,
            set_d, reset_c):
        assert 0 <= conf_d <= 0xffff
        assert 0 <= conf_io <= 0xf
        conf_d ^= 0xffff
        conf_io ^= 0xf
        assert 0 <= state_d <= 0xffff
        assert 0 <= state_io <= 0xf
        assert 0 <= ao0 <= 5
        assert 0 <= ao1 <= 5
        ao0 = int(round(ao0/5.*1023))
        ao1 = int(round(ao1/5.*1023))

        w = divmod(conf_d, 0x100) + divmod(state_d, 0x100) + (
            (conf_io << 4) | state_io,
            (set_d << 4) | (reset_c << 5) | ((ao0 & 0x3) << 2) 
                | ((ao1 & 0x3) << 0),
            ao0 >> 2, ao1 >> 2)
        # print w
        r = self.writeread(w, tmo=1000)

        assert not r[0] & (1 << 7)
        state_d = (r[1] << 8) + r[2]
        state_io = r[3] >> 4
        count = sum((v << i*8) for i,v in enumerate(r[4::-1]))
        return state_d, state_io, count

    def mux_cmd(self, ch, g):
        assert 0 <= g <= 7
        assert (ch >= 8) | (g == 0)
        return (g << 4) | (ch^0x8)

    gains = [1, 2, 4, 5, 8, 10, 16, 20]
    def apply_calibration(self, ch, g, v):
        if ch < 8:
            ch &= 0x7
            off = self.caldata[ch]
            v -= off
            gaincal = self.caldata[ch + 8]
            v += (v-2048) / 512. * (off-gaincal)
        else:
            ch &= 0x3
            gain = self.gains[g]
            czse = self.caldata[2*ch] - self.caldata[2*ch+1]
            off = gain*czse/2. + self.caldata[ch+16] - czse/2.
            v -= off
            ccdiff = (self.caldata[2*ch+8]-self.caldata[2*ch]) - \
                    (self.caldata[2*ch+9]-self.caldata[2*ch+1])
            if ccdiff >= 2:
                v -= (v-2048)/256.
            elif ccdiff <= -2:
			    v += (v-2048)/256.
        return max(min(v, 4095), 0)

    def bits_to_volts(self, ch, g, b):
        if ch < 8:
            return b*20./4096 - 10
        else:
            return (b*40./4096 - 20) / self.gains[g] 

    def analog_input(self, state_io, set_io, led, channels, gains):
        assert len(channels) in (1,2,4) # TODO: 1,2 not implemented
        assert len(channels) == len(gains)
        assert 0 <= state_io <= 0xf
        challenge = ord(np.random.bytes(1))
        muxs = tuple(self.mux_cmd(ch, g) for ch, g in 
                zip(channels, gains))
        cmd = 4
        w = muxs + ((led << 0) | (set_io << 1),
                    (1<<7) | (cmd << 4) | (state_io << 0),
                    0, challenge)
        r = self.writeread(w, 1000)
        assert r[0] & (1 << 7)
        assert r[1] == challenge
        ofchecksum, overvoltage = bool(r[0] % (1<<5)), bool(r[0] & (1<<4))
        state_io = r[0] & 0xf
        bits = (((r[2] & 0xf0) << 4) + r[3],
                ((r[2] & 0x0f) << 8) + r[4],
                ((r[5] & 0xf0) << 4) + r[6],
                ((r[5] & 0x0f) << 8) + r[7])
        volts = [self.bits_to_volts(c, g, self.apply_calibration(c, g, v))
                for c, g, v in zip(channels, gains, bits)]
        return overvoltage, ofchecksum, state_io, volts

    def e_analog_input(self, channels, gains=(0,0,0,0)):
        return [time.time(),self.analog_input(0,0,0,channels,gains)[3]]

    def stream_start(self, state_io, set_io, led, channels, gains,
            rate, read_count):
        assert len(channels) in (1,2,4) # TODO: 1,2 not implemented
        assert len(channels) == len(gains)
        assert 0 <= state_io <= 0xf
        assert 200 <= rate <= 1200
        feature_reports = 1 # dubbed "turbo"
        cmd = 1
        sample_int = int(round(6e6/rate/len(channels)))

        muxs = tuple(self.mux_cmd(ch, g) for ch, g in 
                zip(channels, gains))
        w = muxs + ((led << 0) | (set_io << 1) | (feature_reports << 7) |
                        (read_count << 6),
                    (1<<7) | (cmd << 4) | (state_io << 0)) + \
                    divmod(sample_int, 1<<8)
        self.write(w)
    
    def bulk_read(self, channels, gains):
        resp = self.feature_read()
        while len(resp) >= 8:
            r, resp = resp[:8], resp[8:]
            ofchecksum, overvoltage = bool(r[0] % (1<<5)), bool(r[0] & (1<<4))
            state_io = r[0] & 0xf
            iterations = r[1] >> 5
            bits = (((r[2] & 0xf0) << 4) + r[3],
                    ((r[2] & 0x0f) << 8) + r[4],
                    ((r[5] & 0xf0) << 4) + r[6],
                    ((r[5] & 0x0f) << 8) + r[7])
            volts = [self.bits_to_volts(c, g, self.apply_calibration(c, g, v))
                    for c, g, v in zip(channels, gains, bits)]
            count = sum(v<<i*8 for i,v in enumerate(r[4::-1]))
            yield overvoltage, ofchecksum, state_io, volts, count, iterations
        
    def bulk_stop(self):
        self.read_mem(0)

    def burst_start(self, state_io, set_io, led, channels, gains,
            num_scans, trigger, trigger_state, trigger_on,
            rate):
        assert len(channels) in (1,2,4) # TODO: 1,2 not implemented
        assert len(channels) == len(gains)
        assert 0 <= state_io <= 0xf
        feature_reports = 1 # dubbed "turbo"
        cmd = 2
        sample_int = int(round(6e6/rate/len(channels)))
        if sample_int == 732:
            sample_int = 733
        assert 733 <= sample_int < (1<<14)
        muxs = tuple(self.mux_cmd(ch, g) for ch, g in 
                zip(channels, gains))
        w = muxs + ((led << 0) | (set_io << 1) | (trigger_state << 2) |
                    (trigger << 3) | (10-int(round(np.log2(num_scans)))),
                    (1<<7) | (cmd << 4) | (state_io << 0),
                    (feature_reports<<7) | (trigger_on<<6) | sample_int>>8,
                    sample_int%256)
        self.write(w)

    def count(self, reset):
        w = (reset & 1 | 0, 0, 0, 0, 82, 0, 0, 0)
        a = time.time()
        r = self.writeread(w, 100)
        t = (time.time()+a)/2.
        count = sum((v << i*8) for i,v in enumerate(r[4::-1]))
        return count, t 

if __name__ == "__main__":
    for d in LabjackU12.find_all():
        #print d
        # d.open()
        # print d.reset()
        # print np.array([d.read_mem(i) for i in range(0, 8188, 4)])
        #print d.serial()
        #print d.local_id()
        #print d.calibration()
        #print d.firmware_version()
        #print d.e_analog_input((0,1,2,3))
        d.analog_output(0, 0, 0, 0, 1, 4, False, False) # 16ms
        #print d.digital_io(0,0,0,0,1)
        chans, gains, scans = (8,9,10,11), (0,0,0,0), 1024
        a = time.time()
        #d.stream_start(
        #        state_io=0, set_io=0, led=0, channels=chans, gains=gains,
        #        rate=400, read_count=0)
        d.burst_start(
                state_io=0, set_io=0, led=0, channels=chans, gains=gains,
                num_scans=scans, trigger=0, trigger_state=0, trigger_on=1,
                rate=2048)
        for i in range(scans/16):
            for v in d.bulk_read(chans, gains):
                pass 
                #print v[3]
        d.bulk_stop()
        print scans/(time.time()-a)

        #print d.analog_input(0, False, True, (0,1,2,3), (0,0,0,0)) # 16ms
        #print d.analog_input(0, False, True, (8,9,10,11), (0,1,1,0)) # 16ms
        #print d.count(True), d.count(False)
        #for v in np.arange(0, 5.0001, .1):
        #    d.analog_output(0, 0, 0, 0, v, v, False, False)
        #    print v, d.analog_input(0, False, True, (8,9,8,9), (0,0,0,0))[3]

        # d.close()
