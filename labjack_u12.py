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

import usb, time, random, math
#import numpy as np


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
        # self.id = self.local_id()
        # self.vers = d.firmware_version()
        # assert self.vers >= 1.11
        # self.ser = self.serial()

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
        if hasattr(self, "handle"):
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

    def write(self, buf, tmo=20):
        return self.handle.interruptWrite(
                self.ep_out.address, buf, tmo)

    def read(self, siz, tmo=20):
        return self.handle.interruptRead(
                self.ep_in.address, siz, tmo)

    def writeread(self, w, tmo=20, wtmo=20):
        assert len(w) == 8
        assert self.write(w, wtmo) == 8
        r = self.read(8, tmo)
        return tuple(v&0xff for v in r)

    def read_mem(self, ad):
        assert ad >= 0 & ad <= 8188
        w = (0,0,0,0,0,0x50) + divmod(ad, 0x100)
        r = self.writeread(w)
        assert r[0] == 0x50
        assert w[6:] == r[6:]
        return r[1:5]

    def write_mem(self, ad, val):
        assert ad >= 0 & ad <= 0x1ffc
        assert len(val) == 4
        w = tuple(val) + (0,81) + divmod(ad, 0x100)
        r = self.writeread(w)
        assert r[0] == 81
        assert w[6:] == r[6:]

    def serial(self):
        r = self.read_mem(0)
        return sum(ri << 8*i for i, ri in enumerate(r[::-1]))

    def calibration(self):
        a = [self.read_mem(0x100+(0x010*j)) for j in range(8)]
        b = [self.read_mem(0x180+(0x010*j)) for j in range(4)]
        return [i[1] for i in a] + [i[3] for i in a] + [i[1] for i in b]

    def local_id(self):
        return self.read_mem(8)[3]

    def reset(self):
        return self.write((0,0,0,0, 0,0x5f,0,0))
        # no response

    def reenumerate(self):
        return self.write((0,0,0,0, 0,0x40,0,0))
        # no response

    def firmware_version(self):
        r = self.writeread((1,0,0,0, 0,0x53,0,0))
        return r[0]+r[1]/100.

    # digital lines on the SubD25
    conf_d = 0x0000
    state_d = 0x0000

    # the four main terminal digital ones
    conf_io = 0x0
    state_io = 0x0

    # analog outputs
    ao0 = 0.
    ao1 = 0.

    # analog inputs
    channels = (0,0,0,0) # selected channels
    gains = (1,1,1,1) # for the differential ones only

    def output(self, conf_d=0x0000, conf_io=0x0,
            state_d=0x0000, state_io=0x0, set_d_io=False, 
            ao0=0., ao1=0., set_ao=False,
            reset_c=False):
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

        w = list(divmod(conf_d, 0x100)) + \
            list(divmod(state_d, 0x100)) + \
            [(conf_io << 4) | state_io, 0,
                ao0 >> 2, ao1 >> 2]
        if set_ao:
            w[5] = ((set_d_io << 4) | (reset_c << 5) | 
                    ((ao0 & 0x3) << 2) | ((ao1 & 0x3) << 0))
        else:
            w[5] = 0x57

        r = self.writeread(w, tmo=20)
        
        assert not r[0] & (1 << 7)
        state_d = (r[1] << 8) + r[2]
        state_io = r[3] >> 4
        count = sum((v << i*8) for i,v in enumerate(r[:3:-1]))
        return state_d, state_io, count

    gains = [1, 2, 4, 5, 8, 10, 16, 20]

    def mux_cmd(self, ch, g):
        g = int(round(g))
        assert g in self.gains
        assert (ch >= 8) | (g == 1)
        return (self.gains.index(g) << 4) | (ch^0x8)

    def apply_calibration(self, ch, g, v):
        if ch < 8:
            ch &= 0x7
            off = self.caldata[ch]
            v -= off
            gaincal = self.caldata[ch + 8]
            v += (v-0x800)/512. * (off-gaincal)
        else:
            ch &= 0x3
            czse = self.caldata[2*ch] - self.caldata[2*ch+1]
            off = g*czse/2. + self.caldata[ch+16] - czse/2.
            v -= off
            ccdiff = (self.caldata[2*ch+8]-self.caldata[2*ch]) - \
                    (self.caldata[2*ch+9]-self.caldata[2*ch+1])
            if ccdiff >= 2:
                v -= (v-0x800)/256.
            elif ccdiff <= -2:
			    v += (v-0x800)/256.
        return max(min(v, 0x1000-1), 0)

    def bits_to_volts(self, ch, g, b):
        if ch < 8:
            return b*20./0x1000 - 10.
        else:
            return (b*40./0x1000 - 20.) / g

    def build_ai_command(self, cmd, channels, gains,
            state_io=0x0, set_io=False, led=False,
            rate=1200, feature_reports=True, read_counter=False,
            num_scans=1024,
            trigger=0, trigger_state=False, trigger_on=True):
        assert len(channels) in (1,2,4) # TODO: 1,2 not implemented
        assert len(channels) == len(gains)
        assert 0 <= state_io <= 0xf
        sample_int = int(round(6e6/rate/4.))
        if sample_int == 732:
            sample_int = 733
        assert 733 <= sample_int < (1<<14)
        challenge = random.randint(0,0xff)
        w = [self.mux_cmd(ch, g) for ch, g in zip(channels, gains)] + \
            [(led << 0) | (set_io << 1),
                 (1<<7) | (state_io << 0) | (cmd << 4)] + \
            list(divmod(sample_int, 1<<8))
        if cmd == 1: # stream
            w[4] |= (feature_reports << 7) | (read_counter << 6)
        elif cmd == 2: # burst
            w[4] |= ((int(10-math.ceil(math.log(num_scans, 2))) << 5) | 
                        (trigger_state << 2) | (trigger << 3))
            w[6] |= (feature_reports << 7) | (trigger_on << 6)
        return w

    def parse_ai_response(self, r, channels, gains):
        assert r[0] & (1 << 7)
        ofchecksum, overvoltage = bool(r[0] & (1<<5)), bool(r[0] & (1<<4))
        state_io = r[0] & 0xf
        iterations = r[1] >> 5
        backlog = r[1] & 0x1f
        bits = (((r[2] & 0xf0) << 4) + r[3],
                ((r[2] & 0x0f) << 8) + r[4],
                ((r[5] & 0xf0) << 4) + r[6],
                ((r[5] & 0x0f) << 8) + r[7])
        volts = [self.bits_to_volts(c, g, self.apply_calibration(c, g, v))
                for c, g, v in zip(channels, gains, bits)]
        count = sum(v<<i*8 for i,v in enumerate(r[:3:-1]))
        return (volts, state_io, count, overvoltage, ofchecksum, 
                iterations, backlog)

    def input(self, channels, gains, **kwargs):
        challenge = random.randint(0,0xff)
        w = self.build_ai_command(cmd=4, channels=channels, gains=gains,
                **kwargs)
        w[7] = challenge
        r = self.writeread(w, 20)
        assert r[1] == challenge
        return self.parse_ai_response(r, channels, gains)

    def stream(self, **kwargs):
        w = self.build_ai_command(cmd=1, **kwargs)
        self.write(w)

    def burst(self, **kwargs):
        w = self.build_ai_command(cmd=2, **kwargs)
        self.write(w)
   
    def bulk_read(self, channels, gains):
        resp = self.feature_read()
        while len(resp) >= 8:
            r, resp = resp[:8], resp[8:]
            yield self.parse_ai_response(r, channels, gains)
        
    def bulk_stop(self):
        self.read_mem(0)

    def count(self, reset=False, strobe=False):
        w = ((reset & 1) | (strobe & 2), 0, 0, 0, 0, 82, 0, 0)
        a = time.time()
        r = self.writeread(w, 20)
        t = (time.time()+a)/2.
        count = sum((v << i*8) for i,v in enumerate(r[:3:-1]))
        return count, t

    def pulse(self, t1, t2, lines, num_pulses, clear_first=False):
        assert 0x0 <= lines <= 0xf
        assert 1 <= num_pulses < 0xa000
        y1, y2 = t1*6e6-100, t2*6e6-100
        assert 126 <= y1 <= 5*255+121*255*255
        assert 126 <= y2 <= 5*255+121*255*255
        c1, c2 = max(1,int(y1/121/256)), max(1,int(y2/121/256))
        b1 = max(1,int(round((y1 - 5*c1)/(121*c1))))
        b2 = max(1,int(round((y2 - 5*c2)/(121*c2))))
        t1, t2 = (100+5*c1+121*b1*c1)/6e6, (100+5*c2+121*b2*c2)/6e6
        w = (b1, c1, b2, c2, lines, 0x64, (clear_first << 7) | 
                (num_pulses >> 8), (num_pulses & 0xff))
        r = self.writeread(w, 20) #int(20+1e3*(t1+t2)*num_pulses))
        errmask = r[4]
        return errmask, t1, t2

def main():
    for d in LabjackU12.find_all():
        # print d
        # print d.reset()
        # print [d.read_mem(i) for i in range(0, 8188, 4)]
        print d.serial()
        print d.local_id()
        print d.calibration()
        print d.firmware_version()

        d.output(ao0=1, ao1=2, set_ao=True)
        chans, gains, scans = (8,9,8,9), (1,1,10,10), 1024
        a = time.time()
        #d.stream(channels=chans, gains=gains, rate=420)
        d.burst(channels=chans, gains=gains, num_scans=scans, rate=2048)
        for i in range(scans/16):
            # time.sleep(0.04)
            for v in d.bulk_read(chans, gains):
                pass 
                #print v #v[1]
        d.bulk_stop()
        print scans/(time.time()-a)

        print d.input((0,1,2,3), (1,1,1,1)) # 16ms
        print d.input((8,9,10,11), (10,10,10,10)) # 16ms

        print d.count(True), d.count(False)
        for v in range(0, 6, 1):
            d.output(ao0=v, ao1=v, set_ao=True)
            print v, d.input((0,1,2,3), (1,1,1,1))[0]

        print d.pulse(.1231, .0002063, lines=0xf, num_pulses=100)

        del d

if __name__ == "__main__":
    main()
