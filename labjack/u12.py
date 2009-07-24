#!/usr/bin/python
# -*- coding: utf8 -*-
#
# Labjack U12 driver
# Copyright 2008, 2009 Robert Jordens, Thomas Uehlinger
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

class LabjackU12(object):
    id_vendor = 0x0cd5
    id_product = 0x0001
    id_interface = 0
    id_configuration = 0

    @classmethod
    def find_all(cls):
        """
        find all connected Labjack U12 devices on all busses
        and yields a LabjackU12 object for each
        """
        for bus in usb.busses():
            for dev in bus.devices:
                if (dev.idVendor, dev.idProduct) == (
                        cls.id_vendor, cls.id_product):
                    yield cls(dev)
    
    @classmethod
    def first(cls):
        return cls.find_all().next()
        
    def __init__(self, usbdev):
        """
        initialize a LabjackU12 device from usb device descriptor usbdev
        """
        self.dev = usbdev
        self.open()
        self.init_read()
        assert self.firmware_version() >= 1.10
        self.caldata = self.calibration()

    def open(self):
        conf = self.dev.configurations[self.id_configuration]
        intf = conf.interfaces[self.id_interface][0]
        self.handle = self.dev.open()
        try:
            self.handle.detachKernelDriver(intf)
        except usb.USBError, e:
            pass
        self.handle.setConfiguration(conf)
        self.handle.claimInterface(intf)
        self.ep_in = intf.endpoints[0]
        self.ep_out = intf.endpoints[1]
        assert self.ep_in.address == 0x81
        assert self.ep_out.address == 0x02
        assert self.ep_in.type == usb.ENDPOINT_TYPE_INTERRUPT
        assert self.ep_out.type == usb.ENDPOINT_TYPE_INTERRUPT
        
    def init_read(self):
        """
        perform the initial dummy read necessary
        """
        assert self.write((0,)*8) == 8
        try:
            return self.read(8, tmo=20)
        except usb.USBError:
            pass

    def close(self):
        if hasattr(self, "handle"):
            self.handle.releaseInterface()
            del self.handle

    def feature_read(self, tmo=5000):
        return self.handle.controlMsg(
            requestType=usb.TYPE_CLASS|usb.RECIP_INTERFACE|usb.ENDPOINT_IN,
            request=usb.REQ_CLEAR_FEATURE,
            value=(0x03<<8)+0x00, index=0, buffer=128, 
            timeout=tmo)

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
        """
        reads the non-volatile memory at address ad
        returns four bytes from that address
        """
        assert ad >= 0 & ad <= 8188
        w = (0,0,0,0,0,0x50) + divmod(ad, 0x100)
        r = self.writeread(w)
        assert r[0] == 0x50
        assert w[6:] == r[6:]
        return r[1:5]

    def write_mem(self, ad, val):
        """
        writes the four bytes from val to the non-volatile
        memory at address ad
        """
        assert ad >= 0 & ad <= 0x1ffc
        assert len(val) == 4
        w = tuple(val) + (0,81) + divmod(ad, 0x100)
        r = self.writeread(w)
        assert r[0] == 81
        assert w[6:] == r[6:]

    def reset(self):
        """
        causes the device to reset itself ending in re-enumeration
        and thus invalidation of self.handle
        """
        self.write((0,0,0,0, 0,0x5f,0,0))
        self.close()

    def reenumerate(self):
        """
        causes reenumeration without reset
        iunvalidates self.handle
        """
        self.write((0,0,0,0, 0,0x40,0,0))
        self.close()

    def firmware_version(self):
        """
        reads and returns the firmware version in floating point 
        e.g. 1.10
        """
        return self.watchdog(get_fw=True)

    def serial(self):
        """
        Return the unique serial number of the Labjack device.
        """
        r = self.read_mem(0)
        return sum(ri << 8*i for i, ri in enumerate(r[::-1]))

    def local_id(self):
        """
        reads and returns the local id that can be assigned
        to distinguish different labjack boards connected to the same
        bus and possibly enumerated in different sequences
        """
        return self.read_mem(8)[3]

    def calibration(self):
        """
        reads and returns the calibration array:
        eight bytes for the channel offset corrections
        eight bytes for the channel gain corrections
        four bytes used in differential mode
        """
        a = [self.read_mem(0x100+(0x010*j)) for j in range(8)]
        b = [self.read_mem(0x180+(0x010*j)) for j in range(4)]
        return [i[1] for i in a] + [i[3] for i in a] + [i[1] for i in b]

    clock = 6e6

    def watchdog(self, timeout=None, d0=None, d1=None, d8=None,
            do_reset=False, get_fw=False):
        """
        controls the watchdog function
        if timeout is None: only returns the firmware version
        after timeout seconds of no communication with the host,
        do_reset the device and set d0, d1, d8 to the state specified
        (must be configured as outputs before)
        careful: small timeouts cause permanent reset and prevent
        deactivation of the watchdog!
        """
        ncyc = max(1, int(round((timeout or 0)*self.clock/(1<<16))))
        w = (bool(get_fw), 0, 0, 0,
                ((d0 is not None)<<7) | (bool(d0)<<6) |
                ((d1 is not None)<<5) | (bool(d1)<<4) |
                ((d8 is not None)<<3) | (bool(d8)<<2) |
                (do_reset<<1) | ((timeout is not None)<<0), 0x53) + \
                        divmod(ncyc, 0x100)
        r = self.writeread(w)
        assert r[2:] == w[2:]
        if get_fw:
            # returns firmware version
            return r[0]+r[1]/100.

    gains = [1, 2, 4, 5, 8, 10, 16, 20]

    def mux_cmd(self, ch, g):
        g = int(round(g))
        assert g in self.gains
        assert (ch >= 8) | (g == 1)
        return (self.gains.index(g) << 4) | (ch^0x8)

    def apply_calibration(self, ch, g, v):
        if not ch & 0x8:
            off = self.caldata[ch]
            v -= off
            gaincal = self.caldata[ch+8]
            v += (v-0x800)/512. * (off-gaincal)
        else:
            ch &= 0x3
            czse = self.caldata[2*ch] - self.caldata[2*ch+1]
            off = (g-1)*czse/2. + self.caldata[ch+16]
            v -= off
            ccdiff = (self.caldata[2*ch+8]-self.caldata[2*ch]) - \
                    (self.caldata[2*ch+9]-self.caldata[2*ch+1])
            if ccdiff >= 2:
                v -= (v-0x800)/256.
            elif ccdiff <= -2:
                v += (v-0x800)/256.
        return max(min(v, 0x1000-1), 0)
    
    def ao_volts_to_bits(self, v):
        return int(round(v*1023/5.))
    
    def ai_bits_to_volts(self, ch, g, b):
        if not ch & 0x8:
            return b*20./0x1000 - 10.
        else:
            return (b*40./0x1000 - 20.) / g

    def build_ai_command(self, cmd, channels, gains,
            state_io=None, led=True,
            rate=1200, feature_reports=True, read_counter=False,
            num_scans=1024,
            trigger=0, trigger_state=False):
        """
        build an analog input command
        sample channels (0-7 for single-ended and 8-11 for differential)
        with gains (may only be uneq 1 for differential channels)
        channels and gains are four-tuples of (possibly identical) integers
        set the io lines to state_io before sampling
        use the led (flashes in some way) if led
        for burst and stream, do the scans at rate
        use feature_reports for burst and stream
        alternatively do not sample the analog inputs but sample the
            counter (read_counter) in stream mode
        do num_scans (rounded up to a power of two) in burst mode
        trigger on io0 (trigger=1) or io1 (trigger=2) 
            reaching trigger_state
        """
        #assert len(channels) in (1,2,4) # TODO: 1,2 not implemented
        assert len(channels) == 4
        assert len(channels) == len(gains)
        if state_io is not None:
            assert 0 <= state_io <= 0xf
            self.state_io = state_io
        sample_int = int(round(self.clock/rate/4.))
        if sample_int == 732:
            sample_int = 733
        assert 733 <= sample_int <= 0xffff
        challenge = random.randint(0,0xff)
        w = [self.mux_cmd(ch, g) for ch, g in zip(channels, gains)] + \
            [(led << 0) | ((state_io is not None) << 1),
                 (1<<7) | (self.state_io << 0) | (cmd << 4)] + \
            list(divmod(sample_int, 1<<8))
        if cmd == 1: # stream
            assert not trigger
            w[4] |= (feature_reports << 7) | (read_counter << 6)
        elif cmd == 2: # burst
            assert sample_int < (1<<14)
            assert not read_counter
            w[4] |= ((int(10-math.ceil(math.log(num_scans, 2))) << 5) | 
                        (trigger_state << 2) | ((trigger & 0x3) << 3))
            w[6] |= (feature_reports << 7) | (bool(trigger) << 6)
        elif cmd == 4: # single
            assert not read_counter
            assert not trigger
        return w

    def parse_ai_response(self, r, channels, gains):
        """
        parse analog input or stream/burst scan response
        for channels with gains
        return voltages of the channels, state_io of the io lines, the
        counter value (sensible if read_counter was set on the command),
        overvoltage and overflow/checksum error conditions, iteration
        number and backlog value
        """
        assert r[0] & (1 << 7)
        ofchecksum = bool(r[0] & (1<<5))
        overvoltage = bool(r[0] & (1<<4))
        state_io = r[0] & 0xf
        iterations = r[1] >> 5
        backlog = r[1] & 0x1f
        bits = (((r[2] & 0xf0) << 4) + r[3],
                ((r[2] & 0x0f) << 8) + r[4],
                ((r[5] & 0xf0) << 4) + r[6],
                ((r[5] & 0x0f) << 8) + r[7])
        volts = [self.ai_bits_to_volts(c, g,
                    self.apply_calibration(c, g, v))
                        for c, g, v in zip(channels, gains, bits)]
        count = sum(v<<i*8 for i,v in enumerate(r[:3:-1]))
        return (volts, state_io, count, overvoltage, ofchecksum, 
                iterations, backlog)

    # State management

    # digital lines on the SubD25
    conf_d = 0x0000
    state_d = 0x0000

    # the four main terminal digital ones
    conf_io = 0x0
    state_io = 0x0

    # analog outputs
    ao0 = 0.
    ao1 = 0.

    def output(self, conf_d=None, conf_io=None,
            state_d=None, state_io=None,
            ao0=None, ao1=None,
            reset_counter=False):
        """
        Configure outputs:
        conf_d: I/O configuration bitmask of D0..D15 (0=input, 1=output)
        conf_io: I/O configuration bitmask of IO0..IO3 (0=input, 1=output)
        state_d: output bitmask of D0..D15
        state_io: output bitmask of IO0..IO3
        ao0..ao1: analoge output voltage in the range 0..+5 V
        reset_c: whether to reset the counter after reading it
        Returns: 
            state_d state_io (bitfield of input states)
            counter value
        """ 
        set_d_io = state_d is not None or state_io is not None
        set_ao = ao0 is not None or ao1 is not None

        if conf_d is not None:
            assert 0 <= conf_d <= 0xffff
            self.conf_d = conf_d
        if conf_io is not None:
            assert 0 <= conf_io <= 0xf
            self.conf_io = conf_io
        if state_d is not None:
            assert 0 <= state_d <= 0xffff
            self.state_d = state_d
        if state_io is not None:
            assert 0 <= state_io <= 0xf
            self.state_io = state_io       
        if ao0 is not None:
            assert 0 <= ao0 <= 5
            self.ao0 = ao0
        if ao1 is not None:
            assert 0 <= ao1 <= 5
            self.ao1 = ao1

        ao0 = self.ao_volts_to_bits(self.ao0)
        ao1 = self.ao_volts_to_bits(self.ao1)

        w = list(divmod(self.conf_d^0xffff, 0x100)) + \
            list(divmod(self.state_d, 0x100)) + \
            [((self.conf_io^0xf) << 4) | self.state_io, 0,
                ao0 >> 2, ao1 >> 2]
        if set_ao or reset_counter:
            w[5] = ((set_d_io << 4) | (reset_counter << 5) | 
                    ((ao0 & 0x3) << 2) | ((ao1 & 0x3) << 0))
        else:
            w[5] = 0x57
            w[6] = (set_d_io << 0)
        r = self.writeread(w, tmo=20)
        
        assert not r[0] & (1 << 7)
        state_d = (r[1] << 8) + r[2]
        state_io = r[3] >> 4
        count = sum((v << i*8) for i,v in enumerate(r[:3:-1]))
        return state_d, state_io, count

    def input(self, channels, gains = (1,1,1,1), **kwargs):
        """
        channels: a tuple of 4 channel numers in the range (0..7 
            for single ended channels or 8..11 for differential) to read
        gains: A tuple of 4 gains (one of [1: +-20 V, 2: +-10 V, 4: +-5 V,
            5: +-4 V, 8: +-2.5V, 10: +-2 V, 16: +-1.25 V, 20: +-1 V]). 
            Only applicable for differential channels. In a single ended
            measurement the range is always -10..+10 V.
        keyword arguments: see build_ai_command()
        
        Returns (volts, state_io, count, overvoltage, ofchecksum, 
                iterations, backlog).
        """
        challenge = random.randint(0,0xff)
        w = self.build_ai_command(cmd=4, channels=channels, gains=gains,
                **kwargs)
        w[7] = challenge
        r = self.writeread(w, 20)
        assert r[1] == challenge
        return self.parse_ai_response(r, channels, gains)

    def input_single(self, channel, gain=1):
        r = self.input((channel,)*4, (gain,)*4)
        return sum(r[0])/4.
         
    def stream(self, **kwargs):
        """
        start a streaming sampling
        see build_ai_command for parameters
        results must be read with bulk_read or can be aborted with any
        other command, esp. bulk_stop
        """
        w = self.build_ai_command(cmd=1, **kwargs)
        self.write(w)

    def burst(self, **kwargs):
        """
        start a burst sampling
        see build_ai_command for parameters
        results must be read with bulk_read or can be aborted with any
        other command, esp. bulk_stop
        """
        w = self.build_ai_command(cmd=2, **kwargs)
        self.write(w)
   
    def bulk_read(self, channels, gains, **kwargs):
        """
        read 16 scans and yield them
        see parse_ai_response for format
        """
        resp = self.feature_read(**kwargs)
        while len(resp) >= 8:
            r, resp = resp[:8], resp[8:]
            yield self.parse_ai_response(r, channels, gains)
        
    def bulk_stop(self):
        """
        cancels a streaming or burst acquisition
        """
        self.read_mem(0)

    def stream_sync(self, channels, gains, num_scans, rate, **kwargs):
        """
        initiate a synchronous streaming scan of channels at gains for
        num_scans at rate
        returns if all data has been acquired but yields data as it
        flows in
        see build_ai_command for parameters
        """
        self.stream(channels=channels, gains=gains,
                rate=rate, **kwargs)
        for i in range(int(math.ceil(num_scans/16.))):
            for v in self.bulk_read(channels, gains,
                    tmo=40+16*1000/rate):
                yield v
        self.bulk_stop()

    def burst_sync(self, channels, gains, num_scans, rate,
            trigger_timeout=3, **kwargs):
        """
        initiate a burst scan of channels at gains for
        num_scans at rate
        returns if all data has been acquired but yields data as it
        flows in after completion of the burst acquisition
        see build_ai_command for parameters
        trigger must occur within trigger_timeout, otherwise raise an
        usb timeout
        """
        self.burst(channels=channels, gains=gains,
                num_scans=num_scans, rate=rate, **kwargs)
        time.sleep(num_scans/float(rate)-.02)
        for i in range(int(math.ceil(num_scans/16.))):
            for v in self.bulk_read(channels, gains,
                    tmo=40+(i==0)*trigger_timeout*1000):
                yield v
        self.bulk_stop()

    def count(self, reset=False, strobe=False):
        """
        Read the counter.
        reset: whether to reset the counter to zero after reading it
        strobe: whether to activate the strobe (STB) output
        """
        w = ((reset & 1) | (strobe & 2), 0, 0, 0, 0, 82, 0, 0)
        a = time.time()
        r = self.writeread(w, 20)
        t = (time.time()+a)/2.
        count = sum((v << i*8) for i,v in enumerate(r[:3:-1]))
        return count, t

    def pulse(self, t1, t2, lines, num_pulses, clear_first=False):
        """
        Output num_pulses pulse with off-time t1 and on-time t2 
            on the digital lines D0..D7.
        t1: off-time
        t2: on-time
        lines: bitmask defining on which of D0..D7 the pulses should be output
        num_pulses: number of pulses to output
        clear_first: whether to clear the digital lines first
        """
        assert 0x00 <= lines <= 0xff
        assert 1 <= num_pulses < 0xa000
        y1, y2 = t1*self.clock-100, t2*self.clock-100
        assert 126 <= y1 <= 5*255+121*255*255
        assert 126 <= y2 <= 5*255+121*255*255
        c1, c2 = max(1,int(y1/121/256)), max(1,int(y2/121/256))
        b1 = max(1,int(round((y1 - 5*c1)/(121*c1))))
        b2 = max(1,int(round((y2 - 5*c2)/(121*c2))))
        assert 1 <= b1 <= 256
        assert 1 <= c1 <= 256
        assert 1 <= b2 <= 256
        assert 1 <= c2 <= 256
        t1 = (100+5*c1+121*b1*c1)/self.clock
        t2 = (100+5*c2+121*b2*c2)/self.clock
        w = (b1, c1, b2, c2, lines, 0x64, (clear_first << 7) | 
                (num_pulses >> 8), (num_pulses & 0xff))
        r = self.writeread(w, int(20+1e3*(t1+t2)*num_pulses))
        errmask = r[4]
        return errmask, t1, t2

