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
        self.handle = self.dev.open()
        # self.handle.reset()
        conf = self.dev.configurations[self.id_configuration]
        self.interface = conf.interfaces[self.id_interface][0]
        try:
            self.handle.detachKernelDriver(self.interface)
        except usb.USBError:
            # already detached
            pass
        self.handle.setConfiguration(conf)
        self.handle.claimInterface(self.interface)
        self.ep_in = self.interface.endpoints[0]
        self.ep_out = self.interface.endpoints[1]
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

    def serial(self):
        """
        reads and returns the serial number of the labjack device
        """
        r = self.read_mem(0)
        return sum(ri << 8*i for i, ri in enumerate(r[::-1]))

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

    def local_id(self):
        """
        reads and returns the local id that can be assigned
        to distinguish different labjack boards connected to the same
        bus and possibly enumerated in different sequences
        """
        return self.read_mem(8)[3]

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
        return self.watchdog(timeout=None)

    # general properties

    clock = 6e6

    # State management

    # digital lines on the SubD25
    conf_d = 0x0000
    state_d = 0x0000

    # the four main terminal digital ones
    conf_io = 0x0
    state_io = 0x0

    # analog outputs
    ao0 = 0
    ao1 = 0

    def watchdog(self, timeout=None, d0=None, d1=None, d8=None,
            do_reset=False):
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
        w = ((timeout is None)<<0, 0, 0, 0,
                ((d0 is not None)<<7) | (bool(d0)<<6) |
                ((d1 is not None)<<5) | (bool(d1)<<4) |
                ((d8 is not None)<<3) | (bool(d8)<<2) |
                (do_reset<<1) | ((timeout is not None)<<0), 0x53) + \
                        divmod(ncyc, 0x100)
        r = self.writeread(w)
        assert r[2:] == w[2:]
        # returns firmware version
        return r[0]+r[1]/100.

    def output(self, conf_d=None, conf_io=None,
            state_d=None, state_io=None,
            ao0=0., ao1=0., set_ao=False,
            reset_counter=False):
        """
        Configure outputs:
            conf_d and conf_io: bitfields of high: output and low: input
        Set outputs if either state_d or state_io are uneq None:
            state_d and state_io: bitfields out output state
        Set analog outputs if set_ao:
            voltages ao0 and ao1
        Reset the counter if reset_counter
        Returns: 
            state_d state_io (bitfield of input states)
            counter value
        """ 
        set_d_io = False
        if conf_d is not None:
            assert 0 <= conf_d <= 0xffff
            self.conf_d = conf_d
        if conf_io is not None:
            assert 0 <= conf_io <= 0xf
            self.conf_io = conf_io
        if state_d is not None:
            assert 0 <= state_d <= 0xffff
            self.state_d = state_d
            set_d_io = True
        if state_io is not None:
            assert 0 <= state_io <= 0xf
            self.state_io = state_io
            set_d_io = True
        if set_ao:
            assert 0 <= ao0 <= 5
            assert 0 <= ao1 <= 5
            self.ao0 = int(round(ao0/5.*1023))
            self.ao1 = int(round(ao1/5.*1023))

        w = list(divmod(self.conf_d^0xffff, 0x100)) + \
            list(divmod(self.state_d, 0x100)) + \
            [((self.conf_io^0xf) << 4) | self.state_io, 0,
                self.ao0 >> 2, self.ao1 >> 2]
        if set_ao:
            w[5] = ((set_d_io << 4) | (reset_counter << 5) | 
                    ((self.ao0 & 0x3) << 2) | ((self.ao1 & 0x3) << 0))
        else:
            assert not reset_counter
            w[5] = 0x57
            w[6] = (set_d_io << 0)
        r = self.writeread(w, tmo=20)
        
        assert not r[0] & (1 << 7)
        state_d = (r[1] << 8) + r[2]
        state_io = r[3] >> 4
        count = sum((v << i*8) for i,v in enumerate(r[:3:-1]))
        return state_d, state_io, count

    gains = [1, 2, 4, 5, 8, 10, 16, 20]

    def mux_cmd(self, ch, g):
        assert (ch >= 8) | (g == 1)
        assert g in self.gains
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
            state_io=0x0, set_io=False, led=True,
            rate=1200, feature_reports=True, read_counter=False,
            num_scans=1024,
            trigger=0, trigger_state=False):
        """
        build an analog input command
        sample channels (0-7 for single-ended and 8-11 for differential)
        with gains (may only be uneq 1 for differential channels)
        channels and gains are four-tuples of (possibly identical) integers
        if set_io also set the io lines to state_io before sampling
        use the led (flashes in some way) if led
        for burst and stream, do the scans at rate
        use feature_reports for burst and stream
        alternatively do not sample the analog inputs but sample the
        counter (read_counter) in stream mode
        do num_scans (rounded to a power of two) in burst mode
        trigger on io0 (1) or io1 (2) reaching trigger_state if trigger uneq 0
        """
        assert len(channels) in (1,2,4) # TODO: 1,2 not implemented
        assert len(channels) == len(gains)
        if set_io:
            assert 0 <= state_io <= 0xf
            self.state_io = state_io
        sample_int = int(round(self.clock/rate/4.))
        if sample_int == 732:
            sample_int = 733
        assert 733 <= sample_int <= 0xffff
        challenge = random.randint(0,0xff)
        w = [self.mux_cmd(ch, g) for ch, g in zip(channels, gains)] + \
            [(led << 0) | (set_io << 1),
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
        elif cmd == 4:
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
        """
        sample channels at gains
        (see build_ai_command for parameters and build_ai_response for
        return values)
        """
        challenge = random.randint(0,0xff)
        w = self.build_ai_command(cmd=4, channels=channels, gains=gains,
                **kwargs)
        w[7] = challenge
        r = self.writeread(w, 20)
        assert r[1] == challenge
        return self.parse_ai_response(r, channels, gains)

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
        read 16 scans and yield them as a generator
        see parse_qi_response for format
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
                    tmo=20+16*1000/rate):
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
        read and return counter optionally resetting it and strobing the
        stb line
        """
        w = ((reset & 1) | (strobe & 2), 0, 0, 0, 0, 82, 0, 0)
        a = time.time()
        r = self.writeread(w, 20)
        t = (time.time()+a)/2.
        count = sum((v << i*8) for i,v in enumerate(r[:3:-1]))
        return count, t

    def pulse(self, t1, t2, lines, num_pulses, clear_first=False):
        """
        generate num_pulses with t1 high and t2 low times on d lines
        lines, optionally clearing them first
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


