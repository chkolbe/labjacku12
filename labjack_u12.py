#!/usr/bin/python

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

    def open(self):
        self.handle = self.dev.open()
        # self.handle.reset()
        self.interface = \
            self.dev.configurations[self.id_configuration
                    ].interfaces[self.id_interface][0]
        self.handle.detachKernelDriver(self.id_interface)
        self.handle.claimInterface(self.interface)
        self.ep_in = self.interface.endpoints[0]
        self.ep_out = self.interface.endpoints[1]
        assert self.ep_in.type == usb.ENDPOINT_TYPE_INTERRUPT
        assert self.ep_out.type == usb.ENDPOINT_TYPE_INTERRUPT

    def close(self):
        self.handle.releaseInterface()
        del self.handle

    def feature(self):
        return self.handle.controlMsg(
            requestType=usb.TYPE_CLASS|usb.RECIP_INTERFACE|usb.ENDPOINT_IN,
            request=usb.REQ_CLEAR_FEATURE,
            value=(0x03<<8)+0x00, index=0, buffer=128, 
            timeout=500)

    def write(self, buf, tmo=100):
        return self.handle.interruptWrite(
                self.ep_out.address, buf, tmo)

    def read(self, siz, tmo=100):
        return self.handle.interruptRead(
                self.ep_in.address, siz, tmo)

    def writeread(self, w, tmo=100):
        assert self.write(w) == 8
        try:
            r = self.read(8, tmo)
        except usb.USBError:
            time.sleep(0.01)
            assert self.write(w) == 8
            r = self.read(8, tmo)
        return tuple(v&0xff for v in r)

    def read_mem(self, ad):
        assert ad >= 0 & ad <= 8188
        w = (0,0,0,0,0,80) + divmod(ad, 1<<8)
        r = self.writeread(w)
        assert r[0] == 80
        assert w[6:] == r[6:]
        return r[1:5]

    def serial(self):
        r = self.read_mem(0)
        return sum(ri<<(8*i) for i, ri in enumerate(r[::-1]))

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
            (conf_io<<4) | state_io,
            (set_d<<4) | (reset_c<<5) | ((ao0&0x3)<<2) | ((ao1&0x3)<<0),
            ao0>>2, ao1>>2)
        r = self.writeread(w, tmo=1000)

        assert not r[0] & (1<<7)
        state_d = (r[1]<<8) + r[2]
        state_io = r[3]>>4
        count = sum((v<<(i*8) for i,v in enumerate(r[4:][::-1])))
        return state_d, state_io, count

    def analog_input(self, state_io, set_io, led, channels, gains):
        assert len(channels) in (1,2,4)
        assert 0 <= state_io <= 0xf
        c = np.random.bytes(1)
        c7 = 0
        cmd = 12
        w = (mux1, mux2, mux3, mux4, (led<<0) | (set_io<<1),
                (cmd<<4) | (state_io<<0), 0, c)


if __name__ == "__main__":
    for d in LabjackU12.find_all():
        print d
        d.open()
        #print d.feature()
        #print d.read_mem(0)
        print d.serial()
        print d.local_id()
        print d.calibration()
        print d.firmware_version()
        print d.analog_output(0, 0, 0, 0, 2, 2,
                False, False) # 16ms
        # print d.reset()
        d.close()
