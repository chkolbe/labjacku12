import unittest
import time
from labjack import u12

class LabjackU12Tests(unittest.TestCase):
    def setUp(self):
        self.l = u12.LabjackU12.first()

    def tearDown(self):
        self.l.close()

    def _test_reset(self):
        self.l.reset()
        time.sleep(20)

    def _test_reenumerate(self):
        self.l.reenumerate()
        time.sleep(20)

        # print [d.read_mem(i) for i in range(0, 8188, 4)]

    def test_usb_speed(self):
        n = 1000 # 0x1ffc
        a = time.time()
        for i in range(0,n,4):
            self.l.read_mem(i)
        t = (time.time()-a)/n*4
        self.failUnless(t<18e-3)

    def test_watchdog_off(self):
        self.l.watchdog(timeout=1)
        time.sleep(2)
        self.l.input_single(0, 1)
        self.l.watchdog(timeout=None)

    def _test_watchdog_on(self):
        self.l.watchdog(do_reset=True, timeout=25)
        time.sleep(24)
        self.l.input_single(0, 1)
        time.sleep(26)
        self.failUnlessRaises(u12.usb.USBError, 
                self.l.input_single, 0, 1)
        self.l = None
        while not self.l:
            try:
                self.setUp()
            except StopIteration:
                time.sleep(1)
        self.l.watchdog(timeout=None)

    def test_calibration(self):
        c = self.l.calibration()
        self.failUnlessEqual(len(c), 20)
        self.failUnlessEqual(c, self.l.caldata)

    def test_localid(self):
        i = self.l.local_id()
        # self.failUnlessEqual(i, 0)

    def test_serial(self):
        s = self.l.serial()

    def test_firmware_version(self):
        v = self.l.firmware_version()
        self.failUnlessEqual(v,1.10)

    def test_digital_out(self):
        state_d, state_io, count = self.l.output(
               conf_d=0x55aa, conf_io=0x5,
               state_d=0x5a5a, state_io=0x3)
        self.failUnlessEqual(state_d & 0x55aa & 0x5a5a,
                0x55aa & 0x5a5a)
        self.failUnlessEqual(~state_d & 0x55aa & ~0x5a5a,
                0x55aa & ~0x5a5a)
        self.failUnlessEqual(state_io & 0x5 & 0x3,
                0x5 & 0x3)
        self.failUnlessEqual(~state_io & 0x5 & ~0x3,
                0x5 & ~0x3)

    def test_io_in_out_loop(self):
        """
        connect io0 and io1
        """
        self.l.output(conf_io=0x1, state_io=0x0)
        for i in range(10):
            state_d, state_io, count = self.l.output(state_io=0x1)
            self.failUnless(state_io & 0x2)
            state_d, state_io, count = self.l.output(state_io=0x0)
            self.failUnless(not state_io & 0x2)

    def test_ao(self):
        self.l.output(ao0=1)
        self.failUnlessEqual(self.l.ao0, 1)
        self.l.output(ao1=1)
        self.failUnlessEqual(self.l.ao1, 1)
        self.l.output(ao1=0, ao0=0)
        self.failUnlessEqual(self.l.ao0, 0)
        self.failUnlessEqual(self.l.ao1, 0)

    def test_ai_se(self):
        for c in range(4):
            self.l.input(channels=(c,c+1,c+2,c+3), gains=(1,1,1,1))

    def test_ai_diff(self):
        for c in range(8,11):
            for gi in range(len(self.l.gains)-4):
                g = self.l.gains[gi:gi+4]
                self.l.input(channels=(c,c+1,c+2,c+3), gains=g)

    def test_ai_single_se(self):
        for c in range(8):
            self.l.input_single(c, 1)

    def test_ai_single_diff(self):
        for c in range(8, 12):
            for g in self.l.gains:
                self.l.input_single(c, g)

    def test_analog_in_out_loop(self):
        """
        requires ao0 to be connected with ai0, ai1
        and ao1 with ai2, ai3
        """
        for v in range(0, 6, 1):
            self.l.output(ao0=v, ao1=v)
            r = self.l.input(channels=(8,8,9,9), gains=(1,4,1,4))[0]
            for i in r:
                self.failUnless(abs(v-i) < .1,
                        "measured %g for %g" % (i, v))

    def test_single_analog_in_cal_5v_loop(self):
        """
        requires ai4 to be connected with cal and ai5 with gnd
        requires ai6 to be connected with 5v and ai7 with gnd
        """
        for s,c,e in [(5, 6, .1), (0, 7, .05), (2.5, 4, .05), (0, 5, .05)]:
            for v in self.l.input(channels=(c,c,c,c), gains=(1,1,1,1))[0]:
                self.failUnless(abs(s-v) < e,
                        "%g is not %g, channel %g" % (v, s, c))

    def test_diff_analog_in_cal_5v_loop(self):
        """
        requires ai4 to be connected with cal and ai5 with gnd
        requires ai6 to be connected with 5v and ai7 with gnd
        """
        for g in self.l.gains:
            for s,c,e in [(5, 11, .1), (2.5, 10, .03)]:
                v = self.l.input(channels=(c,c,c,c), gains=(g,g,g,g))
                r = v[0]
                if s*g > 20:
                    if s*g > 25:
                        self.failUnless(v[3],
                            "%s should be overvoltage (%g, %g)" % (v,s,g))
                    continue
                for i in r:
                    self.failUnless(abs(s-i) < e,
                        "%g is not %g, channel %g, gain %g" % (i,s,c,g))

    def test_burst_speed(self):
        chans, gains, scans, rate = (10,10,10,10), (1,2,4,5), 1024, 2048
        a = time.time()
        v = list(self.l.burst_sync(channels=chans, gains=gains,
                num_scans=scans, rate=rate))
        self.failUnlessEqual(len(v), scans)
        scans_sec = scans/(time.time()-a)
        self.failUnless(scans_sec > 500, # transferring the scan!
                "scan took too long (%g Hz)" % scans_sec)

    def test_burst_loop(self):
        """
        requires ai4 to be connected with cal and ai5 with gnd
        """
        chans, gains, scans, rate = (10,10,10,10), (1,2,4,5), 1024, 2048
        v = [v[0] for v in self.l.burst_sync(
                channels=chans, gains=gains,
                num_scans=scans, rate=rate)]
        for vi in v:
            for r in vi:
                self.failUnless(abs(r-2.5) < .1,
                    "%s should be cal, 2.5v" % vi[0])

    def test_stream_speed(self):
        chans, gains, scans, rate = (10,10,10,10), (1,2,4,5), 1024, 500
        a = time.time()
        v = list(self.l.stream_sync(channels=chans, gains=gains, 
                num_scans=scans, rate=rate))
        self.failUnlessEqual(len(v), scans)
        scans_sec = scans/(time.time()-a)
        self.failUnless(scans_sec > 290,
                "scan took too long (%g Hz)" % scans_sec)

    def test_stream_loop(self):
        """
        requires ai4 to be connected with cal and ai5 with gnd
        """
        chans, gains, scans, rate = (10,10,10,10), (1,2,4,5), 1024, 500
        v = [v[0] for v in self.l.stream_sync(
                channels=chans, gains=gains,
                num_scans=scans, rate=rate)]
        for vi in v:
            for r in vi:
                self.failUnless(abs(r-2.5) < .1,
                        "%s should be cal, 2.5v" % vi[0])

    def test_counter(self):
        self.l.count(reset=True)
        c,t = self.l.count(reset=False)
        self.failUnlessEqual(c, 0, "counter is %g" % c)

    def test_counter_reset_by_output(self):
        self.l.output(reset_counter=True)
        c,t = self.l.count(reset=False)
        self.failUnlessEqual(c, 0, "counter is %g" % c)

    def test_io_counter_loop(self):
        """
        connect io3 ant cnt
        """
        self.l.output(conf_io=0x8, state_io=0x0)
        self.l.count(reset=True)
        for i in range(10):
            self.l.output(state_io=0x8)
            self.l.output(state_io=0x0)
        c, t = self.l.count()
        self.failUnlessEqual(c, 9,
                "counted %g pulses" % c)

    def test_pulse_counter_loop(self):
        """
        connect d0 and cnt
        """
        self.l.output(conf_d=0x01, state_d=0x0)
        self.l.count(reset=True)
        t10, t20 = 0.001231, 0.002063
        errmask, t1, t2 = self.l.pulse(t1=t10, t2=t20,
                lines=0x01, num_pulses=100)
        self.failUnless(abs(t1-t10)/t10 < .1)
        self.failUnless(abs(t2-t20)/t20 < .1)
        c, t = self.l.count()
        self.failUnlessEqual(c, 100,
                "counted %g pulses" % c)

if __name__ == "__main__":
    unittest.main()
