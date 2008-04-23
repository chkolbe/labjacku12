import unittest
import time
import numpy
from labjack_u12 import LabjackU12

class LabjackU12Tests(unittest.TestCase):
    def setUp(self):
        self.l = LabjackU12.find_all().next()

    def tearDown(self):
        # self.l.close()
        del self.l

    def test_basics(self):
        self.l.serial()
        self.l.local_id()
        self.l.calibration()
        self.failUnlessEqual(self.l.firmware_version(), 1.10)

        # print d.reset()
        # print [d.read_mem(i) for i in range(0, 8188, 4)]

        #print d.watchdog(ignore=False, timeout=30,
        #    do_reset=True, active=False)
        #time.sleep(29)

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

    def test_io_in_out(self):
        """
        connect io0 and io1
        """
        self.l.output(conf_io=0x1, state_io=0x0)
        for i in range(10):
            state_d, state_io, count = self.l.output(state_io=0x1)
            self.failUnless(state_io & 0x2)
            state_d, state_io, count = self.l.output(state_io=0x0)
            self.failUnless(not state_io & 0x2)

    def test_analog_in_out(self):
        """
        requires ao0 to be connected with ai0, ai1
        and ao1 with ai2, ai3
        """
        for v in range(0, 6, 1):
            self.l.output(ao0=v, ao1=v, set_ao=True)
            r = self.l.input(channels=(8,8,9,9), gains=(1,4,1,4))[0]
            for i in r:
                self.failUnless(abs(v-i) < .1,
                        "measured %g for %g" % (i, v))

    def test_single_analog_in_cal_5v(self):
        """
        requires ai4 to be connected with cal and ai5 with gnd
        requires ai6 to be connected with 5v and ai7 with gnd
        """
        for s,c,e in [(5, 6, .1), (0, 7, .05), (2.5, 4, .05), (0, 5, .05)]:
            for v in self.l.input(channels=(c,c,c,c), gains=(1,1,1,1))[0]:
                self.failUnless(abs(s-v) < e,
                        "%g is not %g, channel %g" % (v, s, c))

    def test_diff_analog_in_cal_5v(self):
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

    def test_burst(self):
        """
        requires ai4 to be connected with cal and ai5 with gnd
        """
        chans, gains, scans = (10,10,10,10), (1,2,4,5), 1024
        a = time.time()
        for v in self.l.burst_sync(channels=chans, gains=gains,
                num_scans=scans, rate=2048):
            for r in v[0]:
                self.failUnless(abs(r-2.5) < .1,
                   "%s should be cal, 2.5v" % v[0])
        scans_sec = 4*scans/(time.time()-a)
        self.failUnless(scans_sec > 3000,
                "scan took too long (%g Hz)" % scans_sec)

    def test_stream(self):
        """
        requires ai4 to be connected with cal and ai5 with gnd
        """
        chans, gains, scans = (10,10,10,10), (1,2,4,5), 1024
        a = time.time()
        for v in self.l.stream_sync(channels=chans, gains=gains, 
                num_scans=scans, rate=400):
            for r in v[0]:
                self.failUnless(abs(r-2.5) < .1,
                        "%s should be cal, 2.5v" % v[0])
        scans_sec = 4*scans/(time.time()-a)
        self.failUnless(scans_sec > 1200,
                "scan took too long (%g Hz)" % scans_sec)

    def test_counter(self):
        self.l.count(reset=True)
        c,t = self.l.count(reset=False)
        self.failUnlessEqual(c, 0, "counter is %g" % c)

    def test_io_counter(self):
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

    def test_pulse_counter(self):
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
