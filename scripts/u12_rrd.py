#!/usr/bin/python

import sys, os, tempfile, time, numpy as np

from labjack_u12 import LabjackU12
import rrdtool

def log_to_rrd(rrd = "u12.rrd", inter = 1):
    if not os.path.exists(rrd):
        rrdtool.create(rrd, '--step', str(inter),
            'DS:ai0:GAUGE:60:U:U', 'DS:ai1:GAUGE:60:U:U',
            'DS:ai2:GAUGE:60:U:U', 'DS:ai3:GAUGE:60:U:U',
            'DS:ai4:GAUGE:60:U:U', 'DS:ai5:GAUGE:60:U:U',
            'DS:ai6:GAUGE:60:U:U', 'DS:ai7:GAUGE:60:U:U',

            'DS:io:GAUGE:60:0:15',
            'DS:d:GAUGE:60:0:65535',

            'DS:c:COUNTER:60:0:4294967295',

            'RRA:AVERAGE:0.9:1:512',
            'RRA:AVERAGE:0.9:10:512',
            'RRA:AVERAGE:0.9:60:512',
            'RRA:AVERAGE:0.9:600:512',
            'RRA:AVERAGE:0.9:3600:512',
            'RRA:AVERAGE:0.9:86400:512',

            'RRA:MIN:0.9:1:512', 'RRA:MAX:0.9:1:512',
            'RRA:MIN:0.9:10:512', 'RRA:MAX:0.9:10:512',
            'RRA:MIN:0.9:60:512', 'RRA:MAX:0.9:60:512',
            'RRA:MIN:0.9:600:512', 'RRA:MAX:0.9:600:512',
            'RRA:MIN:0.9:3600:512', 'RRA:MAX:0.9:3600:512',
            'RRA:MIN:0.9:86400:512', 'RRA:MAX:0.9:86400:512',
        )

    l = LabjackU12.find_all().next()

    while True:
        s = "N:"
        for i in range(8):
            v, io, c, ov = l.input(channels=(i,i,i,i),
                    gains=(1,1,1,1))[:4]
            s += (not ov and str(np.mean(v)) or "U") + ":"
        d, io, c = l.output(conf_d=0x00, conf_io=0x0)
        s += ":".join(map(str, (io, d, c)))
        rrdtool.update(rrd, s)
        time.sleep(inter)

def plot_rrd(rrd="u12.rrd", out="u12_"):
    for s in (1, 10, 60, 600, 3600, 86400):
        rrdtool.graph(out+str(s)+".png",
                  '--imgformat', 'PNG',
                  '--width', '400',
                  '--height', '200',
                  '--step', str(s),
                  '--start', 'now-'+str(s*200)+'s',
                  '--end', 'now',
                  '--vertical-label', 'V',
                  '--title', 'analog inputs',
                  #'--lower-limit', '-0.01',
                  #'--upper-limit', '0.01',

                  'DEF:ai0r=%s:ai0:AVERAGE' % rrd,
                  'DEF:ai1r=%s:ai1:AVERAGE' % rrd,
                  'DEF:ai2r=%s:ai2:AVERAGE' % rrd,
                  'DEF:ai3r=%s:ai3:AVERAGE' % rrd,
                  'DEF:ai4r=%s:ai4:AVERAGE' % rrd,
                  'DEF:ai5r=%s:ai5:AVERAGE' % rrd,
                  'DEF:ai6r=%s:ai6:AVERAGE' % rrd,
                  'DEF:ai7r=%s:ai7:AVERAGE' % rrd,

                  'CDEF:ai0=ai0r,1,*',
                  'CDEF:ai1=ai1r,1,*',
                  'CDEF:ai2=ai2r,1,*',
                  'CDEF:ai3=ai3r,1,*',
                  'CDEF:ai4=ai4r,1,*',
                  'CDEF:ai5=ai5r,1,*',
                  'CDEF:ai6=ai6r,1,*',
                  'CDEF:ai7=ai7r,1,*',

                  'VDEF:ai0avg=ai0,AVERAGE',
                  'VDEF:ai1avg=ai1,AVERAGE',
                  'VDEF:ai2avg=ai2,AVERAGE',
                  'VDEF:ai3avg=ai3,AVERAGE',
                  'VDEF:ai4avg=ai4,AVERAGE',
                  'VDEF:ai5avg=ai5,AVERAGE',
                  'VDEF:ai6avg=ai6,AVERAGE',
                  'VDEF:ai7avg=ai7,AVERAGE',

                  'VDEF:ai0max=ai0,MAXIMUM', 'VDEF:ai0min=ai0,MINIMUM',
                  'VDEF:ai1max=ai1,MAXIMUM', 'VDEF:ai1min=ai1,MINIMUM',
                  'VDEF:ai2max=ai2,MAXIMUM', 'VDEF:ai2min=ai2,MINIMUM',
                  'VDEF:ai3max=ai3,MAXIMUM', 'VDEF:ai3min=ai3,MINIMUM',
                  'VDEF:ai4max=ai4,MAXIMUM', 'VDEF:ai4min=ai4,MINIMUM',
                  'VDEF:ai5max=ai5,MAXIMUM', 'VDEF:ai5min=ai5,MINIMUM',
                  'VDEF:ai6max=ai6,MAXIMUM', 'VDEF:ai6min=ai6,MINIMUM',
                  'VDEF:ai7max=ai7,MAXIMUM', 'VDEF:ai7min=ai7,MINIMUM',

                  'LINE2:ai0#0000ff:ai0',
                  'LINE2:ai1#00ff00:ai1',
                  'LINE2:ai2#ff0000:ai2',
                  'LINE2:ai3#00aaaa:ai3',
                  'LINE2:ai4#aa00aa:ai4',
                  'LINE2:ai5#aaaa00:ai5',
                  'LINE2:ai6#555555:ai6',
                  'LINE2:ai7#aaaaaa:ai7',
                  )

if __name__ == "__main__":
    #log_to_rrd()
    plot_rrd()
