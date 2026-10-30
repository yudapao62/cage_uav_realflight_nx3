#!/usr/bin/env bash
set -Eeuo pipefail
duration=${1:-20}
source /opt/ros/noetic/setup.bash
source /home/nx3/gps_ws/devel/setup.bash
python3 - "$duration" <<'PY'
import sys, time, threading
import rospy
from gps_receiver.msg import AnchorFix

duration=float(sys.argv[1]); lock=threading.Lock(); data={r:{'times':[],'msg':None,'last_seq':None,'last_rx':None} for r in ('A','B')}
def callback(msg, role):
    now=time.monotonic()
    with lock:
        item=data[role]; item['msg']=msg; item['last_rx']=now
        if item['last_seq'] is None or msg.seq != item['last_seq']:
            item['times'].append(now); item['last_seq']=msg.seq
rospy.init_node('check_ground_gps_rate', anonymous=True, disable_signals=True)
rospy.Subscriber('/farm/scene_1/anchor_A/fix',AnchorFix,callback,callback_args='A',queue_size=50)
rospy.Subscriber('/farm/scene_1/anchor_B/fix',AnchorFix,callback,callback_args='B',queue_size=50)
end=time.monotonic()+duration
while time.monotonic()<end and not rospy.is_shutdown(): time.sleep(0.1)
now=time.monotonic()
for role in ('A','B'):
    item=data[role]; msg=item['msg']; times=item['times']
    print('Anchor {}:'.format(role))
    if msg is None:
        print('  rate: NO SAMPLE\n  last_age: NO SAMPLE\n  seq: NO SAMPLE\n  valid: NO SAMPLE\n  satellites: NO SAMPLE')
        continue
    rate=(len(times)-1)/(times[-1]-times[0]) if len(times)>=2 and times[-1]>times[0] else 0.0
    print('  rate: {:.3f} Hz ({} sequence-advancing samples)'.format(rate,len(times)))
    print('  last_age: {:.3f} s'.format(now-item['last_rx']))
    print('  seq: {}'.format(msg.seq)); print('  stamp: {}'.format(msg.stamp))
    print('  valid: {}'.format(msg.valid)); print('  satellites: {}'.format(msg.satellites))
PY
