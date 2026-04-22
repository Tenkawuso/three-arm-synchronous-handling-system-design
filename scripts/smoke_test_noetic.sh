#!/usr/bin/env bash
set -euo pipefail

source /workspace/project/catkin_ws/devel/setup.bash

mkdir -p /workspace/project/.tmp

pkill -f "three_arm_coordinator.py" || true
pkill -f "my_three_arms_moveit_config demo.launch" || true
pkill -f "move_group" || true
sleep 1

roslaunch my_three_arms_moveit_config demo.launch use_rviz:=false > /workspace/project/.tmp/demo.log 2>&1 &
demo_pid=$!

sleep 10

rosrun three_arm_demo three_arm_coordinator.py > /workspace/project/.tmp/coord.log 2>&1 &
coord_pid=$!

sleep 3

python3 - <<'PY'
import rospy
from geometry_msgs.msg import PointStamped

rospy.init_node('sim_targets_stream', anonymous=True)
targets = {
    '/target/P': (0.22, 0.03, 0.01),
    '/target/Y': (-0.07, 0.20, 0.01),
    '/target/G': (-0.06, -0.20, 0.01),
}
publishers = {topic: rospy.Publisher(topic, PointStamped, queue_size=10) for topic in targets}

rate = rospy.Rate(5)
for _ in range(50):
    now = rospy.Time.now()
    for topic, (x, y, z) in targets.items():
        msg = PointStamped()
        msg.header.frame_id = 'world'
        msg.header.stamp = now
        msg.point.x = x
        msg.point.y = y
        msg.point.z = z
        publishers[topic].publish(msg)
    rate.sleep()

print('published_target_stream_done')
PY

timeout_s=360
for ((i=0; i<timeout_s; i++)); do
  if ! kill -0 "$coord_pid" 2>/dev/null; then
    break
  fi
  sleep 1
done

if kill -0 "$coord_pid" 2>/dev/null; then
  echo "coordinator_timeout" > /workspace/project/.tmp/smoke_result.txt
  kill "$coord_pid" || true
else
  echo "coordinator_exited" > /workspace/project/.tmp/smoke_result.txt
fi

python3 - <<'PY'
from pathlib import Path
import sys

log_path = Path('/workspace/project/.tmp/coord.log')
if not log_path.exists():
    print('smoke_fail: coord log missing')
    sys.exit(1)

text = log_path.read_text(errors='ignore')
required_tokens = [
    '[arm1] 任务完成',
    '[arm2] 任务完成',
    '[arm3] 任务完成',
    '所有机械臂任务执行结束',
]

missing = [token for token in required_tokens if token not in text]
if missing:
    print('smoke_fail: missing tokens ->')
    for token in missing:
        print(token)
    tail = text.splitlines()[-60:]
    print('--- coord log tail ---')
    for line in tail:
        print(line)
    sys.exit(2)

print('smoke_pass: all completion tokens found')
sys.exit(0)
PY

kill "$demo_pid" 2>/dev/null || true
pkill -f "my_three_arms_moveit_config demo.launch" || true
pkill -f "move_group" || true
