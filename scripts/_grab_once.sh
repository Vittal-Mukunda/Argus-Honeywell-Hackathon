#!/usr/bin/env bash
# Launch sim headless, grab one cam0 frame to PPM, teardown.
set +e
source /opt/ros/humble/setup.bash 2>/dev/null
source ~/argus/install/setup.bash 2>/dev/null
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42
bash ~/argus/scripts/_killsim.sh >/dev/null 2>&1
sleep 2
ros2 launch argus_bringup argus_sim.launch.py headless:=true > /tmp/grab_launch.log 2>&1 &
LP=$!
sleep 20
python3 ~/argus/scripts/_cam_grab.py /tmp/cam0_live.ppm
kill -INT ${LP} 2>/dev/null
sleep 4
bash ~/argus/scripts/_killsim.sh >/dev/null 2>&1
echo "grab done"
