#!/usr/bin/env bash
# Determinism test: replay baseline_ABC twice (multiple_thread:0, rate 0.2).
# Expect both runs identical drift AND <1.5% -> deterministic + reliable init.
set +u
source /opt/ros/humble/setup.bash 2>/dev/null
source ~/argus/install/setup.bash 2>/dev/null
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42
cd ~/argus
echo "config multiple_thread: $(grep -E '^multiple_thread:' src/argus_vio/config/argus_stereo_imu_config.yaml)"
for i in 1 2; do
  echo "===== RUN ${i} (mt0, rate0.2) ====="
  RATE=0.2 bash scripts/run_vio_offline.sh ~/argus/data/bags/baseline_ABC ~/argus/data/bags/vio_eval_mt0_r${i} 2>&1 | tail -1
  ~/.venvs/argus-eval/bin/python scripts/run_eval.py --bag ~/argus/data/bags/vio_eval_mt0_r${i} --run-id mt0_r${i} \
    --vio-topic /argus/vio/odom_optimized --skip-start-m 2.0 --max-dist-m 24.0 2>&1 \
    | grep -iE "path_length|ate_rmse|drift_pct_ate|drift_pct_final|kitti_drift_pct_mean"
done
echo "===== DETERM TEST DONE ====="
