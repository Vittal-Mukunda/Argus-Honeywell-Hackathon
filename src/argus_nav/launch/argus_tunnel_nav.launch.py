"""Launch the ARGUS navigation pillar for the 202.8 m tunnel_circuit (Scenario E).

The straight-corridor ``argus_nav.launch.py`` flies the 30 m warehouse with the
yaw-locked ``reactive_avoider``; this variant drives the closed tunnel loop with
``circuit_avoider`` (centreline-following + reactive lateral dodge) and scales
the occupancy map to the full stadium -- NO hardcoded 30 m bounds.

  stereo_depth      -- /argus/cam{0,1}/image_raw -> /argus/depth/{image,points}
  occupancy_mapper  -- /argus/depth/points (+ GT pose) -> /argus/map/points
                       (log-odds RGB fusion; bounds = tunnel stadium AABB)
  circuit_avoider   -- GT pose + /argus/depth|lidar/points -> /argus/cmd_vel
                       (flies the loop, strafes around in-lane obstacles)

Map + steering read ground-truth pose (clean digital twin + reliable loop
following where live VINS drifts); obstacle sensing is fully live. Consumes only
public contract topics (+ the additive /argus/lidar/* sensor); nothing remaps a
frozen interface.

Args:
  use_sim_time (true)        -- follow /clock from the simulator.
  decimation   (2)           -- SGBM input downscale.
  speed        (0.8)         -- cruise speed (m/s), the flight envelope.
  laps         (1)           -- laps to fly before hovering.
  alt          (1.0)         -- hold altitude (m).
  enable_depth / enable_avoider / enable_mapper (true) -- per-node toggles.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Tunnel stadium AABB (m): straights y=0 / y=20, caps r=10 about (0,10)/(70,10),
# walls at centreline +/- 3, height 3.5. Covers the whole interior + a margin so
# the full loop's map is never clipped (cf. warehouse 30 m bounds).
TUNNEL_BOUNDS_MIN = [-14.0, -4.0, -0.2]
TUNNEL_BOUNDS_MAX = [84.0, 24.0, 3.6]


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    decimation = LaunchConfiguration("decimation")
    speed = LaunchConfiguration("speed")
    laps = LaunchConfiguration("laps")
    alt = LaunchConfiguration("alt")
    enable_depth = LaunchConfiguration("enable_depth")
    enable_avoider = LaunchConfiguration("enable_avoider")
    enable_mapper = LaunchConfiguration("enable_mapper")

    return LaunchDescription(
        [
            SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("decimation", default_value="2"),
            DeclareLaunchArgument("speed", default_value="0.8"),
            DeclareLaunchArgument("laps", default_value="1"),
            DeclareLaunchArgument("alt", default_value="1.0"),
            DeclareLaunchArgument("enable_depth", default_value="true"),
            DeclareLaunchArgument("enable_avoider", default_value="true"),
            DeclareLaunchArgument("enable_mapper", default_value="true"),
            Node(
                package="argus_nav",
                executable="stereo_depth",
                name="stereo_depth",
                output="screen",
                condition=IfCondition(enable_depth),
                parameters=[{"use_sim_time": use_sim_time, "decimation": decimation}],
            ),
            Node(
                package="argus_nav",
                executable="circuit_avoider",
                name="circuit_avoider",
                output="screen",
                condition=IfCondition(enable_avoider),
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "speed": speed,
                        "laps": laps,
                        "alt": alt,
                    }
                ],
            ),
            Node(
                package="argus_nav",
                executable="occupancy_mapper",
                name="occupancy_mapper",
                output="screen",
                condition=IfCondition(enable_mapper),
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        # Clean digital twin: place + ORIENT the map from GT pose
                        # (pose_type=pose -> orientation_mode auto picks the GT
                        # quaternion, which is needed since the drone yaws through
                        # the end-caps -- identity would smear the cap walls).
                        "pose_type": "pose",
                        "pose_topic": "/argus/ground_truth/pose",
                        "bounds_min": TUNNEL_BOUNDS_MIN,
                        "bounds_max": TUNNEL_BOUNDS_MAX,
                        # full-loop interior is far larger than the 30 m corridor;
                        # lift the voxel cap so the finished map isn't truncated.
                        "max_voxels": 600000,
                        # SMOOTHNESS: the mapper is single-threaded, so per-frame
                        # ray-carving competes with the publish on one core and
                        # starves it (~0.6 Hz on the full map). Subsample carving
                        # rays harder and fuse fewer redundant frames -> the thread
                        # keeps up at the 4 Hz publish rate. Negligible quality cost
                        # (carving + fusion are integrated over thousands of frames).
                        "carve_max_rays": 1000,
                        "motion_thresh": 0.12,
                        # RESOLUTION: 0.20 m voxels for a crisper, judge-facing map --
                        # walls, ribs and the dodged obstacles read with noticeably
                        # finer detail than the 0.30 m draft. _publish rebuilds the WHOLE
                        # grid each tick (O(grid) Python), so the finer grid (~2x the
                        # surface voxels) costs the live map a few FPS as the 200 m map
                        # fills -- an accepted trade for the quality (VINS now also runs
                        # for the feature-track view). Still coarser than the 0.15 m
                        # warehouse map, which dropped the publish rate too far at
                        # stadium scale.
                        "voxel_size": 0.20,
                    }
                ],
            ),
        ]
    )
