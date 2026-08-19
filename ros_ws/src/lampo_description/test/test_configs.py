"""The YAML configs agree with the robot and with the launch files.

Each test here is a bug that cost real debugging time. They exist because the
failures they catch are silent: nothing errors, the robot simply behaves
wrongly, and you spend an afternoon looking at the wrong subsystem.
"""

import ast
import re

from conftest import launch_defaults, load_config, PKG, read

import pytest

NAV2 = 'nav2_params_omni.yaml'


@pytest.fixture(scope='module')
def nav2():
    """Load the Nav2 parameters, with the /** wildcard level stripped off."""
    return load_config(NAV2)['/**']


def test_use_sim_time_is_set_for_every_node(nav2):
    """The wildcard sets use_sim_time once, for all ~15 Nav2 servers.

    It used to be set on amcl alone. Every other server therefore ran on the
    wall clock while TF was stamped in sim time, which surfaces as an endless
    "Lookup would require extrapolation into the future" and looks like a TF
    bug rather than a configuration one.

    RewrittenYaml's param_rewrites can only replace keys that already exist,
    so setting it here is what makes the launch file's rewrite effective.
    """
    assert nav2['ros__parameters']['use_sim_time'] is True


def test_amcl_seed_matches_the_spawn_pose(nav2):
    """AMCL starts where the robot is actually spawned.

    These were two independent hardcoded pairs 2.3 m apart: the robot spawned
    at (-3.5, 2.2) and AMCL was told (-4.42, 0.12). Localisation began
    diverged, and the first goal sent the robot into a wall.

    This is the single most valuable assertion in the suite -- it ties the
    config to the launch file so they cannot drift apart again.
    """
    spawn = launch_defaults('lampo_gz_mm.launch.py')
    seed = nav2['amcl']['ros__parameters']['initial_pose']

    assert float(spawn['x']) == pytest.approx(seed['x'])
    assert float(spawn['y']) == pytest.approx(seed['y'])
    assert float(spawn['yaw']) == pytest.approx(seed['yaw'])


def test_nav_launch_seeds_from_the_same_pose():
    """lampo_nav_omni's x/y/yaw defaults match lampo_gz_mm's.

    The nav launch file feeds these into AMCL through param_rewrites, so a
    user who overrides the spawn pose gets a consistent stack by passing the
    same values to both.
    """
    spawn = launch_defaults('lampo_gz_mm.launch.py')
    nav = launch_defaults('lampo_nav_omni.launch.py')
    for key in ('x', 'y', 'yaw'):
        assert float(spawn[key]) == pytest.approx(float(nav[key])), key


def test_footprint_matches_the_real_chassis(nav2):
    """The costmap footprint is the actual base rectangle.

    It was robot_radius: 0.5 for a 1.02 x 0.55 m base, whose circumscribed
    radius is 0.579 -- an 8 cm under-approximation, all of it at the corners,
    which is exactly where a rectangular robot clips a doorway.
    """
    xacro = read('urdf', 'sweepee', 'sweepee_omni.xacro')
    size = {name: float(re.search(
        rf'name="{name}" value="([\d.]+)"', xacro).group(1))
        for name in ('base_x_size', 'base_y_size')}

    for costmap in ('local_costmap', 'global_costmap'):
        params = nav2[costmap][costmap]['ros__parameters']
        assert 'robot_radius' not in params, f'{costmap} still uses a circle'
        corners = ast.literal_eval(params['footprint'])
        assert max(abs(x) for x, _ in corners) == pytest.approx(size['base_x_size'] / 2)
        assert max(abs(y) for _, y in corners) == pytest.approx(size['base_y_size'] / 2)


def test_amcl_does_not_trust_beams_past_the_sensor_range(nav2):
    """laser_max_range does not exceed what the lidar can actually see.

    It was 100.0 against a 30 m sensor. Every no-return came back as a valid
    100 m reading, so AMCL scored particles against measurements the hardware
    never made.
    """
    xacro = read('urdf', 'sweepee', 'sweepee_omni.xacro')
    sensor_max = float(re.search(r'<max>([\d.]+)</max>', xacro).group(1))
    assert nav2['amcl']['ros__parameters']['laser_max_range'] <= sensor_max


def test_collision_monitor_output_matches_the_launch_default(nav2):
    """The velocity chain ends where the launch file says it does.

    collision_monitor published to `cmd_vel`, which velocity_smoother was also
    subscribed to -- a feedback loop. It now ends at cmd_vel_safe, which is
    what the bridge forwards to Gazebo.
    """
    configured = nav2['collision_monitor']['ros__parameters']['cmd_vel_out_topic']
    assert configured == 'cmd_vel_safe'
    assert launch_defaults('lampo_nav_omni.launch.py')['cmd_vel_topic'] == configured


def test_every_config_uses_a_wildcard_node_key():
    """Parameter files key on /**, not on a bare node name.

    A key like `twist_mux:` matches only a node whose fully qualified name is
    `/twist_mux`. Every node in this package runs inside a namespace, so such a
    key never matches and the whole file is silently ignored -- the node comes
    up on its defaults, nothing errors, and it simply does not behave as
    configured. twist_mux, slam_toolbox and the EKF all shipped this way.
    """
    for name in ('twist_mux.yaml', 'slam_toolbox.yaml', 'ekf.yaml',
                 'nav2_params_omni.yaml'):
        keys = list(load_config(name))
        assert keys == ['/**'], f'{name} keys on {keys}, not the /** wildcard'


def test_sandbox_launches_no_robot():
    """lampo_sandbox brings up the world only.

    The three-terminal workflow depends on this split: the world outlives any
    individual robot, so a student can restart a robot without restarting the
    simulation. Checked by reading the launch file rather than at runtime,
    where a leftover process from a previous run makes the answer unreliable.
    """
    sandbox = read('launch', 'lampo_sandbox.launch.py')
    for robot_only in ('robot_state_publisher', 'spawn', 'controller_manager'):
        assert robot_only not in sandbox, \
            f'lampo_sandbox.launch.py starts {robot_only}; robots belong in lampo_gz_mm'


def test_mppi_model_dt_matches_the_controller_period(nav2):
    """The MPPI model_dt equals 1 / controller_frequency.

    If the controller period is longer than model_dt, MPPI throws during
    configuration -- "Controller period more then model dt" -- and the entire
    navigation group fails to activate. Changing controller_frequency without
    changing model_dt is an easy and total break.
    """
    controller = nav2['controller_server']['ros__parameters']
    period = 1.0 / controller['controller_frequency']
    assert controller['FollowPath']['model_dt'] == pytest.approx(period), \
        'model_dt must equal 1 / controller_frequency'


def test_joystick_outranks_navigation():
    """twist_mux gives a human with a joystick authority over autonomy."""
    topics = load_config('twist_mux.yaml')['/**']['ros__parameters']['topics']
    assert topics['joystick']['priority'] > topics['navigation']['priority']
    assert topics['joystick']['topic'] == 'cmd_vel_joy'
    assert topics['navigation']['topic'] == 'cmd_vel_nav'


def test_twist_mux_speaks_the_same_message_type_as_the_bridge():
    """twist_mux must be stamped, because the bridge expects TwistStamped."""
    params = load_config('twist_mux.yaml')['/**']['ros__parameters']
    assert params['use_stamped'] is True


def test_ekf_frames_use_the_prefix_sentinel():
    """ekf.yaml is namespace-agnostic, like every other config here.

    A literal r1_ in this file would quietly break the second robot.
    """
    ekf = load_config('ekf.yaml')['/**']['ros__parameters']
    for key in ('odom_frame', 'base_link_frame', 'world_frame'):
        assert ekf[key].startswith('PREFIX_'), f'{key} = {ekf[key]}'


def test_slam_config_uses_the_prefix_sentinel():
    """Same for slam_toolbox.

    Checks the parsed values, not the raw text: the file's header comment
    legitimately shows a concrete `/r1_/slam_toolbox/save_map` example.
    """
    slam = load_config('slam_toolbox.yaml')['/**']['ros__parameters']
    assert slam['odom_frame'].startswith('PREFIX_')
    assert slam['base_frame'].startswith('PREFIX_')
    assert slam['map_frame'] == 'map', 'the map frame is global, not per-robot'


def test_bridge_entries_are_complete_and_namespaceable():
    """Every bridge entry is well-formed and gets namespaced at launch.

    The launch file rewrites '/prefix/' to '/<namespace>/'. An entry that
    forgets the sentinel is silently shared between robots -- two robots would
    drive each other.
    """
    # gz_topic_name is optional -- ros_gz_bridge defaults it to the ROS name,
    # which is correct for every sensor here.
    required = {'ros_topic_name', 'ros_type_name', 'gz_type_name', 'direction'}
    entries = load_config('bridge.yaml')
    assert entries, 'bridge.yaml is empty'

    for entry in entries:
        missing = required - set(entry)
        assert not missing, f'{entry.get("ros_topic_name")} missing {missing}'
        assert entry['direction'] in ('ROS_TO_GZ', 'GZ_TO_ROS', 'BIDIRECTIONAL')
        assert entry['ros_topic_name'].startswith('/prefix/'), entry['ros_topic_name']


def test_cmd_vel_is_one_way():
    """The command topic is not bridged BIDIRECTIONAL.

    It was, with no distinct gz_topic_name, so the bridge echoed every command
    back to itself.
    """
    entry = next(e for e in load_config('bridge.yaml')
                 if e['ros_topic_name'].endswith('cmd_vel_safe'))
    assert entry['direction'] == 'ROS_TO_GZ'
    assert entry['ros_type_name'] == 'geometry_msgs/msg/TwistStamped'


def test_clock_is_bridged_once_for_the_whole_simulation():
    """/clock belongs to the world, not to each robot.

    It used to be in the per-robot bridge, so N robots meant N publishers on
    /clock, all racing.
    """
    assert not any('clock' in e['ros_topic_name'] for e in load_config('bridge.yaml')), \
        '/clock is in the per-robot bridge; it belongs in lampo_sandbox.launch.py'
    assert 'clock' in read('launch', 'lampo_sandbox.launch.py')


def test_static_transforms_are_published_on_the_namespaced_topic():
    """robot_state_publisher namespaces /tf_static, and it is relayed globally.

    Nav2's servers are composable nodes launched by nav2_bringup, which remaps
    /tf_static to <ns>/tf_static. If robot_state_publisher publishes statics
    only on the global topic, that namespaced topic has no publisher: AMCL
    never learns base_footprint -> front_laser, drops every scan, and never
    emits map -> odom. Nav2 still reports "active", so the stack looks healthy
    and the robot simply never localises.

    The global copy still exists, via a relay, so RViz and tf2_tools see one
    merged tree. topic_tools/relay matches the source QoS, so the relayed
    topic keeps TRANSIENT_LOCAL and late joiners are still served.
    """
    launch = read('launch', 'lampo_gz_mm.launch.py')
    assert "('/tf_static', 'tf_static')" in launch, \
        'robot_state_publisher must publish /tf_static under the namespace'
    assert "'tf_static', '/tf_static'" in launch, \
        'the namespaced /tf_static must be relayed onto the global topic'


def test_controllers_config_is_namespace_agnostic():
    """ur_controllers.yaml uses the sentinel in keys as well as values.

    The substitution used to rewrite values only, so the joint_trajectory
    controller's per-joint `constraints:` keys kept an unprefixed joint name
    and every tolerance in that block was silently ignored.
    """
    text = read('config', 'ur_controllers.yaml')
    assert 'PREFIX_' in text
    assert 'r1_' not in text


def test_no_package_uris_are_unresolvable():
    """Every package:// URI in the world names a package that exists."""
    from ament_index_python.packages import PackageNotFoundError, get_package_share_directory

    world = (PKG / 'worlds' / 'warehouse.sdf').read_text()
    for pkg in sorted(set(re.findall(r'package://([^/]+)/', world))):
        try:
            get_package_share_directory(pkg)
        except PackageNotFoundError:
            pytest.fail(f'world references package://{pkg}, which is not installed')
