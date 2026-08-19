"""The robot description renders, parses, and is prefixed consistently.

Every assertion here corresponds to a bug that was actually found in this
package. They are cheap to run and they fail loudly, which is the point: a
broken URDF otherwise shows up as a robot that looks fine in Gazebo and has no
transforms in RViz.
"""

import xml.etree.ElementTree as ET

from conftest import PREFIX, render

import pytest

CONFIGURATIONS = [
    pytest.param('true', 'true', id='omni-manipulator'),
    pytest.param('true', 'false', id='omni-base'),
    pytest.param('false', 'true', id='diff-manipulator'),
    pytest.param('false', 'false', id='diff-base'),
]

WHEELS = ['front_left', 'front_right', 'rear_left', 'rear_right']


@pytest.mark.parametrize('omni,mm', CONFIGURATIONS)
def test_renders_and_parses(omni, mm):
    """All four configurations produce well-formed, parseable URDF."""
    root = ET.fromstring(render(omni=omni, mm=mm))
    assert root.tag == 'robot'
    assert root.findall('link'), 'no links in the rendered description'


@pytest.mark.parametrize('omni,mm', CONFIGURATIONS)
def test_single_root_link(omni, mm):
    """Exactly one link is not a child of any joint: the tree has one root.

    Two roots means a disconnected robot -- the classic symptom of a joint
    referring to a link name that a prefix change left behind.
    """
    root = ET.fromstring(render(omni=omni, mm=mm))
    links = {link.get('name') for link in root.findall('link')}
    children = {j.find('child').get('link') for j in root.findall('joint')}
    roots = links - children
    assert roots == {f'{PREFIX}base_footprint'}, f'expected one root, got {roots}'


@pytest.mark.parametrize('omni,mm', CONFIGURATIONS)
def test_everything_is_prefixed(omni, mm):
    """No link or joint escapes the prefix.

    The README tells users to pass `namespace:=r2_` with a trailing separator.
    An unprefixed link would silently collide with the other robot's frame of
    the same name, which is very hard to see and very easy to introduce.
    """
    root = ET.fromstring(render(omni=omni, mm=mm))
    names = ([link.get('name') for link in root.findall('link')]
             + [joint.get('name') for joint in root.findall('joint')])
    unprefixed = [n for n in names if not n.startswith(PREFIX)]
    assert not unprefixed, f'not prefixed: {unprefixed}'


@pytest.mark.parametrize('omni,mm', CONFIGURATIONS)
def test_wheels_are_continuous_with_dynamics(omni, mm):
    """Wheel joints spin freely and have damping.

    They were `revolute` with +/-1.8e308 limits, which is a continuous joint
    spelled the hard way, and carried no <dynamics> at all.
    """
    root = ET.fromstring(render(omni=omni, mm=mm))
    joints = {j.get('name'): j for j in root.findall('joint')}
    for wheel in WHEELS:
        joint = joints[f'{PREFIX}{wheel}_wheel_joint']
        assert joint.get('type') == 'continuous', f'{wheel}: {joint.get("type")}'
        assert joint.find('dynamics') is not None, f'{wheel}: no <dynamics>'
        assert joint.find('limit') is not None, f'{wheel}: no <limit>'


@pytest.mark.parametrize('omni,mm', CONFIGURATIONS)
def test_inertias_are_physically_possible(omni, mm):
    """Every inertia is positive and satisfies the triangle inequality.

    A rigid body cannot have Izz > Ixx + Iyy. Several links in this package
    did, which makes the solver behave in ways that look like a physics bug.
    """
    root = ET.fromstring(render(omni=omni, mm=mm))
    for link in root.findall('link'):
        inertial = link.find('inertial')
        if inertial is None:
            continue
        i = inertial.find('inertia')
        ixx, iyy, izz = (float(i.get(k)) for k in ('ixx', 'iyy', 'izz'))
        name = link.get('name')
        assert float(inertial.find('mass').get('value')) > 0, f'{name}: zero mass'
        assert min(ixx, iyy, izz) > 0, f'{name}: non-positive inertia'
        # Allow a little slack for rounded values.
        tol = 1e-9
        assert ixx <= iyy + izz + tol, f'{name}: ixx > iyy + izz'
        assert iyy <= ixx + izz + tol, f'{name}: iyy > ixx + izz'
        assert izz <= ixx + iyy + tol, f'{name}: izz > ixx + iyy'


def test_camera_has_an_optical_frame(urdf_mm):
    """The RGBD camera carries a REP-103 optical frame.

    Without it the point cloud arrives rotated 90 degrees and every downstream
    consumer is quietly wrong.
    """
    root = ET.fromstring(urdf_mm)
    links = {link.get('name') for link in root.findall('link')}
    assert f'{PREFIX}cam_optical_frame' in links

    joint = next(j for j in root.findall('joint')
                 if j.find('child').get('link') == f'{PREFIX}cam_optical_frame')
    rpy = [float(v) for v in joint.find('origin').get('rpy').split()]
    assert any(abs(v) > 1e-6 for v in rpy), 'optical frame is not rotated'


def test_base_only_configuration_has_no_arm(urdf_base):
    """mm:=false really does leave the arm, gripper and camera out."""
    root = ET.fromstring(urdf_base)
    names = ' '.join(link.get('name') for link in root.findall('link'))
    for absent in ('wrist_3_link', 'robotiq', 'cam_link'):
        assert absent not in names, f'{absent} present with mm:=false'


def test_gripper_control_is_opt_in():
    """The gripper's ros2_control block appears only when asked for.

    It is off by default because the mimic-joint approximation of the 4-bar
    finger linkage drives the knuckle outside its limits, and ros2_control's
    joint limiter responds by aborting the simulator.
    """
    assert 'robotiq_control' not in render(gripper_control='false')
    assert 'robotiq_control' in render(gripper_control='true')


def test_sensors_declare_their_frames(urdf_mm):
    """Lidar, IMU and camera all set gz_frame_id.

    A sensor without it publishes into a frame nothing can resolve.
    """
    assert urdf_mm.count('<gz_frame_id>') >= 3, 'a sensor is missing gz_frame_id'
    for frame in (f'{PREFIX}front_laser',
                  f'{PREFIX}base_link_sweepee',
                  f'{PREFIX}cam_optical_frame'):
        assert f'<gz_frame_id>{frame}</gz_frame_id>' in urdf_mm, frame
