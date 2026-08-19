"""Shared fixtures for the fast test tier.

These tests never start a simulator. They read the package's own sources --
the xacro description, the YAML configs, the launch files and the two markdown
documents -- and assert that the claims those files make about each other are
actually true.

The integration/ directory holds the tests that do launch Gazebo; pytest must
not try to import them, so they are excluded below. They are registered
separately as launch tests and only built with -DBUILD_SIM_TESTS=ON.
"""

from pathlib import Path
import subprocess

import pytest

import yaml

# Do not let pytest collect the launch_testing files.
collect_ignore_glob = ['integration/*']

PKG = Path(__file__).resolve().parents[1]
PREFIX = 'r1_'


def render(prefix=PREFIX, omni='true', mm='true', **extra):
    """Run xacro over system.urdf.xacro and return the URDF text.

    Fails the test with xacro's own stderr rather than a traceback, because
    that message is the useful one.
    """
    args = {'prefix': prefix, 'omni': omni, 'mm': mm, **extra}
    cmd = ['xacro', str(PKG / 'urdf' / 'system.urdf.xacro')]
    cmd += [f'{k}:={v}' for k, v in args.items()]
    proc = subprocess.run(cmd, capture_output=True, text=True)
    if proc.returncode != 0:
        pytest.fail(f'xacro failed for {args}:\n{proc.stderr}')
    return proc.stdout


def load_config(name):
    """Load a file from config/ as YAML."""
    with open(PKG / 'config' / name) as f:
        return yaml.safe_load(f)


def read(*parts):
    """Read a file from the package source tree as text."""
    return (PKG.joinpath(*parts)).read_text()


def launch_defaults(launch_file):
    """Return {argument_name: default_value} for a launch file.

    Reads the declared arguments out of the LaunchDescription rather than
    grepping, so the test sees exactly what `ros2 launch` would.
    """
    from launch import LaunchDescription  # noqa: F401  (import cost is real)
    from launch.actions import DeclareLaunchArgument
    from launch.launch_description_sources import get_launch_description_from_python_launch_file

    ld = get_launch_description_from_python_launch_file(
        str(PKG / 'launch' / launch_file))
    out = {}
    for entity in ld.entities:
        if isinstance(entity, DeclareLaunchArgument):
            default = entity.default_value
            if default is not None:
                default = ''.join(getattr(p, 'text', '') for p in default)
            out[entity.name] = default
    return out


@pytest.fixture(scope='session')
def urdf_mm():
    """Render the full mobile manipulator once for the whole session."""
    return render(mm='true')


@pytest.fixture(scope='session')
def urdf_base():
    """Render the bare mobile base."""
    return render(mm='false')
