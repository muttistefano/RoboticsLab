"""The documentation is true.

README.md and DEMO.md are executable promises: a reader types what is written
there and expects it to work. These tests read both documents, extract every
command and every argument table, and check them against the package.

This catches the failure mode that is otherwise only discovered live, in front
of an audience -- a documented argument that was never plumbed through, a
launch file that was renamed, a flag that no longer exists.
"""

import re

from conftest import launch_defaults, PKG

import pytest

DOCS = ['README.md', 'DEMO.md']
REPO = PKG.parents[2]          # ros_ws/src/lampo_description -> repo root
PACKAGE = 'lampo_description'

LAUNCH_CMD = re.compile(rf'ros2 launch {PACKAGE} (\S+\.launch\.py)((?: +\S+:=\S+)*)')
RUN_CMD = re.compile(rf'ros2 run {PACKAGE} (\S+)')


def document(name):
    """Read one of the markdown documents."""
    return (REPO / name).read_text()


def code_blocks(text):
    """Yield the contents of every fenced code block."""
    return re.findall(r'```(?:bash)?\n(.*?)```', text, re.DOTALL)


def commands(name):
    """Yield every shell command in every fenced block, continuations joined."""
    for block in code_blocks(document(name)):
        joined = block.replace('\\\n', ' ')
        for line in joined.splitlines():
            line = line.strip()
            if line and not line.startswith('#'):
                yield line


@pytest.mark.parametrize('doc', DOCS)
def test_launch_files_referenced_actually_exist(doc):
    """Every `ros2 launch lampo_description X` names a real, installed file."""
    referenced = {m.group(1) for m in LAUNCH_CMD.finditer(document(doc))}
    assert referenced, f'{doc} documents no launch commands'
    for launch_file in sorted(referenced):
        assert (PKG / 'launch' / launch_file).is_file(), \
            f'{doc} references {launch_file}, which does not exist'


@pytest.mark.parametrize('doc', DOCS)
def test_launch_arguments_used_are_declared(doc):
    """Every `arg:=value` in the docs is an argument that launch file declares.

    A typo, or an argument documented before it was implemented, fails here
    instead of silently doing nothing at runtime -- `ros2 launch` accepts
    unknown arguments without complaint.
    """
    problems = []
    for match in LAUNCH_CMD.finditer(document(doc)):
        launch_file, arg_text = match.group(1), match.group(2)
        if not arg_text.strip():
            continue
        declared = launch_defaults(launch_file)
        for pair in arg_text.split():
            name = pair.split(':=')[0]
            if name not in declared:
                problems.append(f'{doc}: {launch_file} has no argument "{name}"')
    assert not problems, '\n'.join(problems)


@pytest.mark.parametrize('doc', DOCS)
def test_executables_referenced_are_installed(doc):
    """Every `ros2 run lampo_description X` names something CMake installs."""
    cmake = (PKG / 'CMakeLists.txt').read_text()
    installed = re.search(r'install\(PROGRAMS(.*?)DESTINATION', cmake, re.DOTALL)
    assert installed, 'CMakeLists.txt installs no programs'
    names = {line.strip().split('/')[-1] for line in installed.group(1).splitlines()
             if line.strip()}

    for match in RUN_CMD.finditer(document(doc)):
        assert match.group(1) in names, \
            f'{doc} runs {match.group(1)}, which is not installed'


def test_readme_argument_tables_match_the_launch_files():
    """Documented defaults are the real defaults.

    Each argument table follows the code block it describes. A block may name
    more than one launch file (the joystick section shows the Nav2 retarget
    alongside it), so the table is attributed to whichever candidate actually
    declares every argument in it. If none does, that is the failure.
    """
    text = document('README.md')
    candidates = []
    problems = []

    for line in text.splitlines():
        found = LAUNCH_CMD.findall(line)
        if found:
            candidates = [f[0] for f in found] + candidates
            continue

        row = re.match(r'\|\s*`([^`]+)`\s*\|\s*`([^`]+)`\s*\|', line)
        if not row or not candidates:
            continue

        names = [n.strip().strip('`') for n in row.group(1).split(',')]
        values = [v.strip().strip('`') for v in row.group(2).split(',')]

        owner = next((c for c in candidates
                      if all(n in launch_defaults(c) for n in names)), None)
        if owner is None:
            problems.append(f'no launch file among {candidates} declares '
                            f'all of {names}')
            continue

        declared = launch_defaults(owner)
        if len(values) != len(names):
            continue
        for name, documented in zip(names, values):
            actual = declared[name]
            if actual is None:
                continue
            # Paths are documented by their tail, not their absolute location.
            if '/' in documented or '/' in actual:
                if documented.split('/')[-1] not in actual:
                    problems.append(f'{owner}: {name} documented as '
                                    f'"{documented}", actually "{actual}"')
            elif documented.lower() != actual.lower():
                problems.append(f'{owner}: {name} documented as '
                                f'"{documented}", actually "{actual}"')

    assert not problems, '\n'.join(problems)


def test_every_launch_file_is_documented():
    """No launch file is undiscoverable.

    A launch file the README never mentions may as well not exist -- which is
    how lampo_joy.launch.py and nodo_prova.py went unnoticed.
    """
    text = document('README.md')
    for path in sorted((PKG / 'launch').glob('*.launch.py')):
        assert path.name in text, f'{path.name} is not mentioned in README.md'


def test_demo_recovery_commands_are_real():
    """The recovery cheatsheet does not tell the reader to run a fiction."""
    text = document('DEMO.md')
    for match in re.finditer(r'ros2 param get (\S+) ([\w_]+)', text):
        assert match.group(2) == 'use_sim_time', \
            f'unexpected parameter in recovery table: {match.group(2)}'


def test_service_calls_in_docs_name_real_types():
    """`ros2 service call` examples use a type that is actually installed."""
    import importlib

    for doc in DOCS:
        for match in re.finditer(r'ros2 service call \S+ (\S+)/srv/(\S+)', document(doc)):
            pkg, srv = match.group(1), match.group(2)
            module = importlib.import_module(f'{pkg}.srv')
            assert hasattr(module, srv), f'{doc}: {pkg}/srv/{srv} does not exist'


def test_topic_pub_examples_name_real_types():
    """`ros2 topic pub` examples use a message type that is installed."""
    import importlib

    for doc in DOCS:
        for match in re.finditer(r'(\w+)/msg/(\w+)', document(doc)):
            pkg, msg = match.group(1), match.group(2)
            module = importlib.import_module(f'{pkg}.msg')
            assert hasattr(module, msg), f'{doc}: {pkg}/msg/{msg} does not exist'
