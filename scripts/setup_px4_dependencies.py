#!/usr/bin/env python3
"""Install pinned bridge dependencies in a user-owned prefix, without sudo.

ROS 2 Humble, CMake, a compiler, OpenSSL, Asio, TinyXML2 and spdlog development
packages must already be present. Agent libraries never enter the ROS prefix.
"""
import argparse
import concurrent.futures
import io
import json
import os
import pathlib
import subprocess
import tarfile
import tempfile
import urllib.request

PINS = {
    'px4_msgs': ('PX4/px4_msgs', '86d8239e962f6939e05c3737784f60c02fa884db'),
    'fastcdr': ('eProsima/Fast-CDR', '757d5e422253568c487ad97c22e44680fc1ddbaf'),
    'foonathan_memory': ('foonathan/memory', '0f0775770fd1c506fa9c5ad566bd6ba59659db66'),
    'fastdds': ('eProsima/Fast-DDS', 'adc13326bdb4b9df05405102238ea205d1974478'),
    'agent': ('eProsima/Micro-XRCE-DDS-Agent', '73622810d984349b80bbac0ef55fc0b694d62222'),
}


def fetch(repo, commit, destination):
    destination = pathlib.Path(destination)
    stamp = destination / '.source-commit'
    if stamp.exists():
        if stamp.read_text().strip() != commit:
            raise RuntimeError(f'Existing source version differs: {destination}')
        return
    if destination.exists():
        raise RuntimeError(f'Unversioned directory already exists: {destination}')
    request = urllib.request.Request(f'https://api.github.com/repos/{repo}/tarball/{commit}',
                                     headers={'User-Agent': 'ORB-SLAM3-PX4-setup'})
    with urllib.request.urlopen(request, timeout=60) as response:
        archive = response.read()
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='.source-', dir=destination.parent) as temporary:
        staging = pathlib.Path(temporary)/'source'
        staging.mkdir()
        with tarfile.open(fileobj=io.BytesIO(archive)) as tar:
            for member in tar.getmembers():
                parts = pathlib.PurePosixPath(member.name).parts[1:]
                if not parts:
                    continue
                if '..' in parts or not (member.isfile() or member.isdir() or member.issym() or member.islnk()):
                    raise RuntimeError(f'Unsupported archive member: {member.name}')
                member.name = str(pathlib.Path(*parts))
                if member.issym():
                    target = pathlib.PurePosixPath(member.name).parent / member.linkname
                    normalized = os.path.normpath(str(target))
                    if target.is_absolute() or normalized == '..' or normalized.startswith('../'):
                        raise RuntimeError(f'Archive symlink escapes source: {member.name}')
                elif member.islnk():
                    target = pathlib.PurePosixPath(member.linkname)
                    if target.is_absolute() or '..' in target.parts:
                        raise RuntimeError(f'Archive hardlink escapes source: {member.name}')
                    member.linkname = str(pathlib.PurePosixPath(*target.parts[1:]))
                tar.extract(member, staging)
        (staging/'.source-commit').write_text(commit + '\n')
        staging.rename(destination)
    print(f'Fetched {repo}@{commit}', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=pathlib.Path, default=pathlib.Path.home()/'.cache/orbslam3-px4')
    parser.add_argument('--agent', action='store_true', help='build standalone Agent 2.4.3 and its private DDS dependencies')
    parser.add_argument('--messages', action='store_true', help='build px4_msgs for ROS 2 Humble')
    parser.add_argument('--jobs', type=int, default=3)
    args = parser.parse_args()
    if not args.agent and not args.messages:
        parser.error('choose --messages and/or --agent')
    root = args.root.resolve()
    names = (['px4_msgs'] if args.messages else []) + (['fastcdr', 'foonathan_memory', 'fastdds', 'agent'] if args.agent else [])
    def source(name):
        return root/'ros2_ws/src/px4_msgs' if name == 'px4_msgs' else root/'sources'/name
    with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
        futures = [pool.submit(fetch, *PINS[name], source(name)) for name in names]
        for future in futures:
            future.result()
    (root/'versions.json').write_text(json.dumps(PINS, indent=2)+'\n')
    if args.messages:
        # Pass the path as a positional shell argument, not interpolated shell code.
        subprocess.run(['bash', '-c', 'source /opt/ros/humble/setup.bash && cd "$1" && '
                        'colcon build --packages-select px4_msgs --cmake-args '
                        '-DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3 '
                        '-DPYTHON_EXECUTABLE=/usr/bin/python3', 'px4-setup', str(root/'ros2_ws')], check=True,
                       env={**os.environ, 'CMAKE_BUILD_PARALLEL_LEVEL': str(args.jobs)})
    if args.agent:
        prefix = root/'agent_install'
        extra = {
            'fastcdr': ['-DBUILD_TESTING=OFF'],
            'foonathan_memory': ['-DFOONATHAN_MEMORY_BUILD_EXAMPLES=OFF', '-DFOONATHAN_MEMORY_BUILD_TESTS=OFF',
                                  '-DFOONATHAN_MEMORY_BUILD_TOOLS=ON'],
            'fastdds': ['-DBUILD_TESTING=OFF', '-DCOMPILE_EXAMPLES=OFF', '-DCOMPILE_TOOLS=OFF',
                        '-DSECURITY=OFF', '-DSHM_TRANSPORT_DEFAULT=OFF', '-DTHIRDPARTY=OFF'],
            'agent': ['-DUAGENT_SUPERBUILD=OFF', '-DUAGENT_P2P_PROFILE=OFF', '-DUAGENT_CED_PROFILE=OFF',
                      '-DUAGENT_USE_SYSTEM_LOGGER=ON'],
        }
        for name in ['fastcdr', 'foonathan_memory', 'fastdds', 'agent']:
            build = root/'build'/name
            subprocess.run(['cmake', '-S', str(source(name)), '-B', str(build), '-DCMAKE_BUILD_TYPE=Release',
                            '-DCMAKE_POSITION_INDEPENDENT_CODE=ON', '-DBUILD_SHARED_LIBS=ON',
                            f'-DCMAKE_INSTALL_PREFIX={prefix}', f'-DCMAKE_PREFIX_PATH={prefix}',
                            f'-DCMAKE_INSTALL_RPATH={prefix}/lib'] + extra[name], check=True)
            subprocess.run(['cmake', '--build', str(build), '-j', str(args.jobs)], check=True)
            subprocess.run(['cmake', '--install', str(build)], check=True)
        print(f'Agent: {prefix}/bin/MicroXRCEAgent')


if __name__ == '__main__':
    main()
