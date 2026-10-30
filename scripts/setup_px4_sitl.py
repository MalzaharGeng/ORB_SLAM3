#!/usr/bin/env python3
"""Build pinned PX4 1.17 SIH without Gazebo, using verified source archives.

Install Tools/setup Python requirements (including ninja) into an isolated venv
first. Git metadata is local build scaffolding; snapshot_manifest.json records
the actual upstream revisions. No flight-controller firmware is installed.
"""
import argparse
import concurrent.futures
import configparser
import json
import os
import pathlib
import subprocess
import sys
import urllib.request
from setup_px4_dependencies import fetch

PX4_COMMIT = 'f65ddce53985f13cd4fa24e7c0d78345b467bf1f'
MICROCDR_COMMIT = '3d1b17703c7cf4f22def2910bc845bdb5152d7b5' # v2.0.1


def api(path):
    request = urllib.request.Request('https://api.github.com/repos/'+path, headers={'User-Agent': 'ORB-SLAM3-PX4-SITL'})
    return json.loads(urllib.request.urlopen(request, timeout=45).read())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=pathlib.Path, default=pathlib.Path.home()/'.cache/orbslam3-px4')
    parser.add_argument('--jobs', type=int, default=4)
    args = parser.parse_args()
    root = args.root.resolve()/'PX4-Autopilot'
    fetch('PX4/PX4-Autopilot', PX4_COMMIT, root)
    manifest = []

    def git_metadata(path, commit):
        if (path/'.git').exists():
            return
        subprocess.run(['git', 'init', '-q', str(path)], check=True)
        subprocess.run(['git', '-C', str(path), '-c', 'user.name=Snapshot Builder', '-c', 'user.email=snapshot@localhost',
                        'commit', '--quiet', '--allow-empty', '-m', 'Source archive '+commit], check=True)

    def submodules(repo, commit, path, selected=None):
        if not (path/'.gitmodules').exists():
            return
        cfg = configparser.ConfigParser()
        cfg.read(path/'.gitmodules')
        tree = api(f'{repo}/git/trees/{commit}?recursive=1')
        if tree.get('truncated'):
            raise RuntimeError('GitHub returned an incomplete submodule tree')
        revisions = {item['path']: item['sha'] for item in tree['tree'] if item['type'] == 'commit'}
        jobs = []
        for section in cfg.sections():
            relative = cfg[section]['path']
            if selected is not None and relative not in selected:
                continue
            url = cfg[section]['url']
            if not url.startswith('https://github.com/'):
                raise RuntimeError('Unsupported submodule host: '+url)
            repo = url.removeprefix('https://github.com/').removesuffix('.git')
            jobs.append((repo, revisions[relative], path/relative))

        def get(job):
            repo, commit, path = job
            if path.exists() and not any(path.iterdir()):
                path.rmdir()
            fetch(repo, commit, path)
            git_metadata(path, commit)
            manifest.append({'repo': repo, 'commit': commit, 'path': str(path.relative_to(root))})
            submodules(repo, commit, path)
        with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
            for future in [pool.submit(get, job) for job in jobs]:
                future.result()

    git_metadata(root, PX4_COMMIT)
    if subprocess.run(['git', '-C', str(root), 'rev-parse', '--verify', 'refs/tags/v1.17.0-0.0.1-dev'],
                      stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode:
        subprocess.run(['git', '-C', str(root), 'tag', 'v1.17.0-0.0.1-dev'], check=True)
    submodules('PX4/PX4-Autopilot', PX4_COMMIT, root, {
        'src/drivers/gps/devices', 'src/lib/events/libevents', 'src/modules/mavlink/mavlink',
        'src/modules/uxrce_dds_client/Micro-XRCE-DDS-Client', 'src/lib/heatshrink/heatshrink'})
    micro_commit = MICROCDR_COMMIT
    micro = args.root.resolve()/'sources/microcdr'
    fetch('eProsima/Micro-CDR', micro_commit, micro)
    # Only acquisition changes: use the exact upstream Micro-CDR source locally,
    # since git transport may be unavailable even when archive downloads work.
    superbuild = root/'src/modules/uxrce_dds_client/Micro-XRCE-DDS-Client/cmake/SuperBuild.cmake'
    text = superbuild.read_text()
    text = text.replace('GIT_REPOSITORY\n            https://github.com/eProsima/Micro-CDR.git\n'
                        '        GIT_TAG\n            ${_microcdr_tag}',
                        f'SOURCE_DIR "{micro}"\n        DOWNLOAD_COMMAND ""\n        UPDATE_COMMAND ""')
    superbuild.write_text(text)
    (root/'snapshot_manifest.json').write_text(json.dumps({
        'px4_upstream_commit': PX4_COMMIT, 'microcdr_commit': micro_commit, 'submodules': manifest,
        'build_only_changes': ['local empty Git metadata', 'Micro-CDR fetched from archive instead of git'],
    }, indent=2)+'\n')
    environment = dict(os.environ, GIT_SUBMODULES_ARE_EVIL='1')
    build = root/'build/px4_sitl_nolockstep'
    subprocess.run(['cmake', '-S', str(root), '-B', str(build), '-G', 'Ninja',
                    '-DCONFIG=px4_sitl_nolockstep', f'-DPYTHON_EXECUTABLE={sys.executable}'],
                   check=True, env=environment)
    subprocess.run(['cmake', '--build', str(build), '-j', str(args.jobs)], check=True, env=environment)
    print('PX4 SIH binary:', build/'bin/px4')


if __name__ == '__main__':
    main()
