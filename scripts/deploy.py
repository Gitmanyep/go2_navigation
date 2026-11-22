#!/usr/bin/env python3
"""Clone a clean, committed release into a NEW onboard folder; never replace old trees."""
import argparse
import hashlib
import json
from pathlib import Path
import shlex
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]


def run(args, **kwargs):
    return subprocess.run(args, check=True, **kwargs)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--host', required=True, help='unitree@CURRENT_IP')
    p.add_argument('--socket', required=True, type=Path, help='Authenticated laptop SSH ControlMaster socket')
    p.add_argument('--host-key-alias', default='192.168.123.18')
    p.add_argument('--destination', default='/home/unitree/go2_navigation')
    p.add_argument('--wheels', required=True, type=Path)
    a = p.parse_args()
    if not a.destination.startswith('/home/unitree/') or '..' in Path(a.destination).parts:
        p.error('destination must be a new absolute directory beneath /home/unitree')
    status = subprocess.check_output(['git', '-C', str(ROOT), 'status', '--porcelain'], text=True)
    if status.strip():
        p.error('commit changes before deployment')
    commit = subprocess.check_output(['git', '-C', str(ROOT), 'rev-parse', 'HEAD'], text=True).strip()
    ref = subprocess.check_output(['git', '-C', str(ROOT), 'symbolic-ref', '--short', 'HEAD'], text=True).strip()
    if ref != 'main':
        p.error('deploy main only')
    wheels = a.wheels.resolve()
    if not list(wheels.glob('*.whl')):
        p.error('wheel directory is empty')
    release = '/home/unitree/go2_deployments/'+commit
    options = ['-o', 'ControlPath='+str(a.socket), '-o', 'ControlMaster=no', '-o', 'BatchMode=yes',
               '-o', 'HostKeyAlias='+a.host_key_alias, '-o', 'StrictHostKeyChecking=yes']
    ssh = ['ssh', *options, a.host]
    q = shlex.quote
    # All preconditions precede writes. Existing directories are never merged or overwritten.
    run([*ssh, 'test ! -e '+q(a.destination)+' && test ! -e '+q(release)])
    output = ROOT/'artifacts'/commit
    output.mkdir(parents=True, exist_ok=False)
    bundle = output/'source.bundle'
    run(['git', '-C', str(ROOT), 'bundle', 'create', str(bundle), 'main', '--tags'])
    run(['git', '-C', str(ROOT), 'bundle', 'verify', str(bundle)])
    run([*ssh, 'mkdir -p '+q(release+'/wheels')])
    run(['scp', *options, str(bundle), a.host+':'+release+'/source.bundle'])
    with subprocess.Popen(['tar', '-C', str(wheels), '-cf', '-', '.'], stdout=subprocess.PIPE) as stream:
        result = subprocess.run([*ssh, 'tar -xf - -C '+q(release+'/wheels')], stdin=stream.stdout)
        stream.stdout.close()
        if stream.wait() or result.returncode:
            raise RuntimeError('wheel upload failed; new release artifacts retained for inspection')
    script = ('set -eu\n'
              'git clone --branch main '+q(release+'/source.bundle')+' '+q(a.destination)+'\n'
              'test "$(git -C '+q(a.destination)+' rev-parse HEAD)" = '+q(commit)+'\n'
              +q(a.destination+'/scripts/setup_offline_env.sh')+' '+q(release+'/wheels')+'\n'
              +q(a.destination+'/bin/go2-nav')+' doctor\n')
    run([*ssh, script])
    # Compare every tracked byte after cloning. No assumptions about timestamps or folder names.
    files = subprocess.check_output(['git', '-C', str(ROOT), 'ls-files', '-z']).decode().split('\0')
    expected = {f:hashlib.sha256((ROOT/f).read_bytes()).hexdigest() for f in files if f}
    payload = ('import hashlib,json; from pathlib import Path; r=Path('+repr(a.destination)+'); '
               'names='+repr(list(expected))+'; '
               'print(json.dumps({n:hashlib.sha256((r/n).read_bytes()).hexdigest() for n in names}))')
    actual = json.loads(subprocess.check_output([*ssh, '/usr/bin/python3 -c '+q(payload)], text=True))
    if actual != expected:
        raise RuntimeError('deployed file hash mismatch')
    report = dict(host=a.host, destination=a.destination, commit=commit, tracked_files=len(expected),
                  file_hashes_match=True, bundle_sha256=hashlib.sha256(bundle.read_bytes()).hexdigest(),
                  wheel_sha256={f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in wheels.glob('*.whl')},
                  old_directories_replaced=False, motion_started=False)
    (output/'deployment.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    try:
        main()
    except (OSError, RuntimeError, subprocess.CalledProcessError) as exc:
        print('Deployment stopped:', exc, file=sys.stderr)
        raise SystemExit(1)
