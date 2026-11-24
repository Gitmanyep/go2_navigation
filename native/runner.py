"""Start all native Go2 viewing processes from the current Git checkout."""
import argparse,datetime,json,os,signal,subprocess,sys,time,xmlrpc.client
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--seconds',type=float)
    parser.add_argument('--no-rviz',action='store_true')
    args=parser.parse_args()
    if args.seconds is not None and not 0<args.seconds<=3600:parser.error('invalid seconds')
    cfg=json.loads((ROOT/'config/native_lidar.json').read_text())
    run=ROOT/'logs'/('native_lidar_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S'))
    run.mkdir(parents=True,exist_ok=False)
    env=dict(os.environ,ROS_MASTER_URI='http://127.0.0.1:11311',ROS_IP='127.0.0.1',
        PYTHONDONTWRITEBYTECODE='1',PYTHONNOUSERSITE='1',CYCLONEDDS_HOME=cfg['cyclonedds_home'],
        LD_LIBRARY_PATH=cfg['cyclonedds_home']+'/lib',DISPLAY=cfg['display'],XAUTHORITY=str(Path.home()/'.Xauthority'))
    for k in ('ROS_HOSTNAME','PYTHONHOME','PYTHONPATH'):env.pop(k,None)
    children=[];streams=[];commands=[];stop=False
    def spawn(name,command):
        cmd=['/bin/bash','-c','source /opt/ros/noetic/setup.bash; exec "$@"','go2-native',*command]
        f=(run/(name+'.log')).open('w');streams.append(f)
        child=subprocess.Popen(cmd,cwd=ROOT,env=env,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
        children.append(child);commands.append(dict(name=name,pid=child.pid,command=cmd))
        return child
    def alive():
        try:return xmlrpc.client.ServerProxy(env['ROS_MASTER_URI']).getPid('/native_view_launcher')[0]==1
        except (OSError,xmlrpc.client.Error):return False
    def stopped(*_):
        nonlocal stop
        stop=True
    signal.signal(signal.SIGINT,stopped);signal.signal(signal.SIGTERM,stopped)
    code=0
    try:
        if not alive():
            master=spawn('roscore',['roscore']);deadline=time.monotonic()+8
            while not alive():
                if stop or master.poll() is not None or time.monotonic()>deadline:raise RuntimeError('ROS master failed')
                time.sleep(.2)
        bridge=spawn('bridge',[sys.executable,str(ROOT/'native/bridge.py'),'--output',str(run)])
        if not args.no_rviz:spawn('rviz',['rviz','-d',str(ROOT/'config/native_lidar.rviz')])
        revision=subprocess.run(['git','rev-parse','HEAD'],cwd=ROOT,capture_output=True,text=True).stdout.strip()
        (run/'metadata.json').write_text(json.dumps(dict(source=str(ROOT),commit=revision,sdk=str(ROOT/'vendor/unitree_sdk2_python'),
            settings=cfg,commands=commands,motion_commands_sent=0),indent=2))
        print('SOURCE',ROOT,'LOG',run,flush=True)
        start=time.monotonic()
        while not stop and (args.seconds is None or time.monotonic()-start<args.seconds):
            if bridge.poll() is not None:raise RuntimeError('Bridge exited. See '+str(run/'bridge.log'))
            time.sleep(.2)
    except Exception:
        code=1
        raise
    finally:
        for child in reversed(children):
            if child.poll() is None:os.killpg(child.pid,signal.SIGTERM)
        for child in children:
            try:child.wait(timeout=4)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid,signal.SIGKILL);child.wait()
        for f in streams:f.close()
        (run/'exit.json').write_text(json.dumps(dict(exit_code=code,time=datetime.datetime.now().isoformat())))
    return code
if __name__=='__main__':raise SystemExit(main())
