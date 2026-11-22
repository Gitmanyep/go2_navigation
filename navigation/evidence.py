"""Read local acceptance evidence, retain hashes, and replay the measured occupancy snapshot.
Historical snapshots are not new sensor samples or closed-loop terrain validation.
"""
from pathlib import Path
import argparse
import hashlib
import json
import numpy as np
from core import config,astar
from scipy import ndimage


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for chunk in iter(lambda:f.read(1024*1024),b''):
            h.update(chunk)
    return h.hexdigest()


def pcd_xyz(path,stride=1):
    """Read PCD ASCII/binary, including this project's x,y,z,normal_* layout.
    Compressed PCD is rejected explicitly. File is memory-mapped, not fully loaded.
    """
    if stride < 1:
        raise ValueError('stride must be >=1')
    header = {}
    with Path(path).open('rb') as f:
        for _ in range(100):
            line = f.readline().decode('ascii').strip()
            if not line or line.startswith('#'):continue
            key,*values = line.split();header[key] = values
            if key=='DATA':break
        else:
            raise ValueError('invalid_pcd_header')
        offset = f.tell()
    if not set(['x','y','z'])<=set(header['FIELDS']):
        raise ValueError('pcd_missing_xyz')
    if header['DATA']==['ascii']:
        with Path(path).open('rb') as f:
            f.seek(offset);a = np.loadtxt(f,ndmin=2)[::stride]
        points = a[:,[header['FIELDS'].index(k) for k in ['x','y','z']]]
    elif header['DATA']==['binary']:
        sizes = list(map(int,header['SIZE']));counts = list(map(int,header.get('COUNT',['1']*len(sizes))))
        formats = []
        for typ,size,count in zip(header['TYPE'],sizes,counts):
            code = {'F':'f','I':'i','U':'u'}[typ]+str(size)
            formats.append(('<'+code,(count,)) if count>1 else '<'+code)
        dtype = np.dtype(list(zip(header['FIELDS'],formats)))
        n = int(header['POINTS'][0])
        if Path(path).stat().st_size-offset < n*dtype.itemsize:
            raise ValueError('truncated_pcd')
        a = np.memmap(path,mode='r',offset=offset,dtype=dtype,shape=(n,))[::stride]
        points = np.column_stack([a[k] for k in ['x','y','z']]).astype(float)
    else:
        raise ValueError('unsupported_pcd_encoding')
    return points[np.isfinite(points).all(axis=1)]


def build(project,output):
    project,output = Path(project),Path(output)
    output.mkdir(parents=True,exist_ok=False)
    relative = ['readme.txt','file/静态规划运行与测试汇总.json',
                'tools/go2_static_nav_20261121/real_final_grid.npz',
                'tools/go2_static_nav_20261121/real_final_status.json',
                'tools/go2_static_nav_20261121/source_timing_audit.json',
                'tools/session_audit_20261121/early_static_validation/acceptance_summary.json']
    files = [project/r for r in relative]
    missing = [str(f) for f in files if not f.is_file()]
    if missing:
        raise FileNotFoundError(str(missing))
    summary = json.loads(files[1].read_text())
    stationary = json.loads(files[5].read_text())['stationary_measurements']
    timing = json.loads(files[4].read_text())
    evidence = dict(project=str(project),sources=[dict(path=str(f),sha256=digest(f)) for f in files],
                    measured=dict(lidar_hz=stationary['topics']['lidar']['hz'],
                                  imu_hz=stationary['topics']['imu']['hz'],
                                  cloud_hz=stationary['topics']['cloud']['hz'],
                                  stationary_axis_span_m=stationary['pose']['axis_span_m'],
                                  stationary_yaw_span_deg=stationary['pose']['yaw_span_deg'],
                                  registered_frame=stationary['pose']['header_frame'],
                                  recovered_source_age_min_s=summary['real_health']['min_source_age_s'],
                                  recovered_source_age_max_s=summary['real_health']['max_source_age_s'],
                                  backlog_s=[15,20]),
                    assumptions=['slope limits are simulation settings, not measured Go2 capability',
                                 '0.45m footprint is provisional; no verified chassis/lidar extrinsic',
                                 'synthetic registered point samples do not model Livox optics or Point-LIO estimation error',
                                 'stationary.bag is referenced in records but absent in this local checkout',
                                 'old 256-sector scans lack an explicit observed mask and cannot certify terrain'],
                    scenario_mapping=dict(sensor_delay='0.30-0.40s stress; measured healthy age was 0.08-0.18s',
                                          backlog_20s='source timestamp preserved through 20s delivery delay',
                                          cloud_dropout='3s missing cloud',odom_dropout='3s missing odometry',
                                          direction_dropout='3s missing direction'),
                    timing_snapshot=timing)
    (output/'data_profile.json').write_text(json.dumps(evidence,ensure_ascii=False,indent=2))
    a = np.load(files[2],allow_pickle=False)
    status = json.loads(files[3].read_text())
    grid = a['grid'];res = status['resolution'];origin = a['origin']
    pose = a['pose'];goal = np.asarray(status['goal'])
    padded = np.pad(grid==0,1,constant_values=False)
    clearance = ndimage.distance_transform_edt(padded)[1:-1,1:-1]*res
    blocked = clearance <= .45+res*np.sqrt(.5)
    zero = np.zeros_like(grid,float)
    terrain = dict(blocked=blocked,slope=zero,gx=zero,gy=zero,clearance=clearance)
    def cell(xy):
        v = np.floor((xy[:2]-origin)/res).astype(int)
        return int(v[0]),int(v[1])
    route = astar(terrain,cell(pose),cell(goal),config())
    report = dict(mode='HISTORICAL_STATIC_2D_REPLAY',terrain_available=False,
                  path_found=bool(route),cells=len(route),unknown_fraction=float((grid<0).mean()),
                  source_stamp=status['source_stamp'],frame=status['frame'],
                  note='Flatness is only a replay assumption; occupancy snapshot contains no 3D terrain.',
                  executable=False,motion_commands_sent=0)
    (output/'real_snapshot_replay.json').write_text(json.dumps(report,indent=2))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,ax = plt.subplots(figsize=(7,6))
    extent = [origin[0],origin[0]+grid.shape[1]*res,origin[1],origin[1]+grid.shape[0]*res]
    from matplotlib.colors import ListedColormap
    ax.imshow(np.where(grid<0,0,np.where(grid==0,1,2)),origin='lower',extent=extent,cmap=ListedColormap(['#535d6b','#f1f3f5','#dc5353']),vmin=0,vmax=2)
    if route:
        points = origin+(np.asarray(route)+.5)*res
        ax.plot(points[:,0],points[:,1],c='#1769d2',lw=2)
    ax.scatter(*pose[:2],c='green');ax.scatter(*goal[:2],c='purple')
    ax.set(title='Measured occupancy snapshot replay | flat-ground assumption only',xlabel='x (m)',ylabel='y (m)')
    fig.tight_layout();fig.savefig(output/'real_snapshot_replay.png',dpi=130);plt.close(fig)
    print(json.dumps(report))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--project',required=True);p.add_argument('--output',required=True)
    a = p.parse_args();build(a.project,a.output)

if __name__=='__main__':main()
