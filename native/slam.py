"""Query native Go2 mapping/navigation services through their DDS command topic."""
import argparse
import json
import os
from pathlib import Path
import sys
import time
ROOT=Path(__file__).resolve().parents[1]


def main():
    p=argparse.ArgumentParser()
    p.add_argument('action',choices=['status','fetch-map','mapping-start','mapping-stop'])
    args=p.parse_args()
    cfg=json.loads((ROOT/'config/native_lidar.json').read_text())
    sys.path.insert(0,str(ROOT/'vendor/unitree_sdk2_python'))
    sys.path.append(cfg['dds_runtime'])
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize,ChannelFactory,ChannelPublisher
    from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
    from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_
    from cyclonedds.domain import DomainParticipant
    from cyclonedds.topic import Topic
    from cyclonedds.sub import DataReader
    from cyclonedds.qos import Qos,Policy
    from cyclonedds.internal import InvalidSample
    ChannelFactoryInitialize(0,cfg['interface'])
    participant=next(v for v in vars(ChannelFactory()).values() if isinstance(v,DomainParticipant))
    reader=DataReader(participant,Topic(participant,'rt/uslam/server_log',String_),
                      Qos(Policy.Reliability.BestEffort,Policy.History.KeepLast(30)))
    map_reader=DataReader(participant,Topic(participant,'rt/uslam/cloud_map',PointCloud2_),
                          Qos(Policy.Reliability.BestEffort,Policy.History.KeepLast(1)))
    writer=ChannelPublisher('rt/uslam/client_command',String_);writer.Init()
    report=dict(source=str(ROOT),sdk=str(ROOT/'vendor/unitree_sdk2_python'),
                command_topic='rt/uslam/client_command',motion_commands_sent=0,commands=[],feedback=[])
    time.sleep(.8)
    queries=('frontend/get_status','mapping/get_status','localization/get_status',
             'navigation/get_status','patrol/get_status','common/get_map_id') if args.action=='status' else {
                 'fetch-map':('mapping/get_cloud_map',),
                 'mapping-start':('mapping/start','mapping/get_status'),
                 'mapping-stop':('mapping/stop','mapping/get_status','mapping/get_cloud_map')}[args.action]
    map_sample=None
    for query in queries:
        sent=time.time()
        print('QUERY',query,flush=True)
        writer.Write(String_(query))
        report['commands'].append(dict(command=query,time=sent))
        until=time.monotonic()+(7. if query in ('mapping/get_cloud_map','mapping/start','mapping/stop') else 1.5)
        while time.monotonic()<until:
            for m in map_reader.take(1):
                if not isinstance(m,InvalidSample):
                    map_sample=m
            for sample in reader.take(30):
                if isinstance(sample,InvalidSample):continue
                feedback=dict(time=time.time(),after_query=query,text=sample.data[:8000])
                report['feedback'].append(feedback)
                print('FEEDBACK',feedback['text'],flush=True)
            time.sleep(.03)
    if map_sample is not None:
        backup=ROOT/'logs'/('native_saved_map_'+time.strftime('%Y%m%d_%H%M%S'))
        backup.mkdir(parents=True,exist_ok=False)
        (backup/'data.bin').write_bytes(bytes(map_sample.data))
        header=dict(frame=map_sample.header.frame_id,stamp_sec=map_sample.header.stamp.sec,
                    stamp_nanosec=map_sample.header.stamp.nanosec,width=map_sample.width,height=map_sample.height,
                    point_step=map_sample.point_step,row_step=map_sample.row_step,is_bigendian=map_sample.is_bigendian,
                    is_dense=map_sample.is_dense,fields=[dict(name=f.name,offset=f.offset,datatype=f.datatype,count=f.count) for f in map_sample.fields])
        (backup/'metadata.json').write_text(json.dumps(header,indent=2))
        report['map_backup']=str(backup)
        print('MAP_BACKUP',backup,flush=True)
    f=ROOT/'logs'/('native_slam_status_'+time.strftime('%Y%m%d_%H%M%S')+'.json')
    f.parent.mkdir(exist_ok=True)
    with f.open('x') as stream:json.dump(report,stream,indent=2)
    print('REPORT',f,flush=True)
    return 0 if report['feedback'] else 1


if __name__=='__main__':raise SystemExit(main())
