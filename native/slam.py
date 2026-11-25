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
    p.add_argument('action',choices=['status'])
    args=p.parse_args()
    cfg=json.loads((ROOT/'config/native_lidar.json').read_text())
    sys.path.insert(0,str(ROOT/'vendor/unitree_sdk2_python'))
    sys.path.append(cfg['dds_runtime'])
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize,ChannelFactory,ChannelPublisher
    from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
    from cyclonedds.domain import DomainParticipant
    from cyclonedds.topic import Topic
    from cyclonedds.sub import DataReader
    from cyclonedds.qos import Qos,Policy
    from cyclonedds.internal import InvalidSample
    ChannelFactoryInitialize(0,cfg['interface'])
    participant=next(v for v in vars(ChannelFactory()).values() if isinstance(v,DomainParticipant))
    reader=DataReader(participant,Topic(participant,'rt/uslam/server_log',String_),
                      Qos(Policy.Reliability.BestEffort,Policy.History.KeepLast(30)))
    writer=ChannelPublisher('rt/uslam/client_command',String_);writer.Init()
    report=dict(source=str(ROOT),sdk=str(ROOT/'vendor/unitree_sdk2_python'),
                command_topic='rt/uslam/client_command',motion_commands_sent=0,commands=[],feedback=[])
    time.sleep(.8)
    for query in ('frontend/get_status','mapping/get_status','localization/get_status',
                  'navigation/get_status','patrol/get_status','common/get_map_id'):
        sent=time.time()
        print('QUERY',query,flush=True)
        writer.Write(String_(query))
        report['commands'].append(dict(command=query,time=sent))
        until=time.monotonic()+1.5
        while time.monotonic()<until:
            for sample in reader.take(30):
                if isinstance(sample,InvalidSample):continue
                feedback=dict(time=time.time(),after_query=query,text=sample.data[:8000])
                report['feedback'].append(feedback)
                print('FEEDBACK',feedback['text'],flush=True)
            time.sleep(.03)
    f=ROOT/'logs'/('native_slam_status_'+time.strftime('%Y%m%d_%H%M%S')+'.json')
    f.parent.mkdir(exist_ok=True)
    with f.open('x') as stream:json.dump(report,stream,indent=2)
    print('REPORT',f,flush=True)
    return 0 if report['feedback'] else 1


if __name__=='__main__':raise SystemExit(main())
