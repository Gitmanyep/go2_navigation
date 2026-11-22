#!/usr/bin/env python3
"""Read-only speed monitor; live mode only subscribes to rt/sportmodestate."""
import argparse
import datetime
import hashlib
import json
import math
import signal
import threading
import time
from pathlib import Path
from types import SimpleNamespace
from telemetry import TelemetryRecorder


def connect(callback,iface):
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
    ChannelFactoryInitialize(0,iface)
    subscriber=ChannelSubscriber('rt/sportmodestate',SportModeState_)
    subscriber.Init(callback,1)
    return subscriber


def demo(recorder,finished):
    start=time.monotonic();x=yaw=0.;previous=start
    while not finished.is_set():
        now=time.monotonic();dt=now-previous;previous=now;t=now-start
        speed=.4*(1.-math.cos(t*1.5));rate=.3*math.sin(t)
        x+=speed*dt;yaw+=rate*dt
        recorder.observe(SimpleNamespace(velocity=[speed,0.,0.],position=[x,0.,0.],
                         yaw_speed=rate,imu_state=SimpleNamespace(rpy=[0.,0.,yaw])))
        finished.wait(.05)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    group=p.add_mutually_exclusive_group(required=True)
    group.add_argument('--live',action='store_true')
    group.add_argument('--demo',action='store_true')
    p.add_argument('--iface',default='eth0')
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--headless',action='store_true')
    p.add_argument('--seconds',type=float,help='Optional bounded recording duration')
    a=p.parse_args()
    if a.seconds is not None and (not math.isfinite(a.seconds) or a.seconds<=0):p.error('seconds must be positive and finite')
    if a.headless and a.seconds is None:p.error('headless mode requires --seconds')
    recorder=TelemetryRecorder(a.output,source='robot_internal_estimate' if a.live else 'synthetic')
    meta=dict(mode='live_readonly' if a.live else 'demo_synthetic',iface=a.iface,
              robot_motion_backend_enabled=False,control_apis_used=[],topic='rt/sportmodestate' if a.live else None,
              sample_period_s=.05,reception_timeout_s=.3,source='robot_internal_estimate' if a.live else 'synthetic',
              source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    (a.output/'metadata.json').write_text(json.dumps(meta,ensure_ascii=False,indent=2)+'\n')
    finished=threading.Event();subscriber=None;thread=None
    print('READONLY_MONITOR',meta['mode'],'OUTPUT',a.output,flush=True)
    try:
        if a.live:subscriber=connect(recorder.observe,a.iface)
        else:
            thread=threading.Thread(target=demo,args=(recorder,finished),daemon=True);thread.start()
        if a.headless:
            signal.signal(signal.SIGTERM,lambda *_:finished.set())
            signal.signal(signal.SIGINT,lambda *_:finished.set())
            finished.wait(a.seconds)
            print(json.dumps(recorder.snapshot(),ensure_ascii=False),flush=True)
        else:
            import tkinter as tk
            from tkinter import ttk
            import tkinter.font as tkfont
            root=tk.Tk();root.title('Go2 只读速度监测'+(' — 演示数据' if a.demo else ' — 实机反馈'))
            root.geometry('940x720')
            family='fangsong ti' if 'fangsong ti' in set(tkfont.families(root)) else 'Sans'
            tkfont.nametofont('TkDefaultFont').configure(family=family,size=12)
            panel=ttk.Frame(root,padding=12);panel.pack(fill='both',expand=True)
            ttk.Label(panel,text='演示数据，未连接机器狗' if a.demo else '只读订阅：不接管、不站起、不发送速度',font=(family,16)).pack(pady=5)
            ttk.Label(panel,text='以下为合成演示数据，不代表机器狗实测结果。' if a.demo else '数值来自机内估计；位移可能漂移。坐标系未标定时，不把 vx 直接当机身前向速度。',wraplength=900).pack(pady=4)
            status=tk.StringVar();values=tk.StringVar();stats=tk.StringVar();notice=tk.StringVar()
            ttk.Label(panel,textvariable=status,font=(family,14)).pack(pady=4)
            ttk.Label(panel,textvariable=values,font=(family,14),wraplength=900).pack(pady=4)
            ttk.Label(panel,textvariable=stats,wraplength=900).pack(pady=4)
            row=ttk.Frame(panel);row.pack(pady=5)
            def reset():
                recorder.reset();notice.set('统计起点已重置；之前的 CSV 数据仍保留。')
            def save():
                try:recorder.save_summary();notice.set('本段统计已保存到 segments.jsonl')
                except OSError as exc:notice.set('保存失败：'+str(exc))
            ttk.Button(row,text='重置统计起点',command=reset).pack(side='left',padx=5)
            ttk.Button(row,text='保存当前统计',command=save).pack(side='left',padx=5)
            canvas=tk.Canvas(panel,width=900,height=300,bg='#f7fafc');canvas.pack(fill='both',expand=True,pady=6)
            ttk.Label(panel,textvariable=notice,wraplength=900).pack(pady=3)
            ttk.Label(panel,text='连续保存 feedback.csv；关闭时保存统计。目录：'+str(a.output),wraplength=900).pack(pady=4)
            def fmt(value,unit=''):return '不可用' if value is None else '%.3f%s'%(value,unit)
            def refresh():
                state=recorder.snapshot();sample=state['latest'];fresh=state['fresh']
                status.set(('演示数据 | ' if a.demo else '')+('反馈接收正常，年龄 %.2fs'%state['receive_age_s'] if fresh else '未收到反馈／数据过期，当前速度不可用'))
                if sample and fresh:
                    values.set(('演示数据：平面速度 ' if a.demo else '机内估计：平面速度 ')+fmt(sample['speed'],' m/s')+'；转向角速度 '+fmt(sample['yaw_rate'],' rad/s')+
                               '\n原始速度分量 v=(%.3f, %.3f, %.3f) m/s'%tuple(sample['velocity']))
                else:values.set('当前平面速度：不可用；当前转向角速度：不可用')
                stats.set('本段峰值：速度 '+fmt(state['peak_speed_mps'],' m/s')+'；角速度 '+fmt(state['peak_yaw_rate_rps'],' rad/s')+
                          '\n估计直线位移 '+fmt(state['displacement_m'],' m')+'；累计航向变化 '+fmt(math.degrees(state['yaw_change_rad']) if state['yaw_change_rad'] is not None else None,'°')+
                          '；无效样本 %s'%state['invalid_samples'])
                if state['write_error']:notice.set('数据文件写入异常：'+state['write_error'])
                with recorder.lock:history=list(recorder.history)
                canvas.delete('all');width=max(200,canvas.winfo_width());height=max(150,canvas.winfo_height())
                for index,(key,label,color) in enumerate((('speed','平面速度 m/s','#2166ac'),('yaw_rate','转向角速度 rad/s','#b35806'))):
                    top=15+index*height/2;bottom=(index+1)*height/2-15;left=145;right=width-15
                    series=[x for x in history if history and x['received']>=history[-1]['received']-15]
                    limit=max([1.]+[abs(x[key]) for x in series if x[key] is not None])*1.1
                    center=(top+bottom)/2;scale=(bottom-top)/(2*limit)
                    canvas.create_text(8,top+15,text=label,anchor='w',fill=color)
                    canvas.create_text(8,top+35,text='范围 ±%.2f'%limit,anchor='w')
                    canvas.create_line(left,center,right,center,fill='#b7c2cc')
                    previous=None
                    for x in series:
                        if x[key] is None:previous=None;continue
                        point=(right-(series[-1]['received']-x['received'])/15*(right-left),center-x[key]*scale)
                        if previous and x['received']-previous[0]<=.3:canvas.create_line(*previous[1],*point,fill=color,width=2)
                        previous=(x['received'],point)
                if not finished.is_set():root.after(100,refresh)
            def close():finished.set();root.destroy()
            root.protocol('WM_DELETE_WINDOW',close)
            signal.signal(signal.SIGTERM,lambda *_:root.after(0,close))
            signal.signal(signal.SIGINT,lambda *_:root.after(0,close))
            if a.seconds is not None:root.after(int(a.seconds*1000),close)
            root.after(100,refresh);root.mainloop()
    finally:
        finished.set()
        if thread:thread.join(timeout=1.)
        recorder.close()
    return 0


if __name__=='__main__':raise SystemExit(main())
