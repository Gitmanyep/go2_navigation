#!/usr/bin/env python3
"""Short-pulse factory high-level teleoperation. Preview is the default."""
import argparse, datetime, fcntl, hashlib, json, math, os, signal, sys
import threading, time
from pathlib import Path

DIRECTIONS = {'forward': (.1,0.,0.), 'backward': (-.1,0.,0.),
              'left': (0.,0.,.25), 'right': (0.,0.,-.25)}

class PulseController:
    """No SDK or GUI dependencies; a worker ticks independently of the GUI."""
    def __init__(self, client, emit=lambda *args, **kw: None, clock=time.monotonic):
        self.client, self.emit, self.clock = client, emit, clock
        self.lock = threading.RLock()
        self.command = (0.,0.,0.)
        self.deadline = 0.
        self.active = self.pending_stop = False
        self.fault = None
        self.closing = False
    def request(self, velocity, duration=.3):
        with self.lock:
            try:
                v = tuple(float(x) for x in velocity)
                duration = float(duration)
                if len(v)!=3 or not all(math.isfinite(x) for x in v+(duration,)):
                    raise ValueError('Command must contain finite numbers')
                if math.hypot(v[0],v[1]) > .100001 or abs(v[2]) > .400001:
                    raise ValueError('Speed exceeds manual test limits')
                if not .05 <= duration <= .5:
                    raise ValueError('Duration must be between 0.05 and 0.5 seconds')
            except (TypeError,ValueError,OverflowError) as e:
                self.active = False
                self.pending_stop = True
                self.emit('rejected',reason=str(e))
                raise ValueError(str(e))
            if self.closing or self.fault:
                raise RuntimeError(self.fault or 'Controller is closing')
            # Do not queue actions or extend the current deadline on repeated clicks.
            if self.active or self.pending_stop:
                return False
            self.command = v
            self.deadline = self.clock()+duration
            self.active = True
            self.emit('request',velocity=v,duration_s=duration)
            return True
    def stop(self, reason='operator'):
        with self.lock:
            self.active = False
            self.pending_stop = True
            self.emit('stop_requested',reason=reason)
    def _stop_output(self):
        errors=[]
        for name, args in [('Move',(0.,0.,0.)),('StopMove',())]:
            try:
                code=getattr(self.client,name)(*args)
                self.emit('stop_result',operation=name,code=code)
                if code!=0:errors.append('%s code %r'%(name,code))
            except Exception as e:
                errors.append('%s: %s'%(name,e))
        self.pending_stop=False
        if errors:
            self.fault='停止未确认，请用原厂遥控器接管。'+'; '.join(errors)
            self.emit('fault',reason=self.fault)
    def tick(self):
        with self.lock:
            if self.active and self.clock() >= self.deadline:
                self.active=False
                self.pending_stop=True
                self.emit('pulse_expired')
            if self.pending_stop:
                self._stop_output()
                return
            if self.active and not self.fault:
                try:
                    code=self.client.Move(*self.command)
                    self.emit('move_result',velocity=self.command,code=code)
                    if code!=0:raise RuntimeError('Move code %r'%code)
                except Exception as e:
                    self.active=False
                    self.pending_stop=True
                    self.fault=str(e)
                    self._stop_output()
                    self.emit('fault',reason=self.fault)
    def close(self):
        with self.lock:
            self.closing=True
            # Never emit SDK calls just because a preview or unused panel closes.
            if self.active:
                self.stop('window_close')
            self.tick()
    def state(self):
        with self.lock:
            return dict(active=self.active,remaining_s=max(0.,self.deadline-self.clock()) if self.active else 0.,fault=self.fault)

class PreviewClient:
    def Move(self,*args):return 0
    def StopMove(self):return 0

class SessionLog:
    def __init__(self, live):
        now=datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8)))
        self.root=Path.home()/'go2_logs'/('manual_panel_'+now.strftime('%Y%m%d_%H%M%S')+'_'+str(os.getpid()))
        self.root.mkdir(mode=0o700,parents=True)
        self.lock=threading.Lock()
        self.stream=(self.root/'events.jsonl').open('a',buffering=1)
        self.meta={'started_at':now.isoformat(),'argv':sys.argv,'pid':os.getpid(),
                   'mode':'live' if live else 'preview','script_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                   'linear_limit_mps':.1,'angular_limit_radps':.4,'pulse_limit_s':.5,
                   'automatic_obstacle_avoidance':False}
        self.save()
    def save(self):
        (self.root/'metadata.json').write_text(json.dumps(self.meta,ensure_ascii=False,indent=2)+'\n')
    def emit(self,event,**fields):
        with self.lock:
            record={'wall_time':datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8))).isoformat(),
                    'monotonic_s':time.monotonic(),'event':event,**fields}
            self.stream.write(json.dumps(record,ensure_ascii=False,allow_nan=False)+'\n')
    def finish(self):
        self.meta['ended_at']=datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8))).isoformat()
        self.save()
        self.stream.close()

def real_client(iface, emit=lambda *args, **kwargs: None):
    # Called only by an explicit --live invocation. No posture, mode or avoidance changes.
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
    from unitree_sdk2py.go2.sport.sport_client import SportClient
    from unitree_sdk2py.go2.obstacles_avoid.obstacles_avoid_client import ObstaclesAvoidClient
    from unitree_sdk2py.core.channel import ChannelSubscriber
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
    from factory_backend import FactoryVelocityClient, StateFeedback
    ChannelFactoryInitialize(0,iface)
    switch=MotionSwitcherClient();switch.SetTimeout(1.);switch.Init()
    code,mode=switch.CheckMode()
    if code!=0 or not isinstance(mode,dict) or mode.get('name')!='mcf':
        raise RuntimeError('原厂 mcf 控制模式未确认：%r %r'%(code,mode))
    client=SportClient();client.SetTimeout(.3);client.Init()
    version_code,server_version=client.GetServerApiVersion()
    if version_code!=0 or server_version!=client.GetApiVersion():
        raise RuntimeError('Sport 服务版本未确认：%r %r'%(version_code,server_version))
    avoid=ObstaclesAvoidClient();avoid.SetTimeout(.3);avoid.Init()
    avoid_code,enabled=avoid.SwitchGet()
    if avoid_code!=0 or type(enabled) is not bool:
        raise RuntimeError('原厂避障开关未确认：%r %r'%(avoid_code,enabled))
    if enabled:
        avoid_code,avoid_version=avoid.GetServerApiVersion()
        if avoid_code!=0 or avoid_version!=avoid.GetApiVersion():
            raise RuntimeError('原厂避障服务版本未确认：%r %r'%(avoid_code,avoid_version))
    feedback=StateFeedback()
    result=FactoryVelocityClient(client,avoid,enabled,feedback,switch.CheckMode,emit)
    subscriber=ChannelSubscriber('rt/sportmodestate',SportModeState_)
    subscriber.Init(feedback.observe,1);result.subscriber=subscriber
    return result,dict(mode,backend=result.backend,avoidance_enabled=enabled,sport_api_version=server_version)

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--live',action='store_true',help='Connect to factory SDK; movement only after a button click')
    p.add_argument('--iface',default='eth0')
    p.add_argument('--pulse-seconds',type=float,choices=(.3,.5),default=.3,
                   help='Initial short-pulse duration; hard maximum remains 0.5 seconds')
    args=p.parse_args()
    log=SessionLog(args.live)
    log.meta['initial_pulse_s']=args.pulse_seconds;log.save()
    lockfile=None
    try:
        if args.live:
            lockfile=(Path(__file__).parent/'.manual_live.lock').open('a')
            fcntl.flock(lockfile,fcntl.LOCK_EX|fcntl.LOCK_NB)
            client,mode=real_client(args.iface,log.emit)
            log.emit('connected_readonly',mode=mode,iface=args.iface)
            log.meta.update(backend=client.backend,factory_avoidance_enabled=client.avoidance_enabled,
                            min_test_body_height_m=client.feedback.MIN_TEST_BODY_HEIGHT_M)
            log.save()
        else:
            client=PreviewClient()
            log.emit('preview_no_sdk')
    except Exception as e:
        log.emit('startup_error',reason=str(e));log.finish()
        print('无法启动实机控制：',e,file=sys.stderr)
        return 1
    import tkinter as tk
    import tkinter.font as tkfont
    from tkinter import ttk
    root=tk.Tk();root.title('Go2 手动控制 — '+('实机' if args.live else '预览，不发送指令'))
    root.geometry('840x700')
    # Conda Tk on this Jetson exposes X11 Chinese bitmap fonts, not fontconfig aliases.
    families=set(tkfont.families(root))
    family='fangsong ti' if 'fangsong ti' in families else 'Sans'
    for name in ['TkDefaultFont','TkTextFont','TkMenuFont','TkHeadingFont']:
        tkfont.nametofont(name).configure(family=family,size=12,weight='normal')
    ttk.Style(root).configure('TButton',font=(family,13))
    controller=PulseController(client,log.emit)
    finished=threading.Event()
    def worker():
        while not finished.is_set():
            try:
                if args.live and client.needs_release():controller.stop('arm_lease_expired')
                controller.tick()
            except Exception as e:
                controller.fault=str(e)
                controller.active=False
                try:controller.stop('worker_fault');controller.tick()
                except Exception:pass
            finished.wait(.05)
    thread=threading.Thread(target=worker,daemon=True);thread.start()
    panel=ttk.Frame(root,padding=20);panel.pack(fill='both',expand=True)
    ttk.Label(panel,text='实机短动作控制' if args.live else '预览模式：不会发送机器人指令',font=(family,16)).pack(pady=6)
    ttk.Label(panel,text='前后请求速度 0.1 米/秒；转向请求速度 0.25 弧度/秒。').pack(pady=4)
    duration=tk.StringVar(value='%.1f'%args.pulse_seconds)
    duration_row=ttk.Frame(panel);duration_row.pack(pady=4)
    ttk.Label(duration_row,text='本次短动作时长（秒）：').pack(side='left')
    duration_box=ttk.Combobox(duration_row,textvariable=duration,values=('0.3','0.5'),state='readonly',width=5)
    duration_box.pack(side='left')
    if args.live:
        ttk.Label(panel,text='当前通道：'+client.backend+'；原厂避障：'+('开启（保持原状）' if client.avoidance_enabled else '关闭（保持原状）')).pack(pady=4)
        ttk.Label(panel,text='先确认站稳并接管，再点一个方向；动作结束交还 API 指令源。\n接管期间遥控接管效果须现场验证，停止异常时用原厂遥控器处理。',foreground='#a34a00',justify='center').pack(pady=4)
    else:
        ttk.Label(panel,text='预览不会连接 SDK，也不会改变原厂避障或控制来源。').pack(pady=4)
    status=tk.StringVar(value='等待指令')
    ttk.Label(panel,textvariable=status,font=(family,14),wraplength=790).pack(pady=8)
    feedback_status=tk.StringVar(value='')
    if args.live:
        def arm():
            with controller.lock:
                if controller.closing or controller.active or controller.pending_stop or controller.fault:return
                try:client.arm()
                except Exception as e:
                    log.emit('arm_failed',reason=str(e))
                    controller.fault='接管失败：'+str(e)
                    controller.stop('arm_failed')
        arm_button=ttk.Button(panel,text='我已确认站稳：接管本次短动作（5 秒内点击方向）',command=arm)
        arm_button.pack(pady=4)
        ttk.Label(panel,textvariable=feedback_status,wraplength=790).pack(pady=4)
    grid=ttk.Frame(panel);grid.pack(pady=8)
    direction_buttons=[]
    def action(name):
        try:
            if controller.request(DIRECTIONS[name],duration=float(duration.get())):status.set(('发送请求：' if args.live else '预览：')+name)
        except Exception as e:status.set(str(e))
    for name,text,row,col in [('forward','前进 ↑',0,1),('left','左转 ↶',1,0),('right','右转 ↷',1,2),('backward','后退 ↓',2,1)]:
        button=ttk.Button(grid,text=text,width=16,command=lambda n=name:action(n))
        button.grid(row=row,column=col,padx=6,pady=5,ipady=10)
        if args.live:button.configure(state='disabled')
        direction_buttons.append(button)
    ttk.Button(grid,text='停止并交还（空格）',width=16,command=lambda:controller.stop('stop_button')).grid(row=1,column=1,padx=6,pady=5,ipady=10)
    ttk.Label(panel,text='动作中再次点击会被忽略，不会排队或延长动作。\n窗口失去焦点时请求停止；请先用预览核对操作。',justify='center').pack(pady=8)
    ttk.Label(panel,text='本次记录：'+str(log.root),wraplength=650).pack(pady=6)
    root.bind('<space>',lambda e:controller.stop('space'))
    def check_focus():
        try:
            if root.focus_displayof() is None and (controller.state()['active'] or (args.live and (client.armed or client.api_owned))):
                controller.stop('focus_lost')
        except tk.TclError:pass
    root.bind('<FocusOut>',lambda e:root.after(25,check_focus))
    def refresh():
        s=controller.state()
        duration_box.configure(state='disabled' if s['active'] or (args.live and client.armed) else 'readonly')
        if args.live:
            armed=client.armed and not client.needs_release()
            for button in direction_buttons:button.configure(state='normal' if armed and not s['fault'] and not s['active'] else 'disabled')
            readiness_error=None
            try:client.feedback.require_test_height()
            except RuntimeError as e:readiness_error=str(e)
            arm_button.configure(state='disabled' if armed or s['fault'] or s['active'] or controller.pending_stop or readiness_error else 'normal')
            feedback=client.feedback.snapshot()
            if feedback:
                feedback_status.set('实际反馈：高度 %.2fm；v=(%.3f, %.3f, %.3f) m/s；接收年龄 %.2fs；mode=%s，error_code=%s（原始值，不据此判定就绪）'%(feedback['body_height'],*feedback['velocity'],feedback['receive_age_s'],feedback['mode'],feedback['error_code']))
            else:feedback_status.set('等待机身状态反馈；尚未确认运动执行')
        if s['fault']:status.set(s['fault']+'；故障锁定，核对实物状态后重新打开面板')
        elif args.live and readiness_error and not s['active']:status.set(readiness_error)
        elif s['active']:status.set(('正在发送请求（不是执行确认）' if args.live else '预览动作')+'：剩余 %.2f 秒'%s['remaining_s'])
        else:status.set(('已接管，等待一次方向点击' if client.armed else '未接管；请确认实物停止后再接管') if args.live else '预览就绪，不发送指令')
        root.after(100,refresh)
    closed=[False]
    def close():
        if closed[0]:return
        closed[0]=True
        if args.live and (client.armed or client.api_owned):controller.stop('window_close_release')
        controller.close();finished.set();thread.join(timeout=2.)
        log.emit('closed',state=controller.state());log.finish();root.destroy()
    root.protocol('WM_DELETE_WINDOW',close)
    signal.signal(signal.SIGINT,lambda *_:root.after(0,close))
    signal.signal(signal.SIGTERM,lambda *_:root.after(0,close))
    root.after(100,refresh)
    print('MODE',log.meta['mode'],'LOG',log.root,flush=True)
    root.mainloop()
    return 0
if __name__=='__main__':sys.exit(main())
