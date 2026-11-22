"""Explicit factory command ownership; no SDK imports or automatic motion."""
import math
import threading
import time


class StateFeedback:
    # Conservative Go2 test interlock, not a firmware readiness specification.
    MIN_TEST_BODY_HEIGHT_M = .20

    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.lock = threading.Lock()
        self.last = None

    def observe(self, message):
        sample = dict(received=self.clock(), mode=int(message.mode),
                      error_code=int(message.error_code), body_height=float(message.body_height),
                      velocity=[float(x) for x in message.velocity])
        with self.lock:
            self.last = sample

    def snapshot(self):
        with self.lock:
            sample = dict(self.last) if self.last else None
        if sample:
            sample['receive_age_s'] = max(0., self.clock()-sample['received'])
        return sample

    def require_fresh(self):
        sample = self.snapshot()
        if not sample or sample['receive_age_s'] > .3:
            raise RuntimeError('机身状态未收到或接收超时；保持停止')
        if not all(math.isfinite(x) for x in sample['velocity']+[sample['body_height']]):
            raise RuntimeError('机身状态包含非有限值；保持停止')
        return sample

    def require_test_height(self):
        sample = self.require_fresh()
        if sample['body_height'] < self.MIN_TEST_BODY_HEIGHT_M:
            raise RuntimeError('机身反馈高度 %.2fm 低于测试门槛 %.2fm；请先用原厂遥控器站稳并核对反馈' %
                               (sample['body_height'], self.MIN_TEST_BODY_HEIGHT_M))
        return sample


class FactoryVelocityClient:
    """An operator arms one short pulse; stopping also returns API ownership.

    A Move return value is a transport result, never an execution acknowledgement.
    Sport StopMove and release replies are preserved independently; either failure
    is returned to PulseController so the existing fault latch remains in force.
    """
    def __init__(self, sport, avoidance, avoidance_enabled, feedback, check_mode,
                 emit=lambda *args, **kwargs: None, clock=time.monotonic):
        self.sport, self.avoidance = sport, avoidance
        self.avoidance_enabled = avoidance_enabled
        self.feedback, self.check_mode = feedback, check_mode
        self.emit, self.clock = emit, clock
        self.armed = False
        self.api_owned = False
        self.arm_deadline = 0.
        self.subscriber = None  # Keep the read-only DDS subscriber alive.

    @property
    def backend(self):
        return 'obstacles_avoid' if self.avoidance_enabled else 'sport'

    def _result(self, operation, code, reply_expected=True):
        self.emit('sdk_result', backend=self.backend, operation=operation, code=code,
                  reply_expected=reply_expected, physical_motion_confirmed=False,
                  feedback=self.feedback.snapshot())
        return code

    def _require_zero(self):
        code = self._result('MoveZero', self.avoidance.Move(0., 0., 0.), False)
        if code != 0:
            raise RuntimeError('避障零速度发送失败：%r' % code)

    def arm(self):
        if self.armed or self.api_owned:
            raise RuntimeError('已有接管，先停止并交还')
        self.feedback.require_test_height()
        code, mode = self.check_mode()
        if code != 0 or not isinstance(mode, dict) or mode.get('name') != 'mcf':
            raise RuntimeError('原厂 mcf 控制模式未确认')
        code, enabled = self.avoidance.SwitchGet()
        if code != 0 or type(enabled) is not bool or enabled != self.avoidance_enabled:
            raise RuntimeError('原厂避障状态已改变或查询失败，请重新打开面板')
        if self.avoidance_enabled:
            # Do not acquire a channel that might still retain an old nonzero request.
            self._require_zero()
            self.api_owned = True  # A failed/uncertain RPC still requires a release attempt.
            try:
                code = self._result('UseRemoteCommandFromApi(true)', self.avoidance.UseRemoteCommandFromApi(True))
                if code != 0:
                    raise RuntimeError('API 接管未确认：%r' % code)
                self._require_zero()
                self.feedback.require_test_height()
            except Exception:
                try:
                    result = self.avoidance.UseRemoteCommandFromApi(False)
                    self._result('UseRemoteCommandFromApi(false)', result)
                    if result == 0:
                        self.api_owned = False
                except Exception:
                    pass
                raise
        else:
            self.feedback.require_test_height()
        self.arm_deadline = self.clock()+5.
        self.armed = True
        self.emit('operator_armed', backend=self.backend, expires_s=5., physical_readiness='operator_confirmed')

    def needs_release(self):
        return self.armed and self.clock() >= self.arm_deadline

    def Move(self, vx, vy, vyaw):
        values = (float(vx), float(vy), float(vyaw))
        if not all(math.isfinite(x) for x in values):
            raise ValueError('速度必须为有限值')
        if any(values):
            if not self.armed or self.needs_release():
                raise RuntimeError('请先确认站稳并接管本次短动作')
            self.feedback.require_test_height()
        client = self.avoidance if self.avoidance_enabled else self.sport
        return self._result('Move', client.Move(*values), False)

    def StopMove(self):
        self.armed = False
        code = 0
        try:
            code = self._result('StopMove', self.sport.StopMove())
        finally:
            if self.api_owned:
                result = self._result('UseRemoteCommandFromApi(false)', self.avoidance.UseRemoteCommandFromApi(False))
                if result == 0:
                    self.api_owned = False
                if code == 0:
                    code = result
        return code
