"""Aggregate system-status data sources into a single thread-safe snapshot."""
from __future__ import annotations

import time
from copy import copy
from dataclasses import dataclass, field
from threading import Lock
from typing import Any, Dict, List, Optional

import aimdk_msgs.msg as aimdk_msg
import aimdk_msgs.srv as aimdk_srv
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

try:
    from aimdk_msgs.msg import DiagnosticInfoArray as _HdsDiagArray
    from aimdk_msgs.msg import AlertCodeArray as _HdsAlertArray
    from aimdk_msgs.msg import PostProcSigArray as _HdsPostProcArray
    HDS_AVAILABLE = True
except ImportError:
    _HdsDiagArray = None
    _HdsAlertArray = None
    _HdsPostProcArray = None
    HDS_AVAILABLE = False

try:
    from aimdk_msgs.msg import ProcessedAudioOutput as _AgentAudio
    AGENT_LIVENESS_AVAILABLE = True
except ImportError:
    _AgentAudio = None
    AGENT_LIVENESS_AVAILABLE = False

try:
    from aimdk_msgs.srv import GetAgentPropertiesRequest as _GetAgentProps
    AGENT_GET_PROPS_AVAILABLE = True
except ImportError:
    _GetAgentProps = None
    AGENT_GET_PROPS_AVAILABLE = False


_SYSTEM_STATUS_NAMES = {
    0: "IN_INITIAL",
    1: "IN_READY",
    2: "IN_MOVE",
    3: "IN_ROLLBACK",
    4: "IN_FALLBACK",
    5: "IN_FALLBACK_MOVE",
}

_MC_ACTION_NAMES = {
    0: "UNSET",
    1: "PASSIVE_DEFAULT",
    2: "SOFT_EMERGENCY_STOP",
    3: "DAMPING_DEFAULT",
    4: "ZERO_TORQUE_DEFAULT",
    100: "JOINT_DEFAULT",
    101: "JOINT_FREEZE",
    200: "STAND_DEFAULT",
    201: "STAND_BODY_CONTROL",
    300: "LOCOMOTION_DEFAULT",
    301: "RUN_DEFAULT",
    302: "LOCOMOTION_STEP",
    400: "VR_REMOTE_CONTROLLER",
    2000: "SIT_DOWN_DEFAULT",
    2002: "CROUCH_DOWN_DEFAULT",
    2004: "LIE_DOWN_DEFAULT",
    2005: "STAND_UP_DEFAULT",
    2006: "ASCEND_STAIRS",
    2008: "DESCEND_STAIRS",
}

_MC_STATUS_NAMES = {
    0: "IDLE",
    100: "RUNNING",
    200: "TRANSITION",
}


@dataclass
class StatusSnapshot:
    mc_action_value: Optional[int] = None
    mc_action_name: Optional[str] = None
    mc_action_desc: Optional[str] = None
    mc_status_value: Optional[int] = None
    mc_status_name: Optional[str] = None

    system_state: Optional[str] = None
    system_status_value: Optional[int] = None
    system_status_name: Optional[str] = None

    pmu_temperature: Optional[float] = None
    orin_voltage: Optional[float] = None
    orin_current: Optional[float] = None
    rk3588_voltage: Optional[float] = None
    rk3588_current: Optional[float] = None
    bus_48v_voltage: Optional[float] = None
    bus_48v_current: Optional[float] = None
    fan_speed: Optional[float] = None
    fan_pct: Optional[int] = None
    battery_pct: Optional[int] = None
    battery_voltage: Optional[float] = None
    battery_current: Optional[float] = None
    battery_temperature: Optional[float] = None
    battery_power: Optional[float] = None
    pmu_bool_status: Optional[int] = None
    bms_status_bits: Optional[int] = None

    orin_cpu_pct_avg: Optional[float] = None
    orin_cpu_cores: Optional[int] = None
    orin_gpu_pct: Optional[float] = None
    orin_mem_used_mb: Optional[float] = None
    orin_mem_total_mb: Optional[float] = None
    orin_temp_cpu: Optional[float] = None
    orin_temp_gpu: Optional[float] = None
    orin_provider: Optional[str] = None

    hds_available: bool = HDS_AVAILABLE
    diag_list: List[Dict[str, Any]] = field(default_factory=list)
    alert_list: List[Dict[str, Any]] = field(default_factory=list)
    postproc_list: List[Dict[str, Any]] = field(default_factory=list)

    agent_liveness_available: bool = AGENT_LIVENESS_AVAILABLE
    agent_get_props_available: bool = AGENT_GET_PROPS_AVAILABLE
    agent_run_mode: Optional[str] = None
    agent_language: Optional[str] = None

    last_pmu_at: float = 0.0
    last_orin_at: float = 0.0
    last_state_at: float = 0.0
    last_mc_at: float = 0.0
    last_diag_at: float = 0.0
    last_alert_at: float = 0.0
    last_postproc_at: float = 0.0
    last_agent_audio_at: float = 0.0
    last_agent_props_at: float = 0.0


class StatusCollector(Node):
    SERVICE_GET_STATE = "/aimdk_5Fmsgs/srv/GetSystemState"
    SERVICE_GET_MC_ACTION = "/aimdk_5Fmsgs/srv/GetMcAction"
    SERVICE_GET_HDS_EVENT_HISTORY = "/aimdk_5Fmsgs/srv/GetHdsEventHistory"
    SERVICE_GET_AGENT_PROPS = "/aimdk_5Fmsgs/srv/GetAgentPropertiesRequest"
    TOPIC_PMU = "/aima/hal/pmu/state"
    TOPIC_ORIN = "/orin/stats"
    TOPIC_HDS_DIAG = "/aima/hds/diag_code_list"
    TOPIC_HDS_ALERT = "/aima/hds/alert_code_list"
    TOPIC_HDS_POSTPROC = "/aima/hds/post_proc/signal"
    TOPIC_AGENT_AUDIO = "/agent/process_audio_output"
    POLL_PERIOD_S = 1.0
    CALL_TIMEOUT_S = 3.0  # cancel stuck async service calls after this many seconds

    def __init__(self) -> None:
        super().__init__("robotop_status_collector")
        self._snap = StatusSnapshot()
        self._lock = Lock()

        sensor_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(
            aimdk_msg.PmuState, self.TOPIC_PMU, self._on_pmu, sensor_qos)
        self.create_subscription(
            DiagnosticArray, self.TOPIC_ORIN, self._on_orin, 10)

        if HDS_AVAILABLE:
            self.create_subscription(
                _HdsDiagArray, self.TOPIC_HDS_DIAG, self._on_hds_diag, 10)
            self.create_subscription(
                _HdsAlertArray, self.TOPIC_HDS_ALERT, self._on_hds_alert, 10)
            self.create_subscription(
                _HdsPostProcArray, self.TOPIC_HDS_POSTPROC, self._on_hds_postproc, 10)

        if AGENT_LIVENESS_AVAILABLE:
            self.create_subscription(
                _AgentAudio, self.TOPIC_AGENT_AUDIO, self._on_agent_audio, sensor_qos
            )

        self._get_state_client = self.create_client(
            aimdk_srv.GetSystemState, self.SERVICE_GET_STATE
        )
        self._get_mc_action_client = self.create_client(
            aimdk_srv.GetMcAction, self.SERVICE_GET_MC_ACTION
        )
        self._pending_state_future = None
        self._state_call_at = 0.0
        self._pending_mc_future = None
        self._mc_call_at = 0.0
        self.create_timer(self.POLL_PERIOD_S, self._poll_state)
        self.create_timer(self.POLL_PERIOD_S, self._poll_mc_action)

        if AGENT_GET_PROPS_AVAILABLE:
            self._get_agent_client = self.create_client(
                _GetAgentProps, self.SERVICE_GET_AGENT_PROPS
            )
            self._pending_agent_future = None
            self._agent_call_at = 0.0
            self.create_timer(self.POLL_PERIOD_S, self._poll_agent_props)
        else:
            self._get_agent_client = None
            self._pending_agent_future = None
            self._agent_call_at = 0.0

    def snapshot(self) -> StatusSnapshot:
        with self._lock:
            return copy(self._snap)

    def _on_pmu(self, msg) -> None:
        with self._lock:
            self._snap.pmu_temperature = float(msg.pmu_temperature)
            self._snap.orin_voltage = float(msg.orin_voltage)
            self._snap.orin_current = float(msg.orin_current)
            self._snap.rk3588_voltage = float(msg.rk3588_voltage)
            self._snap.rk3588_current = float(msg.rk3588_current)
            self._snap.bus_48v_voltage = float(msg.bus_48v_voltage)
            self._snap.bus_48v_current = float(msg.bus_48v_current)
            self._snap.fan_speed = float(msg.fan_speed)
            self._snap.fan_pct = int(msg.fan_pecentage)
            self._snap.battery_pct = int(
                msg.battery_remaining_capacity_percentage)
            self._snap.battery_voltage = float(msg.battery_pack_voltage)
            self._snap.battery_current = float(msg.battery_current)
            self._snap.battery_temperature = float(msg.battery_temperature)
            self._snap.battery_power = float(msg.battery_output_power)
            self._snap.pmu_bool_status = int(msg.pmu_bool_status)
            self._snap.bms_status_bits = int(msg.bms_status_bits)
            self._snap.last_pmu_at = time.time()

    def _on_orin(self, msg) -> None:
        if not msg.status:
            return
        st = msg.status[0]
        kvs = {kv.key: kv.value for kv in st.values}
        with self._lock:
            self._snap.orin_provider = st.message
            self._snap.orin_cpu_pct_avg = _ffloat(kvs.get("cpu_pct_avg"))
            cores = kvs.get("cpu_cores")
            self._snap.orin_cpu_cores = int(
                cores) if cores and cores.isdigit() else None
            self._snap.orin_gpu_pct = _ffloat(kvs.get("gpu_pct"))
            self._snap.orin_mem_used_mb = _ffloat(kvs.get("mem_used_mb"))
            self._snap.orin_mem_total_mb = _ffloat(kvs.get("mem_total_mb"))
            self._snap.orin_temp_cpu = _ffloat(kvs.get("temp_cpu"))
            self._snap.orin_temp_gpu = _ffloat(kvs.get("temp_gpu"))
            self._snap.last_orin_at = time.time()

    def _poll_state(self) -> None:
        now = time.time()
        if self._pending_state_future is not None and not self._pending_state_future.done():
            if now - self._state_call_at < self.CALL_TIMEOUT_S:
                return
            try:
                self._pending_state_future.cancel()
            except Exception:
                pass
            self._pending_state_future = None
        if not self._get_state_client.service_is_ready():
            return
        req = aimdk_srv.GetSystemState.Request()
        clk = self.get_clock().now()
        req.header.header.stamp.sec = clk.nanoseconds // 1_000_000_000
        req.header.header.stamp.nanosec = clk.nanoseconds % 1_000_000_000
        fut = self._get_state_client.call_async(req)
        fut.add_done_callback(self._on_state)
        self._pending_state_future = fut
        self._state_call_at = now

    def _on_state(self, future) -> None:
        try:
            resp = future.result()
        except Exception:
            return
        if resp is None:
            return
        with self._lock:
            self._snap.system_state = resp.cur_state
            self._snap.system_status_value = int(resp.curr_status.value)
            self._snap.system_status_name = _SYSTEM_STATUS_NAMES.get(
                int(
                    resp.curr_status.value), f"unknown({resp.curr_status.value})"
            )
            self._snap.last_state_at = time.time()

    def _poll_mc_action(self) -> None:
        now = time.time()
        if self._pending_mc_future is not None and not self._pending_mc_future.done():
            if now - self._mc_call_at < self.CALL_TIMEOUT_S:
                return
            try:
                self._pending_mc_future.cancel()
            except Exception:
                pass
            self._pending_mc_future = None
        if not self._get_mc_action_client.service_is_ready():
            return
        req = aimdk_srv.GetMcAction.Request()
        clk = self.get_clock().now()
        req.request.header.stamp.sec = clk.nanoseconds // 1_000_000_000
        req.request.header.stamp.nanosec = clk.nanoseconds % 1_000_000_000
        fut = self._get_mc_action_client.call_async(req)
        fut.add_done_callback(self._on_mc_action)
        self._pending_mc_future = fut
        self._mc_call_at = now

    def _on_mc_action(self, future) -> None:
        try:
            resp = future.result()
        except Exception:
            return
        info = resp.info
        action_val = int(info.current_action.value)
        status_val = int(info.status.value)
        with self._lock:
            self._snap.mc_action_value = action_val
            self._snap.mc_action_name = _MC_ACTION_NAMES.get(
                action_val, f"unknown({action_val})")
            self._snap.mc_action_desc = info.action_desc
            self._snap.mc_status_value = status_val
            self._snap.mc_status_name = _MC_STATUS_NAMES.get(
                status_val, f"unknown({status_val})")
            self._snap.last_mc_at = time.time()

    def _on_hds_diag(self, msg) -> None:
        items = []
        for d in msg.diagnostics:
            items.append({
                "timestamp": int(d.timestamp),
                "first_timestamp": int(d.first_timestamp),
                "type": int(d.type.value),
                "module_id": int(d.module_id),
                "code": int(d.code),
                "diag_code": int(d.diag_code),
                "info": d.info,
            })
        with self._lock:
            self._snap.diag_list = items
            self._snap.last_diag_at = time.time()

    def _on_hds_alert(self, msg) -> None:
        items = []
        for a in msg.list:
            items.append({
                "timestamp": int(a.timestamp),
                "first_timestamp": int(a.first_timestamp),
                "code": a.code,
            })
        with self._lock:
            self._snap.alert_list = items
            self._snap.last_alert_at = time.time()

    def _on_hds_postproc(self, msg) -> None:
        items = []
        for p in msg.post_proc_sig:
            s = p.sig_set
            items.append({
                "timestamp": int(p.timestamp),
                "diag_code": int(p.diag_code),
                "robot_sig": int(s.robot_sig),
                "motion_sig": int(s.motion_sig),
                "sound_sig": int(s.sound_sig),
                "led_sig": int(s.led_sig),
                "facial_sig": int(s.facial_sig),
                "funcdisable_sig": int(s.funcdisable_sig),
            })
        with self._lock:
            self._snap.postproc_list = items
            self._snap.last_postproc_at = time.time()

    def _on_agent_audio(self, _msg) -> None:
        with self._lock:
            self._snap.last_agent_audio_at = time.time()

    def _poll_agent_props(self) -> None:
        if self._get_agent_client is None:
            return
        now = time.time()
        if self._pending_agent_future is not None and not self._pending_agent_future.done():
            if now - self._agent_call_at < self.CALL_TIMEOUT_S:
                return
            try:
                self._pending_agent_future.cancel()
            except Exception:
                pass
            self._pending_agent_future = None
        if not self._get_agent_client.service_is_ready():
            return
        req = _GetAgentProps.Request()
        clk = self.get_clock().now()
        req.header.header.stamp.sec = clk.nanoseconds // 1_000_000_000
        req.header.header.stamp.nanosec = clk.nanoseconds % 1_000_000_000
        req.property_ids = [
            aimdk_msg.AgentPropertyIdType(value=0x01),  # LANGUAGE
            aimdk_msg.AgentPropertyIdType(value=0x02),  # RUN_MODE
        ]
        fut = self._get_agent_client.call_async(req)
        fut.add_done_callback(self._on_agent_props)
        self._pending_agent_future = fut
        self._agent_call_at = now

    def _on_agent_props(self, future) -> None:
        try:
            resp = future.result()
        except Exception:
            return
        run_mode = None
        language = None
        for prop in getattr(resp.contents, "properties", []):
            kid = int(prop.key.value)
            if kid == 0x01:
                language = prop.value
            elif kid == 0x02:
                run_mode = prop.value
        with self._lock:
            if run_mode is not None:
                self._snap.agent_run_mode = run_mode
            if language is not None:
                self._snap.agent_language = language
            self._snap.last_agent_props_at = time.time()


def _ffloat(v: Optional[str]) -> Optional[float]:
    if v is None:
        return None
    try:
        return float(v)
    except (TypeError, ValueError):
        return None
