#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""陆空协同无人机 UDP V1.2/V1.1 实飞安全网关。

统一绑定 UAV:8888，处理：
- GS -> UAV: CMD:PING/STATUS/BOOT/LAND/RESET
- CAR -> UAV: CMD:START、TEL:CAR、EVT:CAR_POINT
- UAV -> GS: ACK/ERR/HB/TEL:UAV/EVT

外部协议统一使用场地坐标（左下角为原点，X 向右、Y 向上，cm）。
ROS 内部 /car/state 使用相对 H 点的 MAVROS local 坐标（m）。
"""

from collections import OrderedDict
import json
import queue
import re
import math
import os
import socket
import threading
import time

import rospy
import yaml
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, String


VALID_SEGMENTS = {"AB", "BC", "CD", "DA", "UNKNOWN"}
CMD_ID_RE = re.compile(r"^[A-Za-z0-9_-]{1,24}$")
RUN_ID_RE = re.compile(r"^[A-Za-z0-9_-]{1,24}$")


def clamp(value, low, high):
    return max(low, min(high, value))


def safe_float(value, default=0.0):
    try:
        result = float(value)
        return result if math.isfinite(result) else float(default)
    except Exception:
        return float(default)


def safe_int(value, default=0):
    try:
        return int(value)
    except Exception:
        return int(default)


def normalize_task(value):
    text = str(value).strip().upper()
    if text in {"2", "T2", "TASK2", "DYNAMIC_LAND"}:
        return "T2"
    if text in {"1", "T1", "TASK1", "DROP"}:
        return "T1"
    return ""


def task_to_mission_type(task):
    return "dynamic_land" if normalize_task(task) == "T2" else "drop"


def run_id_to_int(run_id):
    digits = "".join(ch for ch in str(run_id) if ch.isdigit())
    if digits:
        return safe_int(digits, 0)
    return 0


class TrackProgressModel:
    """只根据累计里程计算区段、区段进度和整圈进度。"""

    def __init__(self, cfg):
        self.straight = safe_float(cfg.get("straight_length_m", 1.50), 1.50)
        self.radius = safe_float(cfg.get("radius_m", 0.75), 0.75)
        self.total = 2.0 * self.straight + 2.0 * math.pi * self.radius
        self.boundary_tolerance = safe_float(
            cfg.get("segment_boundary_tolerance_m", 0.08), 0.08
        )

    def calculate(self, path_s_m):
        if self.total <= 1e-6:
            return "UNKNOWN", -1.0, -1.0
        s = clamp(float(path_s_m), 0.0, self.total)
        b = self.straight
        c = b + math.pi * self.radius
        d = c + self.straight
        if s < b:
            return "AB", s / self.straight, s / self.total
        if s < c:
            return "BC", (s - b) / (math.pi * self.radius), s / self.total
        if s < d:
            return "CD", (s - c) / self.straight, s / self.total
        return "DA", (s - d) / (math.pi * self.radius), s / self.total

    def near_boundary(self, path_s_m):
        boundaries = [
            self.straight,
            self.straight + math.pi * self.radius,
            2.0 * self.straight + math.pi * self.radius,
            self.total,
        ]
        return any(abs(float(path_s_m) - value) <= self.boundary_tolerance for value in boundaries)


class FieldTransform:
    """FIELD 坐标与以 H 点为原点的 MAVROS local 相对坐标互换。"""

    def __init__(self, cfg):
        self.h_x_m = safe_float(cfg.get("h_field_x_cm", 75.0), 75.0) / 100.0
        self.h_y_m = safe_float(cfg.get("h_field_y_cm", 75.0), 75.0) / 100.0
        self.theta = math.radians(
            safe_float(cfg.get("local_x_to_field_yaw_deg", 0.0), 0.0)
        )

    def field_to_local_xy(self, x_field_m, y_field_m):
        dx = float(x_field_m) - self.h_x_m
        dy = float(y_field_m) - self.h_y_m
        c = math.cos(self.theta)
        s = math.sin(self.theta)
        return c * dx + s * dy, -s * dx + c * dy

    def local_to_field_xy(self, x_local_m, y_local_m):
        c = math.cos(self.theta)
        s = math.sin(self.theta)
        return (
            self.h_x_m + c * float(x_local_m) - s * float(y_local_m),
            self.h_y_m + s * float(x_local_m) + c * float(y_local_m),
        )

    def field_to_local_vector(self, vx_field, vy_field):
        c = math.cos(self.theta)
        s = math.sin(self.theta)
        return c * vx_field + s * vy_field, -s * vx_field + c * vy_field

    def local_to_field_vector(self, vx_local, vy_local):
        c = math.cos(self.theta)
        s = math.sin(self.theta)
        return c * vx_local - s * vy_local, s * vx_local + c * vy_local

    def field_to_local_yaw(self, yaw_field_rad):
        return math.atan2(
            math.sin(float(yaw_field_rad) - self.theta),
            math.cos(float(yaw_field_rad) - self.theta),
        )

    def local_to_field_yaw_deg(self, yaw_local_rad):
        value = math.degrees(float(yaw_local_rad) + self.theta) % 360.0
        return value if value >= 0.0 else value + 360.0


class UavUdpGateway:
    def __init__(self):
        rospy.init_node("uav_udp_gateway", anonymous=False)
        default_yaml = os.path.expanduser(
            "~/catkin_ws/src/d26_air_ground_uav/config/air_ground_mission.yaml"
        )
        config_path = rospy.get_param("~mission_yaml", default_yaml)
        with open(config_path, "r", encoding="utf-8") as stream:
            cfg = yaml.safe_load(stream) or {}

        pcfg = cfg.get("udp_protocol", {})
        ucfg = pcfg.get("uav", {})
        gcfg = pcfg.get("ground_station", {})
        ccfg = pcfg.get("car", {})
        rcfg = pcfg.get("reliability", {})
        tcfg = pcfg.get("telemetry", {})

        self.proto = str(pcfg.get("version", "1.1"))
        self.listen_ip = str(
            rospy.get_param("~listen_ip", ucfg.get("listen_ip", "0.0.0.0"))
        )
        self.listen_port = int(
            rospy.get_param("~listen_port", int(ucfg.get("listen_port", 8888)))
        )
        self.gs_addr = (
            str(
                rospy.get_param(
                    "~ground_station_ip", gcfg.get("ip", "192.168.151.101")
                )
            ),
            int(
                rospy.get_param(
                    "~ground_station_port", int(gcfg.get("port", 8889))
                )
            ),
        )
        self.car_ip = str(
            rospy.get_param("~car_ip", ccfg.get("ip", "192.168.151.103"))
        )
        self.strict_source_ip = bool(
            rospy.get_param(
                "~strict_source_ip", bool(pcfg.get("strict_source_ip", False))
            )
        )
        # DHCP 场景默认不要求预先配置地面站 IP。关闭严格来源校验后，
        # 第一个合法的地面站控制命令会更新后续 HB/TEL/EVT 的目标 IP。
        self.learn_ground_station_ip = bool(
            rospy.get_param("~learn_ground_station_ip", not self.strict_source_ip)
        )
        self.gs_addr_learned = bool(self.strict_source_ip)
        self.gs_addr_lock = threading.Lock()

        self.allow_gs_direct_start = bool(
            rospy.get_param(
                "~allow_gs_direct_start",
                bool(pcfg.get("allow_gs_direct_start", False)),
            )
        )
        self.car_telemetry_enabled = bool(
            rospy.get_param("~car_telemetry_enabled", True)
        )
        self.forced_task = normalize_task(rospy.get_param("~forced_task", ""))
        self.allow_legacy = bool(
            rospy.get_param(
                "~allow_legacy_format", bool(pcfg.get("allow_legacy_format", True))
            )
        )
        self.command_result_timeout = max(
            0.20,
            safe_float(
                rospy.get_param(
                    "~command_result_timeout_s",
                    rcfg.get("command_result_timeout_s", 0.80),
                ),
                0.80,
            ),
        )
        self.status_max_age_s = max(
            0.20, safe_float(rospy.get_param("~status_max_age_s", 0.80), 0.80)
        )
        self.start_pose_max_age_s = max(
            0.10, safe_float(rospy.get_param("~start_pose_max_age_s", 0.30), 0.30)
        )
        self.max_packet_bytes = max(
            256, min(65507, safe_int(rospy.get_param("~max_packet_bytes", 4096), 4096))
        )
        self.dedup_limit = max(8, safe_int(rcfg.get("dedup_cache_size", 32), 32))
        self.event_repeat = max(1, safe_int(rcfg.get("event_repeat", 3), 3))
        self.event_repeat_interval = safe_float(
            rcfg.get("event_repeat_interval_s", 0.05), 0.05
        )
        self.hb_rate_hz = max(0.2, safe_float(tcfg.get("heartbeat_rate_hz", 1.0), 1.0))
        self.tel_rate_hz = max(1.0, safe_float(tcfg.get("uav_rate_hz", 10.0), 10.0))
        self.require_segment_consistency = bool(
            ccfg.get("require_segment_consistency", True)
        )
        self.max_delay_compensation_s = safe_float(
            ccfg.get("max_delay_compensation_s", 0.30), 0.30
        )

        self.transform = FieldTransform(pcfg.get("field_transform", {}))
        self.track = TrackProgressModel(pcfg.get("track", {}))

        self.car_pub = None
        if self.car_telemetry_enabled:
            self.car_pub = rospy.Publisher("/car/state", String, queue_size=50)
        self.car_event_pub = rospy.Publisher("/car/event", String, queue_size=20)
        self.command_pub = rospy.Publisher("/uav/mission_command", String, queue_size=10)
        self.mission_type_pub = rospy.Publisher("/uav/mission_type", String, queue_size=10)
        self.land_pub = rospy.Publisher("/uav/land", Bool, queue_size=10)
        self.reset_pub = rospy.Publisher("/uav/reset", Bool, queue_size=10)

        # 状态字段必须在创建订阅者前初始化，避免回调抢先触发。
        self.latest_status = {}
        self.latest_status_rx = 0.0
        self.latest_fcu_state = State()
        self.latest_fcu_state_rx = 0.0
        self.latest_pose_rx = 0.0
        self.battery_percent = -1
        self.boot_task = ""
        self.boot_state = "STOPPED"
        self.boot_ready_sent = False
        self.run_id = "R000"
        self.run_start_monotonic = None
        self.hb_seq = 0
        self.tel_seq = 0
        self.last_hb = 0.0
        self.last_tel = 0.0
        self.dedup = OrderedDict()
        self.result_condition = threading.Condition()
        self.command_results = {}
        self.event_queue = queue.Queue(maxsize=128)
        self.event_stop = threading.Event()

        rospy.Subscriber("/uav/mission_status", String, self.status_cb, queue_size=30)
        rospy.Subscriber("/uav/mission_event", String, self.event_cb, queue_size=30)
        rospy.Subscriber(
            "/uav/mission_command_result", String, self.command_result_cb, queue_size=20
        )
        rospy.Subscriber("/mavros/state", State, self.fcu_state_cb, queue_size=20)
        rospy.Subscriber(
            "/mavros/local_position/pose", PoseStamped, self.pose_cb, queue_size=30
        )
        rospy.Subscriber("/mavros/battery", BatteryState, self.battery_cb, queue_size=10)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 262144)
        self.sock.bind((self.listen_ip, self.listen_port))
        self.sock.settimeout(0.03)
        self.event_thread = threading.Thread(
            target=self.event_sender_loop,
            name="uav_udp_event_sender",
            daemon=True,
        )
        self.event_thread.start()
        rospy.on_shutdown(self.shutdown)

        gs_mode = (
            "FIXED {}:{}".format(self.gs_addr[0], self.gs_addr[1])
            if self.strict_source_ip
            else "DYNAMIC (learn from first valid GS command, reply port {})".format(
                self.gs_addr[1]
            )
        )
        rospy.logwarn(
            "UDP V%s gateway listening %s:%d, GS=%s, CAR=%s, "
            "STRICT_SOURCE_IP=%s, GS_DIRECT_START=%s, CAR_TELEMETRY=%s, "
            "FORCED_TASK=%s, POSE_MAX_AGE=%.2fs, STATUS_MAX_AGE=%.2fs",
            self.proto,
            self.listen_ip,
            self.listen_port,
            gs_mode,
            self.car_ip,
            self.strict_source_ip,
            self.allow_gs_direct_start,
            self.car_telemetry_enabled,
            self.forced_task or "ANY",
            self.start_pose_max_age_s,
            self.status_max_age_s,
        )

    def shutdown(self):
        self.event_stop.set()
        try:
            self.sock.close()
        except Exception:
            pass

    def fcu_state_cb(self, msg):
        self.latest_fcu_state = msg
        self.latest_fcu_state_rx = time.monotonic()

    def pose_cb(self, _msg):
        self.latest_pose_rx = time.monotonic()

    def status_cb(self, msg):
        try:
            self.latest_status = json.loads(msg.data)
            self.latest_status_rx = time.monotonic()
        except Exception as exc:
            rospy.logwarn_throttle(1.0, "Invalid mission status JSON: %s", str(exc))

    def battery_cb(self, msg):
        value = safe_float(msg.percentage, -1.0)
        if 0.0 <= value <= 1.0:
            value *= 100.0
        if 0.0 <= value <= 100.0:
            self.battery_percent = int(round(value))

    def command_result_cb(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        cmd_id = str(data.get("cmd_id", ""))
        if not cmd_id:
            return
        with self.result_condition:
            self.command_results[cmd_id] = data
            self.result_condition.notify_all()

    def event_cb(self, msg):
        try:
            data = json.loads(msg.data)
            event = str(data.get("event", "")).strip().upper()
            run_id = str(data.get("run_id", self.run_id))
            task = normalize_task(data.get("task", ""))
        except Exception:
            event = str(msg.data).strip().upper()
            run_id = self.run_id
            task = ""
        if not event:
            return
        if event == "MISSION_START":
            packet = "EVT:MISSION_START:{}:{}".format(run_id, task or self.boot_task or "T1")
        elif event == "UAV_BOOT_READY":
            packet = "EVT:UAV_BOOT_READY:{}".format(task or self.boot_task or "T1")
        else:
            packet = "EVT:{}:{}".format(event, run_id)
        self.send_event(packet)

    def send_event(self, packet):
        """关键事件进入独立发送队列，绝不阻塞 LAND 等命令接收。"""
        try:
            self.event_queue.put_nowait(str(packet))
        except queue.Full:
            rospy.logerr_throttle(1.0, "UDP event queue full; drop event: %s", packet)

    def event_sender_loop(self):
        while not self.event_stop.is_set() and not rospy.is_shutdown():
            try:
                packet = self.event_queue.get(timeout=0.10)
            except queue.Empty:
                continue
            encoded = packet.encode("utf-8")
            for index in range(self.event_repeat):
                if self.event_stop.is_set() or rospy.is_shutdown():
                    break
                target = self.get_ground_station_addr()
                if target is None:
                    rospy.logwarn_throttle(
                        2.0,
                        "No ground station learned yet; event postponed/dropped: %s",
                        packet,
                    )
                    break
                try:
                    self.sock.sendto(encoded, target)
                except OSError as exc:
                    if not rospy.is_shutdown():
                        rospy.logwarn_throttle(1.0, "UDP event send failed: %s", str(exc))
                    break
                if index + 1 < self.event_repeat:
                    self.event_stop.wait(self.event_repeat_interval)
            self.event_queue.task_done()

    def publish_land_burst(self):
        """异步重复发布 LAND，降低 ROS 单帧在节点重连瞬间丢失的概率。"""
        for index in range(3):
            if rospy.is_shutdown():
                return
            self.land_pub.publish(Bool(data=True))
            if index < 2:
                time.sleep(0.05)

    def get_ground_station_addr(self):
        """返回当前地面站遥测目标；动态模式尚未学习时返回 None。"""
        with self.gs_addr_lock:
            if not self.gs_addr_learned and not self.strict_source_ip:
                return None
            return self.gs_addr

    def learn_ground_station(self, addr, action):
        """从合法 GS 控制命令动态学习 IP，端口仍使用协议约定的 8889。"""
        if self.strict_source_ip or not self.learn_ground_station_ip:
            return
        if action not in {"PING", "STATUS", "BOOT", "LAND", "RESET", "START"}:
            return
        learned = (str(addr[0]), int(self.gs_addr[1]))
        with self.gs_addr_lock:
            changed = (not self.gs_addr_learned) or learned != self.gs_addr
            old = self.gs_addr
            self.gs_addr = learned
            self.gs_addr_learned = True
        if changed:
            rospy.logwarn(
                "Ground station IP learned/updated: %s:%d -> %s:%d (action=%s)",
                old[0], old[1], learned[0], learned[1], action,
            )

    def source_role(self, addr):
        ip = str(addr[0])
        if ip == self.car_ip:
            return "CAR"
        if ip in {"127.0.0.1", "localhost"}:
            return "DEBUG"
        if self.strict_source_ip:
            return "GS" if ip == self.gs_addr[0] else "UNKNOWN"
        # DHCP 模式：任何非小车来源的合法控制报文均按 GS 处理；
        # 命令格式、任务状态、FCU/定位新鲜度仍会继续校验。
        return "GS"

    def send_text(self, text, addr):
        try:
            self.sock.sendto(text.encode("utf-8"), addr)
        except OSError as exc:
            rospy.logwarn_throttle(1.0, "UDP send failed: %s", str(exc))

    def cache_reply(self, key, reply):
        self.dedup[key] = reply
        self.dedup.move_to_end(key)
        while len(self.dedup) > self.dedup_limit:
            self.dedup.popitem(last=False)

    def ack(self, cmd_id, action, result, detail=""):
        parts = ["ACK", str(cmd_id), str(action), str(result)]
        if detail:
            parts.append(str(detail))
        return ":".join(parts)

    def err(self, cmd_id, action, code, detail=""):
        parts = ["ERR", str(cmd_id), str(action), str(code)]
        if detail:
            parts.append(str(detail))
        return ":".join(parts)

    def parse_command(self, text):
        parts = text.strip().split(":")
        if len(parts) >= 3 and parts[0].upper() == "CMD":
            cmd_id = parts[1].strip()
            action = parts[2].strip().upper()
            if not CMD_ID_RE.fullmatch(cmd_id) or not action:
                return None
            return cmd_id, action, parts[3:], False
        if self.allow_legacy and len(parts) == 2 and parts[0].upper() == "CMD":
            legacy_id = "L{:06d}".format(int(time.monotonic() * 1000) % 1000000)
            return legacy_id, parts[1].upper(), [], True
        return None

    def wait_command_result(self, cmd_id):
        deadline = time.monotonic() + self.command_result_timeout
        with self.result_condition:
            while cmd_id not in self.command_results:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    break
                self.result_condition.wait(remaining)
            return self.command_results.pop(cmd_id, None)

    def status_fresh(self):
        return bool(self.latest_status) and (
            time.monotonic() - self.latest_status_rx <= self.status_max_age_s
        )

    def pose_fresh(self):
        return self.latest_pose_rx > 0.0 and (
            time.monotonic() - self.latest_pose_rx <= self.start_pose_max_age_s
        )

    def fcu_connected(self):
        return bool(self.latest_fcu_state.connected) and (
            time.monotonic() - self.latest_fcu_state_rx <= self.status_max_age_s
        )

    def fsm_state(self):
        return str(self.latest_status.get("fsm_state", ""))

    def mission_already_active(self, run_id, task):
        if not self.status_fresh():
            return False
        state = self.fsm_state()
        if state in {"", "WAIT_START", "WAIT_RESET"}:
            return False
        status_run = str(self.latest_status.get("run_id", ""))
        mission_type = str(self.latest_status.get("mission_type", ""))
        status_task = "T2" if mission_type == "dynamic_land" else "T1"
        return status_run == str(run_id) and status_task == normalize_task(task)

    def boot_ready(self):
        status = self.latest_status
        mavros = status.get("mavros", {})
        home = status.get("home", {})
        state = str(status.get("fsm_state", ""))
        return (
            self.status_fresh()
            and self.pose_fresh()
            and self.fcu_connected()
            and not bool(self.latest_fcu_state.armed)
            and bool(mavros.get("connected", False))
            and home.get("x") is not None
            and home.get("y") is not None
            and home.get("z") is not None
            and state == "WAIT_START"
        )

    def check_boot_transition(self):
        if self.boot_state != "STARTING" or not self.boot_task:
            return
        if self.boot_ready():
            self.boot_state = "READY"
            if not self.boot_ready_sent:
                self.boot_ready_sent = True
                self.send_event("EVT:UAV_BOOT_READY:{}".format(self.boot_task))

    def handle_command(self, text, addr):
        parsed = self.parse_command(text)
        if parsed is None:
            self.send_text("ERR:0000:UNKNOWN:BAD_FORMAT", addr)
            return
        cmd_id, action, args, legacy = parsed
        role = self.source_role(addr)
        if role in {"GS", "DEBUG"}:
            self.learn_ground_station(addr, action)
        key = (addr[0], cmd_id, action)
        if key in self.dedup:
            self.send_text(self.dedup[key], addr)
            return

        cacheable = True
        if self.strict_source_ip and role == "UNKNOWN":
            reply = self.err(cmd_id, action, "UNKNOWN_CMD", "SOURCE_NOT_ALLOWED")
        elif action == "PING":
            reply = self.ack(cmd_id, action, "OK", "UAV")
        elif action == "STATUS":
            reply = self.ack(cmd_id, action, "OK", "UAV")
            self.send_uav_telemetry(target=addr, force=True)
        elif action == "BOOT":
            if role not in {"GS", "DEBUG"}:
                reply = self.err(cmd_id, action, "UNKNOWN_CMD", "SOURCE_NOT_ALLOWED")
            else:
                task = normalize_task(args[0] if args else "")
                if not task:
                    reply = self.err(cmd_id, action, "BAD_TASK")
                elif self.forced_task and task != self.forced_task:
                    reply = self.err(
                        cmd_id, action, "MODE_MISMATCH", "FORCED_{}".format(self.forced_task)
                    )
                elif self.latest_fcu_state.armed:
                    reply = self.err(cmd_id, action, "BUSY", "ARMED")
                else:
                    self.boot_task = task
                    self.boot_state = "STARTING"
                    self.boot_ready_sent = False
                    self.mission_type_pub.publish(String(data=task_to_mission_type(task)))
                    reply = self.ack(cmd_id, action, "ACCEPTED", task)
        elif action == "START":
            allowed = role in {"CAR", "DEBUG"} or (
                role == "GS" and self.allow_gs_direct_start
            )
            if not allowed:
                reply = self.err(cmd_id, action, "UNKNOWN_CMD", "SOURCE_NOT_ALLOWED")
            else:
                run_id = str(args[0]).strip() if len(args) >= 1 else "R000"
                task = normalize_task(args[1] if len(args) >= 2 else self.boot_task)
                if not RUN_ID_RE.fullmatch(run_id):
                    reply = self.err(cmd_id, action, "BAD_FORMAT", "BAD_RUN_ID")
                elif not task:
                    reply = self.err(cmd_id, action, "BAD_TASK")
                elif self.forced_task and task != self.forced_task:
                    reply = self.err(
                        cmd_id, action, "MODE_MISMATCH", "FORCED_{}".format(self.forced_task)
                    )
                elif self.mission_already_active(run_id, task):
                    # ACK 丢失后的重发：任务已经按同一 run_id 启动，不再重复触发。
                    self.run_id = run_id
                    reply = self.ack(cmd_id, action, "OK", run_id)
                elif self.latest_fcu_state.armed:
                    reply = self.err(cmd_id, action, "ALREADY_RUNNING", self.fsm_state())
                elif not self.fcu_connected():
                    reply = self.err(cmd_id, action, "FCU_DISCONNECTED")
                elif not self.status_fresh():
                    reply = self.err(cmd_id, action, "NOT_READY", "STALE_FSM_STATUS")
                elif not self.pose_fresh():
                    reply = self.err(cmd_id, action, "NOT_READY", "STALE_LOCAL_POSITION")
                elif self.boot_state != "READY":
                    reply = self.err(cmd_id, action, "NOT_READY", "BOOT")
                elif self.boot_task and task != self.boot_task:
                    reply = self.err(cmd_id, action, "MODE_MISMATCH")
                elif self.fsm_state() != "WAIT_START":
                    reply = self.err(cmd_id, action, "NOT_READY", self.fsm_state() or "FSM")
                else:
                    command = {
                        "action": "START",
                        "cmd_id": cmd_id,
                        "run_id": run_id,
                        "task": task,
                        "source": role,
                        "legacy": legacy,
                    }
                    self.command_pub.publish(
                        String(data=json.dumps(command, separators=(",", ":")))
                    )
                    result = self.wait_command_result(cmd_id)
                    if result is None:
                        # 不缓存不确定结果。若 FSM 稍后已启动，重发会由
                        # mission_already_active() 返回同一 run_id 的 ACK。
                        cacheable = False
                        if self.mission_already_active(run_id, task):
                            self.run_id = run_id
                            self.run_start_monotonic = time.monotonic()
                            reply = self.ack(cmd_id, action, "OK", run_id)
                            cacheable = True
                        else:
                            reply = self.err(cmd_id, action, "NOT_READY", "FSM_NO_REPLY")
                    elif bool(result.get("ok", False)):
                        self.run_id = run_id
                        self.run_start_monotonic = time.monotonic()
                        reply = self.ack(cmd_id, action, "OK", run_id)
                    else:
                        reply = self.err(
                            cmd_id,
                            action,
                            str(result.get("error", "NOT_READY")),
                            str(result.get("detail", "")),
                        )
        elif action == "LAND":
            if role not in {"GS", "DEBUG"}:
                reply = self.err(cmd_id, action, "UNKNOWN_CMD", "SOURCE_NOT_ALLOWED")
            else:
                run_id = str(args[0]).strip() if args else self.run_id
                if not RUN_ID_RE.fullmatch(run_id):
                    reply = self.err(cmd_id, action, "BAD_FORMAT", "BAD_RUN_ID")
                else:
                    # 紧急命令先 ACK，再触发 ROS，避免事件重发拖慢确认。
                    reply = self.ack(cmd_id, action, "ACCEPTED", run_id)
                    self.cache_reply(key, reply)
                    self.send_text(reply, addr)
                    threading.Thread(
                        target=self.publish_land_burst,
                        name="uav_udp_land_burst",
                        daemon=True,
                    ).start()
                    self.send_event("EVT:UAV_ABORTING:{}".format(run_id))
                    rospy.logerr("UDP LAND accepted from %s for %s", addr[0], run_id)
                    return
        elif action == "RESET":
            if role not in {"GS", "DEBUG"}:
                reply = self.err(cmd_id, action, "UNKNOWN_CMD", "SOURCE_NOT_ALLOWED")
            elif self.latest_fcu_state.armed:
                reply = self.err(cmd_id, action, "BUSY", "ARMED")
            elif self.fsm_state() not in {"WAIT_START", "WAIT_RESET", "WAIT_FCU"}:
                reply = self.err(cmd_id, action, "BUSY", self.fsm_state() or "FSM")
            else:
                self.reset_pub.publish(Bool(data=True))
                self.run_id = "R000"
                self.run_start_monotonic = None
                self.boot_state = "STARTING" if self.boot_task else "STOPPED"
                self.boot_ready_sent = False
                reply = self.ack(cmd_id, action, "ACCEPTED")
        elif action == "STOP_NODES":
            reply = self.err(cmd_id, action, "UNKNOWN_CMD", "DISABLED_IN_FLIGHT_BUILD")
        else:
            reply = self.err(cmd_id, action, "UNKNOWN_CMD")

        if cacheable:
            self.cache_reply(key, reply)
        self.send_text(reply, addr)

    def parse_car_telemetry(self, text, addr):
        if not self.car_telemetry_enabled or self.car_pub is None:
            rospy.logwarn_throttle(
                2.0,
                "TEL:CAR ignored because fake-car mode owns /car/state",
            )
            return
        role = self.source_role(addr)
        if self.strict_source_ip and role not in {"CAR", "DEBUG"}:
            return
        prefix = "TEL:CAR:"
        try:
            data = json.loads(text[len(prefix):])
        except Exception as exc:
            rospy.logwarn_throttle(1.0, "Invalid TEL:CAR JSON: %s", str(exc))
            return

        required = [
            "seq", "time_ms", "run", "task", "state", "x_cm", "y_cm",
            "speed_cm_s", "yaw_deg", "segment", "path_s_cm",
            "line_detected", "error",
        ]
        missing = [key for key in required if key not in data]
        if missing:
            rospy.logwarn_throttle(1.0, "TEL:CAR missing fields: %s", ",".join(missing))
            return

        task = normalize_task(data.get("task"))
        if not task:
            rospy.logwarn_throttle(1.0, "TEL:CAR bad task: %s", str(data.get("task")))
            return
        segment_reported = str(data.get("segment", "UNKNOWN")).upper()
        if segment_reported not in VALID_SEGMENTS:
            segment_reported = "UNKNOWN"

        x_field = safe_float(data.get("x_cm")) / 100.0
        y_field = safe_float(data.get("y_cm")) / 100.0
        speed = max(0.0, safe_float(data.get("speed_cm_s")) / 100.0)
        yaw_field = math.radians(safe_float(data.get("yaw_deg")))
        vx_field = speed * math.cos(yaw_field)
        vy_field = speed * math.sin(yaw_field)
        x_local, y_local = self.transform.field_to_local_xy(x_field, y_field)
        vx_local, vy_local = self.transform.field_to_local_vector(vx_field, vy_field)
        yaw_local = self.transform.field_to_local_yaw(yaw_field)

        path_s_m = max(0.0, safe_float(data.get("path_s_cm")) / 100.0)
        segment_from_path, segment_progress, lap_progress = self.track.calculate(path_s_m)
        consistent = (
            segment_reported == segment_from_path
            or segment_reported == "UNKNOWN"
            or self.track.near_boundary(path_s_m)
        )
        control_segment = segment_reported
        if self.require_segment_consistency and not consistent:
            control_segment = "UNKNOWN"
            rospy.logwarn_throttle(
                0.8,
                "CAR segment/path mismatch: reported=%s calculated=%s path=%.2fm",
                segment_reported,
                segment_from_path,
                path_s_m,
            )

        packet_delay = 0.0
        if self.run_start_monotonic is not None and str(data.get("run")) == self.run_id:
            local_elapsed = time.monotonic() - self.run_start_monotonic
            remote_elapsed = max(0.0, safe_float(data.get("time_ms")) / 1000.0)
            packet_delay = clamp(
                local_elapsed - remote_elapsed, 0.0, self.max_delay_compensation_s
            )
            x_local += vx_local * packet_delay
            y_local += vy_local * packet_delay

        state = str(data.get("state", "RUNNING")).upper()
        output = {
            "type": "car_state",
            "stamp": rospy.Time.now().to_sec(),
            "seq": safe_int(data.get("seq"), 0),
            "time_ms": safe_int(data.get("time_ms"), 0),
            "run_id": str(data.get("run", "R000")),
            "mission_id": run_id_to_int(data.get("run", "R000")),
            "mission_type": task_to_mission_type(task),
            "x": x_local,
            "y": y_local,
            "vx": vx_local,
            "vy": vy_local,
            "yaw": yaw_local,
            "speed": speed,
            "segment": control_segment,
            "segment_reported": segment_reported,
            "segment_from_path": segment_from_path,
            "segment_consistent": consistent,
            "segment_progress": segment_progress,
            "path_s": path_s_m,
            "path_s_cm": safe_float(data.get("path_s_cm")),
            "lap_progress": lap_progress,
            "point": str(data.get("point", "")),
            "running": state not in {"IDLE", "READY", "FINISHED", "FAULT", "LINE_LOST"},
            "state": state,
            "line_detected": bool(data.get("line_detected", False)),
            "battery": safe_int(data.get("battery", -1), -1),
            "error": safe_int(data.get("error", 0), 0),
            "packet_delay_s": packet_delay,
        }
        self.car_pub.publish(
            String(data=json.dumps(output, ensure_ascii=False, separators=(",", ":")))
        )

    def parse_car_event(self, text, addr):
        role = self.source_role(addr)
        if self.strict_source_ip and role not in {"CAR", "DEBUG"}:
            return
        self.car_event_pub.publish(String(data=text))
        # 地面站即使没有直接收到小车事件，也能从无人机转发链看到一次。
        target = self.get_ground_station_addr()
        if target is not None:
            self.send_text(text, target)

    @staticmethod
    def map_public_state(status):
        detail = str(status.get("fsm_state", "WAIT_START"))
        mission_type = str(status.get("mission_type", "drop"))
        if detail == "WAIT_START":
            return "WAIT_START"
        if detail in {"WAIT_FCU", "TAKEOFF"}:
            return "TAKEOFF"
        if detail == "DROP_HOVER":
            return "HOVER"
        if detail == "INTERCEPT":
            return "SEARCH_CAR" if mission_type == "drop" else "APPROACH_CAR"
        if detail in {"FOLLOW_DROP", "FOLLOW_CD"}:
            return "FOLLOW" if mission_type == "drop" else "APPROACH_CAR"
        if detail in {"DROP_DESCENT", "DROP_ALIGN"}:
            return "PREPARE_DROP"
        if detail == "DROP_WAIT_ACK":
            return "DROPPING"
        if detail == "POST_DROP_FOLLOW":
            return "DROP_DONE"
        if detail in {"DYNAMIC_DESCENT", "PLATFORM_DISARM"}:
            return "LAND_ON_CAR"
        if detail == "PLATFORM_DWELL":
            return "ON_CAR"
        if detail == "PLATFORM_TAKEOFF":
            return "TAKEOFF_FROM_CAR"
        if detail == "RETURN_HOME":
            return "RETURN_HOME"
        if detail == "HOME_LAND":
            return "LAND_HOME"
        if detail == "EMERGENCY_LAND":
            return "LAND_HOME"
        if detail == "WAIT_RESET":
            return "DONE"
        return detail

    @staticmethod
    def map_safety(status):
        detail = str(status.get("fsm_state", ""))
        safety = str(status.get("safety_state", "NORMAL")).upper()
        if detail == "EMERGENCY_LAND":
            return "LANDING"
        if detail == "WAIT_RESET" and not bool(status.get("mavros", {}).get("armed", False)):
            return "LANDED"
        if safety in {"IDLE", "NORMAL"}:
            return "NORMAL"
        if "EMERGENCY" in safety or "ABORT" in safety:
            return "ABORTING"
        return "WARNING"

    def uav_payload(self):
        status = self.latest_status or {}
        uav = status.get("uav", {})
        x_local = safe_float(uav.get("x_rel_home"), 0.0)
        y_local = safe_float(uav.get("y_rel_home"), 0.0)
        x_field, y_field = self.transform.local_to_field_xy(x_local, y_local)
        vx_field, vy_field = self.transform.local_to_field_vector(
            safe_float(uav.get("vx"), 0.0), safe_float(uav.get("vy"), 0.0)
        )
        yaw_deg = self.transform.local_to_field_yaw_deg(safe_float(uav.get("yaw"), 0.0))
        mission_type = str(status.get("mission_type", "drop"))
        task = 2 if mission_type == "dynamic_land" else 1
        run_id = str(status.get("run_id", self.run_id))
        if run_id in {"", "None"}:
            run_id = self.run_id
        if self.run_start_monotonic is None:
            time_ms = 0
        else:
            time_ms = int(max(0.0, time.monotonic() - self.run_start_monotonic) * 1000.0)
        tracking = status.get("tracking", {})
        vision = status.get("vision", {})
        abort_reason = str(status.get("abort_reason", ""))
        return {
            "proto": self.proto,
            "seq": self.tel_seq,
            "time_ms": time_ms,
            "run": run_id,
            "task": task,
            "boot": self.boot_state,
            "state": self.map_public_state(status),
            "safety": self.map_safety(status),
            "armed": bool(status.get("mavros", {}).get("armed", False)),
            "fcu": bool(status.get("mavros", {}).get("connected", False)),
            "mode": str(status.get("mavros", {}).get("mode", "")),
            "x_cm": round(x_field * 100.0, 1),
            "y_cm": round(y_field * 100.0, 1),
            "z_cm": round(safe_float(uav.get("z_rel_home"), 0.0) * 100.0, 1),
            "vx_cm_s": round(vx_field * 100.0, 1),
            "vy_cm_s": round(vy_field * 100.0, 1),
            "vz_cm_s": round(safe_float(uav.get("vz"), 0.0) * 100.0, 1),
            "yaw_deg": round(yaw_deg, 1),
            "battery": self.battery_percent,
            "target_locked": bool(vision.get("valid", False) or tracking.get("valid", False)),
            "error": 0 if not abort_reason else 1,
        }

    def send_uav_telemetry(self, target=None, force=False):
        if not self.latest_status:
            return
        now = time.monotonic()
        if not force and now - self.last_tel < 1.0 / self.tel_rate_hz:
            return
        self.last_tel = now
        self.tel_seq += 1
        payload = self.uav_payload()
        payload["seq"] = self.tel_seq
        packet = "TEL:UAV:" + json.dumps(
            payload, ensure_ascii=False, separators=(",", ":")
        )
        destination = target or self.get_ground_station_addr()
        if destination is not None:
            self.send_text(packet, destination)

    def send_heartbeat(self):
        now = time.monotonic()
        if now - self.last_hb < 1.0 / self.hb_rate_hz:
            return
        self.last_hb = now
        self.hb_seq += 1
        state = self.map_public_state(self.latest_status) if self.latest_status else self.boot_state
        target = self.get_ground_station_addr()
        if target is not None:
            self.send_text("HB:UAV:{}:{}".format(self.hb_seq, state), target)

    def process_packet(self, text, addr):
        if text.startswith("CMD:"):
            self.handle_command(text, addr)
        elif text.startswith("TEL:CAR:"):
            self.parse_car_telemetry(text, addr)
        elif text.startswith("EVT:CAR_POINT:"):
            self.parse_car_event(text, addr)
        elif self.allow_legacy and text.strip().upper() in {"PING", "STATUS", "LAND", "RESET"}:
            legacy = "CMD:L{:06d}:{}".format(
                int(time.monotonic() * 1000) % 1000000, text.strip().upper()
            )
            self.handle_command(legacy, addr)
        else:
            rospy.logwarn_throttle(1.0, "Unsupported UDP packet from %s: %s", addr[0], text[:100])

    def spin(self):
        while not rospy.is_shutdown():
            try:
                raw, addr = self.sock.recvfrom(self.max_packet_bytes + 1)
                if len(raw) > self.max_packet_bytes:
                    self.send_text("ERR:0000:UNKNOWN:BAD_FORMAT:PACKET_TOO_LARGE", addr)
                    continue
                text = raw.decode("utf-8", errors="replace").strip()
                if text:
                    self.process_packet(text, addr)
            except socket.timeout:
                pass
            except OSError as exc:
                if not rospy.is_shutdown():
                    rospy.logwarn_throttle(1.0, "UDP receive failed: %s", str(exc))
            self.check_boot_transition()
            self.send_heartbeat()
            self.send_uav_telemetry()


if __name__ == "__main__":
    try:
        UavUdpGateway().spin()
    except rospy.ROSInterruptException:
        pass
