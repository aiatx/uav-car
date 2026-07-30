#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""实飞用安全虚拟小车节点。

用途：
1. 在没有真实小车时，连续发布 /car/state，让真实无人机沿标定赛道跟随虚拟目标；
2. 可选发布合成 /uav/platform_vision，仅用于任务一低速航迹测试；
3. 默认不自动启动任务、不自动移动，必须收到明确命令或进入 INTERCEPT 后才运动；
4. 任务进入返航、降落、应急或等待复位时，虚拟小车立即停止。

严禁：在没有真实承载平台时，用合成视觉测试 dynamic_land 动态降落。
"""

import json
import math
import os
import threading

import rosgraph
import rospy
import yaml
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String
from tf.transformations import euler_from_quaternion


def clamp(value, low, high):
    return max(low, min(high, value))


def safe_float(value, default=0.0):
    try:
        return float(value)
    except Exception:
        return float(default)


def safe_int(value, default=0):
    try:
        return int(value)
    except Exception:
        return int(default)


class StadiumTrackModel:
    """以 H 点为原点的体育场形赛道模型。"""

    def __init__(self, cfg):
        self.a_x = safe_float(cfg.get("a_x_m", 1.25), 1.25)
        self.a_y = safe_float(cfg.get("a_y_m", 0.75), 0.75)
        self.yaw_ab = math.radians(safe_float(cfg.get("ab_yaw_deg", 90.0), 90.0))
        self.length = max(0.05, safe_float(cfg.get("straight_length_m", 1.50), 1.50))
        self.radius = max(0.05, safe_float(cfg.get("radius_m", 0.75), 0.75))
        self.total = 2.0 * self.length + 2.0 * math.pi * self.radius

    def local_to_relative_home(self, u, v):
        forward_x = math.cos(self.yaw_ab)
        forward_y = math.sin(self.yaw_ab)
        right_x = math.sin(self.yaw_ab)
        right_y = -math.cos(self.yaw_ab)
        return (
            self.a_x + forward_x * u + right_x * v,
            self.a_y + forward_y * u + right_y * v,
        )

    def heading(self, du, dv):
        forward_x = math.cos(self.yaw_ab)
        forward_y = math.sin(self.yaw_ab)
        right_x = math.sin(self.yaw_ab)
        right_y = -math.cos(self.yaw_ab)
        return math.atan2(
            forward_y * du + right_y * dv,
            forward_x * du + right_x * dv,
        )

    def map(self, path_s, speed):
        s = clamp(float(path_s), 0.0, self.total)
        ab_end = self.length
        bc_end = ab_end + math.pi * self.radius
        cd_end = bc_end + self.length

        if s < ab_end:
            segment = "AB"
            progress = s / self.length
            u, v = s, 0.0
            du, dv = 1.0, 0.0
        elif s < bc_end:
            theta = (s - ab_end) / self.radius
            segment = "BC"
            progress = theta / math.pi
            u = self.length + self.radius * math.sin(theta)
            v = self.radius - self.radius * math.cos(theta)
            du, dv = math.cos(theta), math.sin(theta)
        elif s < cd_end:
            q = s - bc_end
            segment = "CD"
            progress = q / self.length
            u = self.length - q
            v = 2.0 * self.radius
            du, dv = -1.0, 0.0
        else:
            theta = (s - cd_end) / self.radius
            segment = "DA"
            progress = theta / math.pi
            u = -self.radius * math.sin(theta)
            v = self.radius + self.radius * math.cos(theta)
            du, dv = -math.cos(theta), -math.sin(theta)

        x, y = self.local_to_relative_home(u, v)
        yaw = self.heading(du, dv)
        return {
            "x": x,
            "y": y,
            "vx": speed * math.cos(yaw),
            "vy": speed * math.sin(yaw),
            "yaw": yaw,
            "segment": segment,
            "segment_progress": clamp(progress, 0.0, 1.0),
            "path_s": s,
            "lap_progress": s / self.total,
        }


class RealFlightFakeCar:
    TERMINAL_STATES = {
        "RETURN_HOME",
        "HOME_LAND",
        "EMERGENCY_LAND",
        "WAIT_RESET",
    }

    def __init__(self):
        rospy.init_node("real_flight_fake_car", anonymous=False)

        default_yaml = os.path.expanduser(
            "~/catkin_ws/src/d26_air_ground_uav/config/air_ground_mission.yaml"
        )
        config_path = rospy.get_param("~mission_yaml", default_yaml)
        with open(config_path, "r", encoding="utf-8") as stream:
            cfg = yaml.safe_load(stream) or {}

        track_cfg = cfg.get("car_udp", {}).get("track", {})
        self.track = StadiumTrackModel(track_cfg)

        self.mission_type = str(rospy.get_param("~mission_type", "drop")).strip().lower()
        if self.mission_type not in {"drop", "dynamic_land"}:
            raise ValueError("mission_type must be drop or dynamic_land")

        self.max_speed = max(
            0.02,
            safe_float(rospy.get_param("~max_speed", 0.45), 0.45),
        )
        self.segment_speeds = {
            "AB": clamp(
                safe_float(rospy.get_param("~speed_ab", 0.08), 0.08),
                0.0,
                self.max_speed,
            ),
            "BC": clamp(
                safe_float(rospy.get_param("~speed_bc", 0.06), 0.06),
                0.0,
                self.max_speed,
            ),
            "CD": clamp(
                safe_float(rospy.get_param("~speed_cd", 0.08), 0.08),
                0.0,
                self.max_speed,
            ),
            "DA": clamp(
                safe_float(rospy.get_param("~speed_da", 0.06), 0.06),
                0.0,
                self.max_speed,
            ),
        }
        self.telemetry_rate_hz = clamp(
            safe_float(rospy.get_param("~telemetry_rate_hz", 30.0), 30.0),
            1.0,
            100.0,
        )
        requested_update_rate = max(
            20.0,
            safe_float(rospy.get_param("~update_rate_hz", 100.0), 100.0),
        )
        self.update_rate_hz = max(requested_update_rate, self.telemetry_rate_hz)
        self.path_s = clamp(
            safe_float(rospy.get_param("~start_path_s", 0.0), 0.0),
            0.0,
            self.track.total,
        )
        self.loop_track = bool(rospy.get_param("~loop_track", False))
        self.auto_run_on_intercept = bool(
            rospy.get_param("~auto_run_on_intercept", False)
        )
        self.publish_fake_vision = bool(
            rospy.get_param("~publish_fake_vision", False)
        )
        self.allow_dynamic_land_fake_vision = bool(
            rospy.get_param("~allow_dynamic_land_fake_vision", False)
        )
        self.abort_on_topic_conflict = bool(
            rospy.get_param("~abort_on_topic_conflict", True)
        )
        self.max_fake_vision_range = max(
            0.5,
            safe_float(rospy.get_param("~max_fake_vision_range_m", 6.0), 6.0),
        )

        if (
            self.publish_fake_vision
            and self.mission_type == "dynamic_land"
            and not self.allow_dynamic_land_fake_vision
        ):
            raise RuntimeError(
                "Synthetic vision for dynamic_land is disabled: no real platform means unsafe descent"
            )

        self.running = False
        self.scan_enabled = False
        self.fsm_state = "UNKNOWN"
        self.last_fsm_state = "UNKNOWN"
        self.mission_id = 1
        self.home = None
        self.current_pose = None
        self.current_yaw = 0.0
        self.vision_stable_count = 0
        self.start_timer = None
        self.lock = threading.Lock()

        self.car_pub = rospy.Publisher("/car/state", String, queue_size=30)
        self.vision_pub = rospy.Publisher(
            "/uav/platform_vision", String, queue_size=20
        )
        self.type_pub = rospy.Publisher(
            "/uav/mission_type", String, queue_size=3, latch=True
        )
        self.start_pub = rospy.Publisher("/uav/start", Bool, queue_size=3)
        self.status_pub = rospy.Publisher(
            "/fake_car/status", String, queue_size=3, latch=True
        )

        rospy.Subscriber("/fake_car/run", Bool, self.run_cb, queue_size=5)
        rospy.Subscriber("/fake_car/reset", Bool, self.reset_cb, queue_size=5)
        rospy.Subscriber("/fake_car/start_mission", Bool, self.start_mission_cb, queue_size=5)
        rospy.Subscriber("/uav/mission_status", String, self.mission_status_cb, queue_size=20)
        rospy.Subscriber("/uav/platform_scan_enable", Bool, self.scan_cb, queue_size=5)
        rospy.Subscriber(
            "/mavros/local_position/pose", PoseStamped, self.pose_cb, queue_size=20
        )

        self.type_pub.publish(String(data=self.mission_type))
        rospy.Timer(rospy.Duration(1.0), self.conflict_check_cb, oneshot=True)

        rospy.logwarn(
            "Real-flight fake car ready: task=%s path=%.2fm fake_vision=%s telemetry=%.1fHz. "
            "It is STOPPED until /fake_car/run or /fake_car/start_mission.",
            self.mission_type,
            self.path_s,
            self.publish_fake_vision,
            self.telemetry_rate_hz,
        )
        rospy.logwarn(
            "Segment target speeds: AB=%.3f, BC=%.3f, CD=%.3f, DA=%.3fm/s",
            self.segment_speeds["AB"],
            self.segment_speeds["BC"],
            self.segment_speeds["CD"],
            self.segment_speeds["DA"],
        )
        rospy.logwarn(
            "Track calibration: A=(%.3f, %.3f)m relative H, AB yaw=%.1fdeg, total=%.3fm",
            self.track.a_x,
            self.track.a_y,
            math.degrees(self.track.yaw_ab),
            self.track.total,
        )

    def conflict_check_cb(self, _event):
        if not self.abort_on_topic_conflict or rospy.is_shutdown():
            return
        try:
            master = rosgraph.Master(rospy.get_name())
            publishers, _, _ = master.getSystemState()
            topic_publishers = {topic: nodes for topic, nodes in publishers}
            conflicts = []
            for topic in ["/car/state"]:
                others = [
                    node for node in topic_publishers.get(topic, [])
                    if node != rospy.get_name()
                ]
                if others:
                    conflicts.append("{} <- {}".format(topic, ",".join(others)))
            if self.publish_fake_vision:
                others = [
                    node for node in topic_publishers.get("/uav/platform_vision", [])
                    if node != rospy.get_name()
                ]
                if others:
                    conflicts.append(
                        "/uav/platform_vision <- {}".format(",".join(others))
                    )
            if conflicts:
                rospy.logfatal(
                    "Fake-car topic conflict. Stop UDP/legacy/real-vision publishers first: %s",
                    "; ".join(conflicts),
                )
                rospy.signal_shutdown("topic publisher conflict")
        except Exception as exc:
            rospy.logerr("Unable to check ROS topic conflicts: %s", str(exc))

    def pose_cb(self, msg):
        self.current_pose = msg
        q = msg.pose.orientation
        _, _, self.current_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

    def scan_cb(self, msg):
        self.scan_enabled = bool(msg.data)
        if not self.scan_enabled:
            self.vision_stable_count = 0

    def mission_status_cb(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception:
            return

        self.last_fsm_state = self.fsm_state
        self.fsm_state = str(data.get("fsm_state", "UNKNOWN")).upper()
        self.mission_id = safe_int(data.get("mission_id", self.mission_id), self.mission_id)

        home = data.get("home", {})
        if all(home.get(key) is not None for key in ("x", "y", "z")):
            self.home = (
                safe_float(home.get("x"), 0.0),
                safe_float(home.get("y"), 0.0),
                safe_float(home.get("z"), 0.0),
            )

        if self.fsm_state in self.TERMINAL_STATES:
            if self.running:
                rospy.logwarn("Fake car stopped because FSM entered %s", self.fsm_state)
            self.running = False

        if (
            self.auto_run_on_intercept
            and self.fsm_state == "INTERCEPT"
            and self.last_fsm_state != "INTERCEPT"
        ):
            self.running = True
            rospy.logwarn("Fake car motion started automatically at INTERCEPT")

    def run_cb(self, msg):
        with self.lock:
            self.running = bool(msg.data)
        rospy.logwarn("Fake car running=%s", self.running)

    def reset_cb(self, msg):
        if not msg.data:
            return
        with self.lock:
            self.running = False
            self.path_s = clamp(
                safe_float(rospy.get_param("~start_path_s", 0.0), 0.0),
                0.0,
                self.track.total,
            )
            self.vision_stable_count = 0
        rospy.logwarn("Fake car reset to path_s=%.3fm and stopped", self.path_s)

    def start_mission_cb(self, msg):
        if not msg.data:
            return
        if self.home is None or self.current_pose is None:
            rospy.logerr("Start rejected: MAVROS pose/FSM home is not ready")
            return
        if self.fsm_state not in {"WAIT_START", "UNKNOWN"}:
            rospy.logerr("Start rejected: FSM state is %s", self.fsm_state)
            return

        self.type_pub.publish(String(data=self.mission_type))
        if self.start_timer is not None:
            try:
                self.start_timer.shutdown()
            except Exception:
                pass
        self.start_timer = rospy.Timer(
            rospy.Duration(0.20), self.publish_start_once, oneshot=True
        )
        rospy.logwarn(
            "Fake-car test start accepted. Mission=%s; car motion will %s.",
            self.mission_type,
            "begin at INTERCEPT" if self.auto_run_on_intercept else "remain stopped until /fake_car/run=True",
        )

    def publish_start_once(self, _event):
        self.start_pub.publish(Bool(data=True))

    def current_segment(self):
        return self.track.map(self.path_s, 0.0)["segment"]

    def current_target_speed(self):
        return self.segment_speeds[self.current_segment()]

    def advance(self, dt):
        if not self.running:
            return

        target_speed = self.current_target_speed()
        if target_speed <= 0.0:
            return

        next_s = self.path_s + target_speed * dt
        if next_s >= self.track.total:
            if self.loop_track:
                next_s %= self.track.total
            else:
                next_s = self.track.total
                self.running = False
                rospy.logwarn("Fake car reached end of one lap and stopped")
        self.path_s = next_s

    def current_car(self):
        effective_speed = self.current_target_speed() if self.running else 0.0
        return self.track.map(self.path_s, effective_speed)

    def publish_car_state(self, car, now):
        payload = {
            "type": "car_state",
            "source": "real_flight_fake_car",
            "mission_id": self.mission_id,
            "mission_type": self.mission_type,
            "stamp": now.to_sec(),
            "x": car["x"],
            "y": car["y"],
            "vx": car["vx"],
            "vy": car["vy"],
            "target_speed_mps": self.current_target_speed() if self.running else 0.0,
            "yaw": car["yaw"],
            "segment": car["segment"],
            "segment_progress": car["segment_progress"],
            "path_s": car["path_s"],
            "path_s_cm": car["path_s"] * 100.0,
            "lap_progress": car["lap_progress"],
            "segment_reported": car["segment"],
            "segment_from_path": car["segment"],
            "segment_consistent": True,
            "running": self.running,
        }
        self.car_pub.publish(
            String(data=json.dumps(payload, ensure_ascii=False, separators=(",", ":")))
        )

    def publish_synthetic_vision(self, car, now):
        if not self.publish_fake_vision or not self.scan_enabled:
            return

        detected = False
        forward = 0.0
        left = 0.0
        confidence = 0.0

        if self.home is not None and self.current_pose is not None:
            car_abs_x = self.home[0] + car["x"]
            car_abs_y = self.home[1] + car["y"]
            uav_x = self.current_pose.pose.position.x
            uav_y = self.current_pose.pose.position.y
            dx = car_abs_x - uav_x
            dy = car_abs_y - uav_y
            distance = math.hypot(dx, dy)
            if distance <= self.max_fake_vision_range:
                c = math.cos(self.current_yaw)
                s = math.sin(self.current_yaw)
                forward = c * dx + s * dy
                left = -s * dx + c * dy
                detected = True
                confidence = 1.0

        if detected:
            self.vision_stable_count = min(self.vision_stable_count + 1, 1000000)
        else:
            self.vision_stable_count = 0

        payload = {
            "target": "landing_platform",
            "source": "synthetic_real_flight_test",
            "detected": detected,
            "stamp": now.to_sec(),
            "confidence": confidence,
            "stable_count": self.vision_stable_count,
            "forward_m": forward,
            "left_m": left,
        }
        self.vision_pub.publish(
            String(data=json.dumps(payload, ensure_ascii=False, separators=(",", ":")))
        )

    def publish_status(self, car, now):
        payload = {
            "stamp": now.to_sec(),
            "running": self.running,
            "current_target_speed_mps": self.current_target_speed(),
            "segment_speeds_mps": dict(self.segment_speeds),
            "telemetry_rate_hz": self.telemetry_rate_hz,
            "path_s_m": self.path_s,
            "track_total_m": self.track.total,
            "segment": car["segment"],
            "segment_progress": car["segment_progress"],
            "fsm_state": self.fsm_state,
            "mission_type": self.mission_type,
            "fake_vision": self.publish_fake_vision,
            "scan_enabled": self.scan_enabled,
            "home_ready": self.home is not None,
            "pose_ready": self.current_pose is not None,
        }
        self.status_pub.publish(
            String(data=json.dumps(payload, ensure_ascii=False, separators=(",", ":")))
        )

    def spin(self):
        rate = rospy.Rate(self.update_rate_hz)
        last = rospy.Time.now()
        last_status = rospy.Time(0)
        telemetry_accumulator = 1.0 / self.telemetry_rate_hz
        telemetry_period = 1.0 / self.telemetry_rate_hz

        while not rospy.is_shutdown():
            now = rospy.Time.now()
            dt = clamp((now - last).to_sec(), 0.0, 0.10)
            last = now
            telemetry_accumulator += dt

            with self.lock:
                self.advance(dt)
                car = self.current_car()

            if telemetry_accumulator >= telemetry_period:
                self.publish_car_state(car, now)
                self.publish_synthetic_vision(car, now)
                telemetry_accumulator %= telemetry_period

            if (now - last_status).to_sec() >= 0.5:
                self.publish_status(car, now)
                last_status = now

            rate.sleep()


if __name__ == "__main__":
    try:
        RealFlightFakeCar().spin()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        pass
