#!/usr/bin/env python3
"""ROS 2 드라이버 노드.

구독:  /cmd_vel            (geometry_msgs/Twist)
발행:  /odom               (nav_msgs/Odometry)
       /joint_states       (sensor_msgs/JointState)  좌/우 휠
       /drive/diagnostics  (diagnostic_msgs/DiagnosticArray)
TF:    odom → base_link
서비스:/reset_odometry     (std_srvs/Trigger)

파라미터
  hardware_config : hardware.yaml 경로 (기본: 패키지 상대 config/hardware.yaml)
  profile_config  : CANopen 프로파일 YAML 경로
  can_backend     : virtual | socketcan | vcan | pcan  (bus.yaml 의 backend 를 덮어씀)
  with_simulator  : true 면 같은 프로세스에 가상 드라이브를 띄움(하드웨어 0개 데모)
  publish_tf      : odom→base_link TF 발행 여부
  odom_frame / base_frame
"""
from __future__ import annotations

import math
import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Quaternion, Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from tf2_ros import TransformBroadcaster

# candrive 를 import 경로에 넣는다 (colcon 설치 시엔 pip 설치본을 씀)
_HERE = os.path.dirname(os.path.abspath(__file__))
for _cand in (os.path.abspath(os.path.join(_HERE, "..", "..", "..")),
              os.environ.get("CANDRIVE_ROOT", "")):
    if _cand and os.path.isdir(os.path.join(_cand, "candrive")):
        sys.path.insert(0, _cand)
        break

from candrive.config import Settings                      # noqa: E402
from candrive.runner import DriveController               # noqa: E402


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class CanOdomDriver(Node):
    def __init__(self):
        super().__init__("can_odom_driver")

        self.declare_parameter("hardware_config", "")
        self.declare_parameter("profile_config", "")
        self.declare_parameter("can_backend", "")
        self.declare_parameter("with_simulator", False)
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")

        hw_path = self.get_parameter("hardware_config").value or None
        pf_path = self.get_parameter("profile_config").value or None
        backend = self.get_parameter("can_backend").value or None
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value

        self.settings = Settings.load(backend=backend, hardware_path=hw_path,
                                      profile_path=pf_path)
        self.get_logger().info("\n" + self.settings.hw.summary())

        # 하드웨어 없이 데모할 때: 같은 프로세스에서 가상 드라이브 기동
        self.sim = None
        self.sim_link = None
        if bool(self.get_parameter("with_simulator").value):
            from candrive.bus import CanLink
            from candrive.sim.node import VirtualDrive
            self.sim_link = CanLink(self.settings.bus, name="drive-sim")
            self.sim = VirtualDrive(self.sim_link, self.settings.hw,
                                    self.settings.profile)
            self.sim.start()
            self.get_logger().warn(
                "가상 드라이브 모드입니다 (실물 2ELD2-CAN 없음). "
                "실장 시 with_simulator:=false 로 두세요.")

        self.ctl = DriveController(self.settings, verbose=False)
        try:
            self.ctl.bringup()
        except Exception as exc:                            # noqa: BLE001
            self.get_logger().error(f"드라이브 기동 실패: {exc}")
            raise

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Twist, "cmd_vel", self.on_cmd_vel, qos)
        self.pub_odom = self.create_publisher(Odometry, "odom", qos)
        self.pub_js = self.create_publisher(JointState, "joint_states", qos)
        self.pub_diag = self.create_publisher(DiagnosticArray, "drive/diagnostics", 1)
        self.tf_bc = TransformBroadcaster(self)
        self.create_service(Trigger, "reset_odometry", self.on_reset)

        period = self.settings.hw.control_period_s
        self.create_timer(period, self.on_tick)
        self.create_timer(1.0, self.on_diag)
        self.get_logger().info(
            f"can_odom_driver 준비 완료 — {self.ctl.link.describe()}, "
            f"제어주기 {period*1000:.0f} ms")

    # ------------------------------------------------------------------ 콜백
    def on_cmd_vel(self, msg: Twist) -> None:
        self.ctl.set_cmd_vel(msg.linear.x, msg.angular.z)

    def on_reset(self, req, res):
        self.ctl.odom.reset()
        res.success = True
        res.message = "오도메트리 원점 초기화"
        return res

    def on_tick(self) -> None:
        smp = self.ctl.tick()
        now = self.get_clock().now().to_msg()
        pose = self.ctl.odom.pose

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = pose.x
        odom.pose.pose.position.y = pose.y
        odom.pose.pose.orientation = yaw_to_quat(pose.theta)
        odom.twist.twist.linear.x = self.ctl.odom.twist.linear
        odom.twist.twist.angular.z = self.ctl.odom.twist.angular
        # 휠 오도메트리 기본 공분산 (필요 시 튜닝)
        odom.pose.covariance[0] = 0.002
        odom.pose.covariance[7] = 0.002
        odom.pose.covariance[35] = 0.005
        odom.twist.covariance[0] = 0.002
        odom.twist.covariance[35] = 0.005
        self.pub_odom.publish(odom)

        if self.publish_tf:
            tf = TransformStamped()
            tf.header.stamp = now
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = pose.x
            tf.transform.translation.y = pose.y
            tf.transform.rotation = yaw_to_quat(pose.theta)
            self.tf_bc.sendTransform(tf)

        js = JointState()
        js.header.stamp = now
        js.name = ["left_wheel_joint", "right_wheel_joint"]
        cpr = self.settings.hw.counts_per_wheel_rev
        js.position = [smp.left_counts / cpr * 2 * math.pi,
                       smp.right_counts / cpr * 2 * math.pi]
        js.velocity = [smp.fb_left_rpm * 2 * math.pi / 60.0,
                       smp.fb_right_rpm * 2 * math.pi / 60.0]
        self.pub_js.publish(js)

    def on_diag(self) -> None:
        tel = self.ctl.drive.telemetry
        st = DiagnosticStatus()
        st.name = "2ELD2-CAN drive"
        st.hardware_id = f"node{self.settings.hw.node_id}"
        ok = all(tel.axes[a].state == "operation_enabled" for a in (1, 2))
        st.level = DiagnosticStatus.OK if ok else DiagnosticStatus.ERROR
        st.message = "정상" if ok else "축 상태 이상"
        st.values = [
            KeyValue(key="axis1_state", value=tel.axes[1].state),
            KeyValue(key="axis2_state", value=tel.axes[2].state),
            KeyValue(key="axis1_statusword", value=f"0x{tel.axes[1].statusword:04X}"),
            KeyValue(key="axis2_statusword", value=f"0x{tel.axes[2].statusword:04X}"),
            KeyValue(key="heartbeat", value=f"0x{tel.heartbeat_state:02X}"),
            KeyValue(key="online", value=str(tel.online())),
            KeyValue(key="watchdog_tripped", value=str(self.ctl.watchdog_tripped)),
            KeyValue(key="can_tx", value=str(self.ctl.link.tx_count)),
            KeyValue(key="can_rx", value=str(self.ctl.link.rx_count)),
            KeyValue(key="bus", value=self.ctl.link.describe()),
        ]
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = [st]
        self.pub_diag.publish(arr)

    def destroy_node(self):
        try:
            self.ctl.shutdown()
            if self.sim:
                self.sim.stop()
            if self.sim_link:
                self.sim_link.close()
        finally:
            super().destroy_node()


def main(argv=None):
    rclpy.init(args=argv)
    node = CanOdomDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
