"""candrive — 2ELD2-CAN7020B (CANopen CiA402) 2축 드라이브용 상위 제어 스택.

하드웨어 없이도 backend=virtual 로 전체 경로가 동작하고,
실물이 오면 config/hardware.yaml 의 제원값과 bus.yaml 의 backend만 수정
"""
from .config import Settings, HardwareConfig, BusConfig, ProfileConfig  # noqa: F401
from .bus import CanLink                                                # noqa: F401
from .drive import DualAxisDrive                                        # noqa: F401
from .kinematics import (DiffDriveKinematics, Odometry, Pose2D,         # noqa: F401
                         SlewLimiter, Twist)
from .runner import DriveController, DemoRunner, Segment, Sample        # noqa: F401

__version__ = "0.1.0"
