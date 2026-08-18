from __future__ import annotations

import json
import os
from dataclasses import asdict
from typing import List, Optional

TEMPLATE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             "template.html")


def build_dashboard(runner, out_path: str,
                    trace_limit: int = 400,
                    sample_stride: Optional[int] = None) -> str:
    samples = [asdict(s) for s in runner.samples]
    if sample_stride is None:
        sample_stride = max(1, len(samples) // 1500)
    samples = samples[::sample_stride]

    hw = runner.s.hw
    payload = {
        "meta": {
            "robot": hw.raw["meta"].get("robot_name"),
            "drive": hw.raw["meta"].get("drive_model"),
            "status": hw.raw["meta"].get("status"),
            "bus": runner.ctl.link.describe(),
            "node_id": hw.node_id,
            "bitrate": hw.bitrate,
            "control_hz": round(1.0 / hw.control_period_s, 1),
            "profile": runner.s.profile.name,
            "velocity_unit": runner.s.profile.velocity_unit,
        },
        "spec": [
            ["휠 반지름", f"{hw.wheel_radius_m*1000:.1f} mm", "robot.wheel_radius_m"],
            ["트레드(좌우 간격)", f"{hw.wheel_separation_m*1000:.1f} mm",
             "robot.wheel_separation_m"],
            ["감속비", f"{hw.gear_ratio:.2f} : 1", "drivetrain.gear_ratio"],
            ["엔코더", f"{hw.counts_per_motor_rev} cnt/모터rev",
             "drivetrain.encoder_counts_per_motor_rev"],
            ["휠 1회전 카운트", f"{hw.counts_per_wheel_rev:.0f} cnt", "(계산값)"],
            ["오도 분해능", f"{hw.meters_per_count*1000:.5f} mm/cnt", "(계산값)"],
            ["최대 모터 회전수", f"{hw.max_motor_rpm:.0f} rpm", "limits.max_motor_rpm"],
            ["최대 직진 속도", f"{hw.max_linear_mps:.2f} m/s", "limits.max_linear_mps"],
            ["최대 선회 각속도", f"{hw.max_angular_rps:.2f} rad/s", "limits.max_angular_rps"],
            ["CAN 노드 / 보레이트", f"{hw.node_id} / {hw.bitrate//1000} kbit/s",
             "can.node_id, can.bitrate"],
        ],
        "report": runner.report(),
        "samples": samples,
        "trace": runner.ctl.link.recent(trace_limit),
    }

    with open(TEMPLATE_PATH, "r", encoding="utf-8") as f:
        html = f.read()
    html = html.replace("/*__DATA__*/null",
                        json.dumps(payload, ensure_ascii=False))
    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as f:
        f.write(html)
    return out_path
