"""CLI 진입점.

  python -m candrive.cli demo            # 하드웨어 0개, 가상 드라이브 포함 주행
  python -m candrive.cli demo --scenario square --dashboard
  python -m candrive.cli info            # 설정 요약 + 스케일 계산 검산
  python -m candrive.cli sim             # 가상 드라이브만 띄우기(다른 IPC 에서 접속)
  python -m candrive.cli drive --backend socketcan --v 0.3 --w 0.0 --time 5
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time

from .config import Settings, CONFIG_DIR, ROOT
from .runner import DEFAULT_SCENARIO, SQUARE_SCENARIO, DemoRunner, DriveController

LOG_DIR = os.path.join(ROOT, "logs")
SCENARIOS = {"default": DEFAULT_SCENARIO, "square": SQUARE_SCENARIO}


def cmd_info(args) -> int:
    s = Settings.load(backend=args.backend)
    print(s.hw.summary())
    print(f"\n[프로파일] {s.profile.name} (axis offset 0x{s.profile.axis_index_offset:X}, "
          f"velocity unit = {s.profile.velocity_unit})")
    print(f"[버스]     backend={s.bus.backend} {s.bus.kwargs}")
    print("\n[검산] 지령 → CAN 원시값 → 되돌린 값")
    from .kinematics import DiffDriveKinematics, Twist
    kin = DiffDriveKinematics(s.hw)
    for v, w in [(0.5, 0.0), (0.0, 1.0), (0.3, 0.5), (1.2, 0.0)]:
        l, r = kin.twist_to_wheel_rpm(Twist(v, w))
        print(f"  v={v:5.2f} m/s, w={w:5.2f} rad/s  →  좌 {l:8.2f} rpm, "
              f"우 {r:8.2f} rpm  (모터 {l*s.hw.gear_ratio:8.1f} / "
              f"{r*s.hw.gear_ratio:8.1f} rpm)")
    print(f"\n[COB-ID 맵] node {s.hw.node_id}")
    for p in s.profile.rx_pdos:
        print(f"  RPDO 0x{p.cob_base + s.hw.node_id:03X}  {p.name}")
    for p in s.profile.tx_pdos:
        print(f"  TPDO 0x{p.cob_base + s.hw.node_id:03X}  {p.name}")
    print(f"  SDO  0x{s.profile.cob['sdo_rx_base'] + s.hw.node_id:03X} / "
          f"0x{s.profile.cob['sdo_tx_base'] + s.hw.node_id:03X}")
    print(f"  HB   0x{s.profile.cob['heartbeat_base'] + s.hw.node_id:03X}")
    return 0


def cmd_demo(args) -> int:
    scenario = SCENARIOS[args.scenario]
    with DemoRunner(backend=args.backend, verbose=not args.quiet,
                    with_simulator=not args.no_sim) as runner:
        runner.run(scenario)
        os.makedirs(LOG_DIR, exist_ok=True)
        csv_path = runner.save_csv(os.path.join(LOG_DIR, "run.csv"))
        trace_path = runner.save_trace(os.path.join(LOG_DIR, "can_trace.json"))
        rep = runner.report()
        print("\n" + json.dumps(rep, ensure_ascii=False, indent=2))
        print(f"\n주행 로그 : {csv_path}")
        print(f"CAN 트레이스: {trace_path}")
        if args.dashboard:
            from dashboard.build import build_dashboard
            out = build_dashboard(runner, os.path.join(LOG_DIR, "dashboard.html"))
            print(f"대시보드   : {out}")
    return 0


def cmd_sim(args) -> int:
    from .bus import CanLink
    from .sim.node import VirtualDrive
    s = Settings.load(backend=args.backend)
    link = CanLink(s.bus, name="drive-sim")
    sim = VirtualDrive(link, s.hw, s.profile)
    sim.start()
    print(f"[sim] 가상 2ELD2-CAN 드라이브 실행 중 — {link.describe()}, "
          f"node {s.hw.node_id}")
    print("      Ctrl+C 로 종료")
    try:
        while True:
            time.sleep(1.0)
            t = sim.truth()
            print(f"  nmt=0x{t['nmt_state']:02X} "
                  + "  ".join(f"축{ax}:{v['state']}/{v['rpm']:.0f}rpm"
                              for ax, v in t["axes"].items()))
    except KeyboardInterrupt:
        sim.stop()
        link.close()
    return 0


def cmd_drive(args) -> int:
    """실물(또는 별도 프로세스 시뮬)에 연결해 단일 지령 주행."""
    s = Settings.load(backend=args.backend)
    ctl = DriveController(s)
    try:
        ctl.bringup()
        end = time.monotonic() + args.time
        while time.monotonic() < end:
            t0 = time.monotonic()
            ctl.set_cmd_vel(args.v, args.w)
            smp = ctl.tick(t0)
            print(f"\r v={smp.slew_v:5.2f} w={smp.slew_w:5.2f} | "
                  f"L{smp.fb_left_rpm:7.1f} R{smp.fb_right_rpm:7.1f} rpm | "
                  f"x={smp.x:6.3f} y={smp.y:6.3f} θ={smp.theta_deg:6.1f}°", end="")
            time.sleep(max(0.0, s.hw.control_period_s - (time.monotonic() - t0)))
        print()
    finally:
        ctl.shutdown()
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="candrive",
                                 description="2ELD2-CAN CANopen 데모 스택")
    ap.add_argument("--backend", default=None,
                    help="virtual | socketcan | vcan | pcan (기본: bus.yaml)")
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("info", help="설정/스케일 검산 출력")
    p.set_defaults(func=cmd_info)

    p = sub.add_parser("demo", help="하드웨어 없이 시나리오 주행")
    p.add_argument("--scenario", choices=list(SCENARIOS), default="default")
    p.add_argument("--no-sim", action="store_true",
                   help="가상 드라이브를 띄우지 않음(실물/외부 시뮬에 연결)")
    p.add_argument("--dashboard", action="store_true", help="HTML 대시보드 생성")
    p.add_argument("--quiet", action="store_true")
    p.set_defaults(func=cmd_demo)

    p = sub.add_parser("sim", help="가상 드라이브만 실행")
    p.set_defaults(func=cmd_sim)

    p = sub.add_parser("drive", help="단일 지령으로 주행")
    p.add_argument("--v", type=float, default=0.3)
    p.add_argument("--w", type=float, default=0.0)
    p.add_argument("--time", type=float, default=5.0)
    p.set_defaults(func=cmd_drive)

    args = ap.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.path.insert(0, ROOT)
    sys.exit(main())
