# 2ELD2-CAN 오도메트리 데모



```
                     ┌──────────────── 여기만 교체 ────────────────┐
                     │ config/hardware.yaml   (기구·엔코더·한계값) │
                     │ config/bus.yaml        (backend: virtual→socketcan) │
                     │ config/profiles/*.yaml (오브젝트 인덱스·PDO 매핑)   │
                     └──────────────────────────────────────────────┘
                                        ↓
 /cmd_vel ─→ 슬루제한 ─→ 역기구학 ─→ CANopen 코덱 ─→ [백엔드] ─→ 드라이브
                                        ↑                          │
 /odom + TF ←─ 오도 적분 ←─ 카운트 환산 ←─ CANopen 코덱 ←──────────┘
                                                    virtual / socketcan(PCAN-USB) / pcan
```


```bash
pip install python-can PyYAML
cd can_odom_demo

python3 -m candrive.cli info                 # 제원·스케일·COB-ID 맵 검산
python3 -m candrive.cli demo --dashboard     # 시나리오 주행 + HTML 대시보드
python3 -m pytest                            # 38개 검증 테스트
```


| 파일 | 내용 |
|---|---|
| `logs/dashboard.html` | 궤적·휠속도·CAN 프레임 트레이스 (브라우저에서 열면 재생 가능) |
| `logs/run.csv` | 매 제어주기 샘플 (지령/피드백/카운트/pose) |
| `logs/can_trace.json` | 마스터 관점 CAN 프레임 덤프 |


```bash
# 워크스페이스에 심볼릭 링크
ln -s $(pwd)/ros2/can_odom_ros ~/ros2_ws/src/
export CANDRIVE_ROOT=$(pwd)          # candrive 코어 위치
cd ~/ros2_ws && colcon build --packages-select can_odom_ros && source install/setup.bash

# 하드웨어 0개 데모 (가상 드라이브 + 대본 주행)
ros2 launch can_odom_ros demo.launch.py

# rviz 로 궤적 보기
ros2 launch can_odom_ros demo.launch.py rviz:=true

# 실물 연결 후 (PCAN-USB → can0), teleop 으로 직접 조종
ros2 launch can_odom_ros demo.launch.py \
    with_simulator:=false can_backend:=socketcan run_scenario:=false
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

| 인터페이스 | 타입 | 비고 |
|---|---|---|
| `/cmd_vel` (sub) | `geometry_msgs/Twist` | `linear.x`, `angular.z` |
| `/odom` (pub) | `nav_msgs/Odometry` | 공분산 포함 |
| `/joint_states` (pub) | `sensor_msgs/JointState` | 좌/우 휠 |
| `/drive/diagnostics` (pub) | `diagnostic_msgs/DiagnosticArray` | statusword, 하트비트, 프레임 카운트 |
| `/reset_odometry` (srv) | `std_srvs/Trigger` | 원점 초기화 |
| TF | `odom → base_link` | |


### 3-1. 결선·설정
- 드라이브 **DIP 스위치**: 노드 ID, 보레이트, 종단저항(양 끝단 120Ω 2개).
- PCAN-USB ↔ 드라이브: CAN_H / CAN_L / GND.

### 3-2. `config/bus.yaml`
```yaml
backend: socketcan       # virtual → socketcan
```
Linux 준비:
```bash
sudo ip link set can0 type can bitrate 500000
sudo ip link set up can0
candump can0             # 프레임이 흐르는지 눈으로 확인
```
Windows 라면 `backend: pcan` (PEAK PCAN-Basic 드라이버 필요).

### 3-3. `config/hardware.yaml` — `[SIM]` 태그가 붙은 값 전부 교체

| 값 |
|---|---|
| `wheel_radius_m` | 하중 실린 상태에서 실측 (무부하 반지름 아님) |
| `wheel_separation_m` | 좌우 구동륜 **접지점** 간 거리 |
| `gear_ratio` | 감속기 사양 |
| `encoder_counts_per_motor_rev` | 드라이브가 `0x6064` 로 리포트하는 최종 카운트 (체배 포함) |
| `invert_command` / `invert_feedback` | 아래 3-5 절차로 결정 |
| `max_motor_rpm` | 모터 정격 |
| `node_id`, `bitrate` | DIP 스위치와 반드시 일치 |

다 바꾸고 `meta.status` 를 `COMMISSIONED` 로 변경.

### 3-4. `config/profiles/cia402_leadshine_2eld2.yaml`
표준 CiA402 기준으로 채워둔 상태입니다. Leadshine 오브젝트 딕셔너리 표를 받으면
아래만 확인하세요.

1. **축2 인덱스 오프셋** (`axis_index_offset`, 지금 `0x800`)
   → `python3 -m candrive.cli info` 로 계산된 인덱스를 매뉴얼과 대조.
2. **`0x60FF` 목표속도의 단위** (`units.velocity`: `rpm` / `rpm_x10` / `counts_per_sec`)
   → 이게 틀리면 속도가 배수로 어긋납니다. 가장 흔한 실수 지점.
3. **기본 PDO 매핑** — 매뉴얼과 다르면 `pdo:` 표를 고치거나, `sdo_startup:` 에
   PDO 재매핑 SDO 를 추가.

### 3-5. 부호(방향) 확정 절차
```bash
# 바퀴를 띄운 상태에서
python3 -m candrive.cli drive --backend socketcan --v 0.1 --w 0 --time 3
```
- 두 바퀴가 **전진 방향**으로 돌면 `invert_command` OK. 반대면 해당 쪽을 뒤집기.
- 화면의 `L/R rpm` 피드백이 **양수**로 나오면 `invert_feedback` OK. 음수면 뒤집기.
- 그 다음 `x` 가 **증가**하면 오도메트리 부호 완료.

### 3-6. 캘리브레이션
```bash
# 직진: 바닥에 3 m 표시 → 실제 이동거리 / 화면 x
#   linear_scale = 실측 / 계산
# 회전: 제자리 10회전 → 실제 회전각 / 화면 θ
#   angular_scale = 실측 / 계산   (트레드 오차를 여기서 흡수)
```
`config/hardware.yaml` 의 `robot.calibration` 에 반영.

---

## 4. 구조

```
config/
  hardware.yaml                     ★ 실장 시 여기만 수정
  bus.yaml                          ★ backend 한 줄
  profiles/cia402_leadshine_2eld2.yaml   ★ 매뉴얼 확정 후 인덱스 교체
candrive/
  config.py       설정 로더 + 검증 + 파생 스케일 (모든 단위환산이 여기 한 곳)
  bus.py          백엔드 추상화 (virtual / socketcan / pcan) + 프레임 트레이스
  canopen.py      데이터타입·SDO·PDO·NMT 코덱 (프로파일 YAML 만 보고 동작)
  drive.py        2축 드라이브 마스터 클라이언트 + CiA402 상태머신
  kinematics.py   차동구동 역/정기구학, 원호적분 오도메트리, 슬루리미터
  runner.py       상위 제어 루프 (ROS 노드와 CLI 가 공유), 데모 시나리오
  cli.py          info / demo / sim / drive
  sim/            ★ 가상 하드웨어 — 실물 오면 그냥 안 띄우면 됨
    motor.py      모터 1차지연 모델 + 엔코더
    node.py       CANopen 슬레이브 시뮬레이터
ros2/can_odom_ros/  ROS 2 ament_python 패키지
dashboard/          단일 파일 HTML 대시보드 생성기
tests/              기구학·코덱·엔드투엔드 38개 테스트
```

---

## 5. 두 대 이상 / 실 버스 관찰

가상 버스 대신 Linux `vcan` 을 쓰면 `candump` 로 실제 프레임을 볼 수 있어
데모 설득력이 올라갑니다.

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0

# 터미널 1 — 가상 드라이브
python3 -m candrive.cli --backend vcan sim
# 터미널 2 — 프레임 관찰
candump -tz vcan0
# 터미널 3 — 마스터 (ROS 든 CLI 든)
python3 -m candrive.cli --backend vcan drive --v 0.3 --time 10
```

---

## 6. 주의점

- **`feedback_period_ms` 는 `control_period_ms` 의 절반 이하여야 합니다. 같으면 두
  주기가 맞물려 속도 추정에 톱니 리플이 생깁니다. 
- 엔코더 차분 미분에는 `can.odom_velocity_lpf_hz` 저역통과
  `/odom` 의 `twist` 는 필터값, CSV 의 `odom_v_raw/odom_w_raw` 는 원시값
- 휠 오도메트리는 미끄러짐을 볼 수 없습니다. 데모 시뮬은 `wheel_slip_ratio` 로
  이 오차를 일부러 재현 정밀도가 필요시 IMU 융합(`robot_localization`)을 추가
- `cmd_timeout_s` 워치독: 지령이 끊기면 자동 정지합니다.
- STO(Safe Torque Off)·브레이크 출력은 이 스택 범위 밖입니다. 안전회로는 별도 배선.
