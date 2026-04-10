# 모바일 매니퓰레이터 코드 수정 보고서

**작성일:** 2026-04-10 (0330 버전 병합 포함)

---

## 1. 개요

본 문서는 모바일 매니퓰레이터 ROS 워크스페이스에서 수행된 코드 수정 내용을 정리한 것입니다.
주요 변경 사항은 MoveIt을 roboticstoolbox-python으로 교체하고, pose 제어를 위한 좌표 변환을 보정하며, IK(역기구학) 초기값으로 joint CSV 데이터를 활용하는 것입니다.

---

## 2. 수정된 파일 목록

| 파일 | 유형 | 설명 |
|------|------|------|
| `arm_controller.py` | 수정 | MoveIt → roboticstoolbox 교체 (시뮬레이션용) |
| `arm_controllerwithscan.py` | 수정 | MoveIt → roboticstoolbox 교체 (시뮬레이션+스캔) |
| `arm_controllerrealwithscan_v2.py` | 신규 | 실제 로봇용 v2 (Fairino SDK + 보정된 변환) |
| `task_manager.py` | 수정 | pose 작업에 joint CSV q0 데이터 연동 |
| `task_executor.py` | 수정 | TEST_POSE 디버그 명령 추가 |
| `send_debug_cmd.py` | 수정 | TEST_POSE 명령 템플릿 추가 |

---

## 3. 주요 변경 사항

### 3.1 MoveIt → roboticstoolbox 교체

**대상 파일:** `arm_controller.py`, `arm_controllerwithscan.py`

**제거된 항목:**
- `import moveit_commander`
- `RobotCommander`, `PlanningSceneInterface`, `MoveGroupCommander`
- `self.arm.go()`, `self.arm.stop()`, `self.arm.set_pose_target()` 등 MoveIt API

**추가된 항목:**
- `import roboticstoolbox as rtb` — URDF 기반 로봇 모델 로딩
- `from spatialmath import SE3, UnitQuaternion` — 자세 표현
- `from roboticstoolbox import jtraj` — 관절 공간 궤적 보간 (5차 다항식)
- `/joint_states` 토픽 퍼블리셔 (50Hz)

**URDF:** `src/frcobot_ros/frcobot_description/urdf/fr10v6_vision.urdf`
- 구조: world → base_link → j1~j6 (revolute) → tool_Link → vision
- DOF: 6

**Joint 제어:**
```python
traj = jtraj(current_q, q_target, steps)  # 5차 다항식 보간
self._execute_trajectory(traj.q)           # /joint_states에 발행
```

**Pose 제어:**
```python
sol = self.robot.ikine_LM(T_target, q0=ik_seed)  # 수치 IK 풀이
traj = jtraj(current_q, sol.q, steps)              # 궤적 보간
```

**속도 제어:** `speed` 파라미터 (0~100)가 보간 스텝 수를 결정합니다. 값이 높을수록 스텝이 적어 동작이 빠릅니다.

---

### 3.2 좌표 변환 보정 (process_transforms)

**문제점:** 기존 `process_transforms`는 단순한 좌표 차이값을 사용하여 아이작 심(Isaac Sim) 월드 좌표계에서 arm base_link 좌표계로의 변환이 정확하지 않았습니다.

**발견된 문제:**
1. `msg.theta` (heading)가 각도(degree)인데 라디안으로 처리
2. CSV 자세(rx, ry, rz)의 오일러 순서가 `xyz`가 아닌 `zyx`
3. 좌표계 변환 매트릭스가 시뮬레이션 환경에 맞지 않음
4. 아암 마운트 오프셋이 로봇 heading에 따라 회전해야 함

**보정 방법:** grid_path_line1/line2 (pose CSV)와 optimized_joints_line1/line2 (joint CSV) 총 655개 데이터 쌍(4개 태그 그룹: 105, 106, 117, 118)을 사용하여 9-DOF 변환 파라미터를 최적화했습니다.

**9-DOF 보정 파라미터:**

| 파라미터 | 값 | 설명 |
|---------|-----|------|
| `arm_body_offset_x` | -0.166715 m | 바디 프레임 아암 오프셋 X |
| `arm_body_offset_y` | -0.254772 m | 바디 프레임 아암 오프셋 Y |
| `arm_base_z` | 0.974167 m | 아암 베이스 높이 (월드 Z) |
| `arm_tilt_x` | 0.054898 rad (3.15 deg) | R_aw X축 기울기 보정 |
| `arm_tilt_y` | 0.017894 rad (1.03 deg) | R_aw Y축 기울기 보정 |
| `arm_heading_bias` | 0.014368 rad (0.82 deg) | heading 편향 보정 |
| `arm_ori_corr_x` | -0.095520 rad (-5.47 deg) | 자세 보정 X |
| `arm_ori_corr_y` | -0.052944 rad (-3.03 deg) | 자세 보정 Y |
| `arm_ori_corr_z` | -0.008688 rad (-0.50 deg) | 자세 보정 Z |

**보정 결과:**

| 그룹 | Zone | 위치 오차 (평균) | 위치 오차 (최대) | 자세 오차 (평균) |
|------|------|----------------|----------------|----------------|
| 105 | B | 20.9 mm | 39.1 mm | 4.62 deg |
| 106 | B | 17.5 mm | 27.6 mm | 5.10 deg |
| 117 | C | 8.6 mm | 14.2 mm | 5.63 deg |
| 118 | C | 14.2 mm | 21.5 mm | 4.10 deg |

**IK 성공률:** 100% (34/34 테스트 포인트)

**변환 흐름:**
```
1. robot_pose (매니퓰레이터 프레임) → Isaac 월드 프레임 변환
   isaac_x = -msg.y, isaac_y = -msg.x

2. 바디 프레임 아암 오프셋 → heading으로 월드 프레임 회전
   world_off = Rz(theta) @ [body_off_x, body_off_y]

3. R_aw = Rz(theta + bias) * Ry(tilt_y) * Rx(tilt_x)

4. T_aw 구성 → 월드 좌표를 arm base_link 좌표로 변환
   p_arm = T_aw @ p_world

5. 자세: CSV ZYX 오일러 → R_corr 적용 → R_aw 회전
```

---

### 3.3 IK 초기값 (q0) 지원

**배경:** pose 모드에서 IK를 풀 때, 초기 관절 각도(q0)가 목표 해에 가까울수록 수렴이 빠르고 정확합니다. 이를 위해 기존에 보유한 joint CSV 데이터를 q0로 활용합니다.

**task_manager.py 변경:**

pose 작업 정의에 `joint_file` 필드를 추가했습니다:
```python
"scan_grid_line1": {
    "file": "grid_path_line1.csv",
    "type": "scan",
    "scan_mode": "pose",
    "joint_file": "optimized_joints_line1.csv",  # 추가
},
```

로딩 시 `(group_id, point_id)`를 키로 joint 데이터를 매핑하여 각 pose 스캔 포인트에 `q0` 필드를 추가합니다:
```python
point["q0"] = [q1, q2, q3, q4, q5, q6]  # 라디안
```

**arm_controller.py (시뮬레이션) 변경:**
```python
def _execute_pose_goal(self, pos, quat, speed, q0=None):
    if q0 is not None:
        ik_seed = np.array(q0, dtype=float)  # joint CSV 데이터 사용
    else:
        ik_seed = self.current_q              # 현재 관절 위치 사용
    sol = self.robot.ikine_LM(T_target, q0=ik_seed)
```

**arm_controllerrealwithscan_v2.py (실제 로봇) 변경:**

Fairino SDK의 `GetInverseKinRef` API를 활용합니다:
```python
def _exec_pose(self, p):
    q0 = p.get("q0")
    if q0 is not None:
        q0_deg = [np.degrees(j) for j in q0]
        ret, joints = self.robot.GetInverseKinRef(0, target, q0_deg)
    else:
        ret, joints = self.robot.GetInverseKin(0, target, config=-1)
```

| Fairino SDK API | 용도 |
|----------------|------|
| `GetInverseKin(type, desc_pos, config=-1)` | 기본 IK (현재 관절 위치 참조) |
| `GetInverseKinRef(type, desc_pos, joint_pos_ref)` | 지정된 관절 위치를 참조하여 IK 풀이 |
| `GetInverseKinHasSolution(type, desc_pos, joint_pos_ref)` | IK 해 존재 여부 확인 |

---

### 3.4 arm_controllerrealwithscan_v2.py (신규 파일)

**원본:** `arm_controllerrealwithscan.py`를 복사하여 생성

**변경 사항:**
1. `_transform_pose` — 보정된 9-DOF 변환으로 교체 (출력: mm + degree, Fairino SDK 호환)
2. `_exec_pose` — `q0` 가 있을 때 `GetInverseKinRef` 사용
3. 기존 `pose_base_z`, `pose_mb_z`, `pose_arm_z` 파라미터 제거

**변경되지 않은 부분:**
- 카메라 (CameraInterface), 추론 (InferenceInterface)
- Keyence 센서 거리 조정
- MoveJ, MoveL, SetSpeed, StopMotion
- 홈 위치 복귀, 취소 기능

**사용 방법:** `task_executor.py`에서 import 변경:
```python
# from arm_controllerrealwithscan import ArmController
from arm_controllerrealwithscan_v2 import ArmController
```

---

### 3.5 TEST_POSE 디버그 명령

**task_executor.py에 추가:**
```
TEST_POSE x y z [rx ry rz]
```

사용 예:
```bash
rostopic pub -1 /task_command std_msgs/String "TEST_POSE 0.737 2.14 0.704"
rostopic pub -1 /task_command std_msgs/String "TEST_POSE 0.737 2.14 0.704 1.5708 0.0 3.1416"
```

`/robot_pose`가 없는 경우 자동으로 모바일 컨트롤러의 위치를 사용하여 주입합니다.

---

## 4. 0330 버전 병합 (안전성/검증 기능)

0330 버전에서 추가된 안전성 및 검증 기능을 현재 rtb 기반 코드에 병합했습니다.

### 4.1 병합된 기능

| 기능 | 설명 |
|------|------|
| `JOINT_LIMITS` | FR10v6 관절 각도 제한 (6축) |
| `VELOCITY_LIMITS` | 관절 최대 속도 [3.15, 3.15, 3.15, 3.2, 3.2, 3.2] rad/s |
| `validate_joint_values()` | IK/운동 전 관절 한계 검증 |
| `validate_velocities()` | 관절 속도 한계 검증 |
| `is_discontinuous` 지원 | =1이면 해당 포인트 이동 전 Home 복귀 |
| 반환값 `(bool, str)` | 모든 운동 함수가 성공/실패 상태 반환 |
| `isaac_collision_detected` | Isaac Sim 충돌 검출 플래그 |
| `_save_results_to_new_csv()` | 실행 결과를 `_result.csv`로 실시간 저장 (withscan만) |
| `scan_joints_line1_new` 작업 | 새 작업 정의 (groups 103-109) |

### 4.2 복사된 파일

| 파일 | 설명 |
|------|------|
| `optimized_joints_line1_new.csv` | 새 joint CSV (is_discontinuous 열 포함) |
| `polishing_env_10255.usd` | 충돌 mesh trigger 객체 제거됨 |

### 4.3 task_manager.py 확장

모든 scan point에 메타데이터 필드 추가:
- `point_id` — CSV의 포인트 인덱스
- `group_id` — 태그 그룹 ID
- `csv_path` — 원본 CSV 파일 경로
- `is_discontinuous` — 비연속성 플래그 (0 또는 1)

**기존 q0/joint_file 로직 유지** (0330 버전에서 삭제되었으나 본 코드에서는 보존)

---

## 5. 의존성 변경

| 패키지 | 버전 | 용도 |
|-------|------|------|
| `roboticstoolbox-python` | 1.1.1 | URDF 로딩, FK/IK, 궤적 보간 |
| `spatialmath-python` | 1.1.15 | SE3, UnitQuaternion 자세 표현 |

설치:
```bash
pip3 install roboticstoolbox-python
```

**참고:** `scipy` 0.x~1.3 버전에서는 `as_matrix()` 대신 `as_dcm()`, `from_matrix()` 대신 `from_dcm()`을 사용해야 합니다. 본 코드에서는 해당 호환성 문제를 반영했습니다.

---

## 5. 좌표계 정리

```
Isaac Sim 월드 프레임 (CSV pose 좌표)
        │
        │ isaac_x = -manip_y
        │ isaac_y = -manip_x
        ▼
매니퓰레이터 프레임 (robot_pose msg)
        │
        │ T_aw = 보정된 9-DOF 변환
        ▼
Arm base_link 프레임 (IK 입력)
```

| 프레임 | 사용처 | 단위 |
|-------|--------|------|
| Isaac Sim 월드 | CSV pose 좌표 (x,y,z) | m, rad |
| 매니퓰레이터 | `/robot_pose` (x,y,theta) | m, deg |
| Arm base_link | IK 입력 (rtb) | m, rad |
| Arm base_link | IK 입력 (Fairino) | mm, deg |

---

## 6. 공개 인터페이스 (변경 없음)

`task_executor.py`가 사용하는 ArmController의 공개 API는 변경되지 않았습니다:

```python
class ArmController:
    def __init__(self, model_path=None)
    def execute_scan_points(self, scan_points)
    def move_to_home(self)
    def is_busy(self) -> bool
    def cancel(self)
    # 발행: /scan_finished (Bool)
```

---

## 7. ROS 파라미터 (신규)

`~` 프리픽스 프라이빗 파라미터로 변환 보정값을 오버라이드할 수 있습니다:

```yaml
arm_body_offset_x: -0.166715
arm_body_offset_y: -0.254772
arm_base_z: 0.974167
arm_tilt_x: 0.054898
arm_tilt_y: 0.017894
arm_heading_bias: 0.014368
arm_ori_corr_x: -0.095520
arm_ori_corr_y: -0.052944
arm_ori_corr_z: -0.008688
```
