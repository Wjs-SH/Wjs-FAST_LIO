# 원본 대비 변경 내역

기준 원본은 [Ericsii/FAST_LIO](https://github.com/Ericsii/FAST_LIO) `ros2` 브랜치 커밋 `2fffc57`이다. 이 저장소는 [hku-mars/FAST_LIO](https://github.com/hku-mars/FAST_LIO)의 ROS 2 포트다.

아래 변경은 모두 `src/fast_lio/src/`의 4개 파일에 있다. 환경변수로 켜는 기능은 값이 0(기본)이면 원본과 똑같이 동작한다.

## 1. Hesai XT16 라이다 지원 (`preprocess.h`, `preprocess.cpp`)

- `lidar_type: 5`(`HESAI`)를 추가했다.
- 점 형식: `x, y, z, intensity(float32)`, `timestamp(float64, 절대 초)`, `ring(uint16)`
  - Velodyne의 `time`(float32, 스캔 시작 기준 상대값)과 이름·형식·의미가 모두 다르다. 그래서 처리 함수를 따로 만들었다.
  - 점마다 절대 시각에서 첫 점 시각을 빼서 FAST-LIO가 쓰는 상대 시간(ms)으로 바꾼다.
- `NaN` 점은 버린다(`is_dense=false`).
- `FASTLIO_MAXRANGE`(m, 0=끔): 이보다 먼 점을 버린다. XT16 사양 거리는 약 40 m인데 가끔 98 m까지 튀는 점이 있어서 넣었다.

## 2. IMU 진동 적응 공분산 (`IMU_Processing.hpp`)

로봇이 걸을 때 생기는 순간 진동에만 IMU를 덜 믿게 한다.

- IMU 샘플마다 저역 기준선을 갱신한다: `lp = α·lp + (1−α)·현재`
- 고주파 성분 `hf = 현재 − lp`로 공정 잡음 공분산을 키운다.
  - `cov_gyr ×= 1 + KG·|hf_gyr|²`
  - `cov_acc ×= 1 + KA·|hf_acc|²`
- 환경변수: `FASTLIO_SPK_KG`, `FASTLIO_SPK_KA`, `FASTLIO_SPK_ALPHA`(기본 0.9). 레시피는 KG=100, KA=1이다.

## 3. 구독 QoS (`laserMapping.cpp`)

- LiDAR·IMU 구독을 `RELIABLE + KeepAll`로 바꿨다. 원본은 SensorData / depth 10이다.
- 효과: bag 재생 속도가 처리 속도에 맞춰져 스캔을 버리지 않고, 결과가 실행마다 같아진다.
- 대신 재생 쪽도 reliable로 발행해야 한다: `ros2 bag play --qos-profile-overrides-path config/qos_reliable.yaml`

## 4. EKF 갱신 직후 상태 보정 (`laserMapping.cpp`)

`/leg_odom`(`nav_msgs/Odometry`)을 항상 구독한다. 반복 EKF 갱신이 끝날 때마다 아래 보정을 켜진 것만 적용하고 `kf.change_x`로 되돌려 넣는다.

| 환경변수 | 수식 | 가정 | 비고 |
|---|---|---|---|
| `FASTLIO_LEG_K` | 속도 크기를 다리 속도 쪽으로 `v ×= 1 + k(v_leg/‖v‖ − 1)`. ‖v‖>0.05 m/s이고 배율 0.2~3일 때만 | `/leg_odom.twist.linear.x`가 몸체 속력 | 레시피 0.5. 방향은 LiDAR+자이로가 정하고 크기만 보정한다 |
| `FASTLIO_ATTK` | 몸체 위쪽 축을 월드 +Z 쪽으로 각도의 k만큼 돌린다 | 평지 보행 | 레시피 0.05 |
| `FASTLIO_GNDK` | `z ← z + k(z0 − z)` (z0 = 첫 프레임 높이) | 한 층, 평평한 바닥 | 3층 200초 시험에서 z 드리프트 20.8 m → 1.25 m (k=0.05) |
| `FASTLIO_YAWK`, `FASTLIO_MANH_WARM` | 정합된 평면 법선 중 수평인 것(수직 벽)의 방향을 90° 단위로 평균한다. 처음 `MANH_WARM`(기본 150) 프레임으로 기준을 잡고, 이후 벗어난 만큼 yaw를 k만큼 되돌린다 | 벽이 서로 직교하는 건물 | 특징 없는 긴 복도에서 방향 오차 11.3° → 4.2° (k=0.05). 벽이 조금 두꺼워진다 |
| `FASTLIO_ODOMYAW` | `/leg_odom` 자세의 yaw에 처음 차이를 유지하며 k만큼 맞춘다 | `/leg_odom`에 자세가 들어 있음 | 실험용. `add_leg_odom.py`는 자세를 넣지 않으므로 기본 레시피에서는 효과가 없다 |
| `FASTLIO_ZK` | `v_z ×= 1 − k` | 평지 | 시험에서 효과 없었다. 스캔 정합이 매 프레임 위치를 다시 밀어 올리기 때문이다 |

## 5. 기타

- `rviz/fastlio.rviz`: 보기 설정만 바꿨다.
- `include/ikd-Tree`: 원래 서브모듈이던 것을 소스로 직접 넣었다([hku-mars/ikd-Tree](https://github.com/hku-mars/ikd-Tree) `fast_lio` 브랜치 `e2e3f4e`). clone만으로 빌드할 수 있게 하기 위해서다.
- 원본에서 뺀 것: `doc/`(README 그림 128 MB), `.github/`, `rviz_cfg/`(ROS 1용, 설치되지 않음). 그래서 `src/fast_lio/README.md`의 그림 링크는 깨져 있다.
- `src/livox_ros_driver2`: [Livox-SDK/livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2)에서 `CustomMsg`·`CustomPoint` 메시지 정의만 남긴 최소판이다.
  - FAST-LIO가 이 메시지 형식에 의존해서 필요하다.
  - Livox 드라이버 노드와 SDK는 빌드하지 않는다.
