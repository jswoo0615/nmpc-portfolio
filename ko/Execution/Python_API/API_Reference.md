# 파이썬 API 레퍼런스 (Python API Reference)

!!! info "API 문서 (API Documentation)"
    이 문서는 `nmpc_wrapper.cpp`에서 내보낸(exported) 파이썬 바인딩(bindings)을 요약합니다. 컴파일된 모듈의 이름은 일반적으로 `nmpc_core`입니다.

---

## :material-language-python: `SparseNMPCWrapper`

파이썬과 C++ NMPC 솔버를 연결하는 메인 클래스입니다.

### `__init__`
```python
def __init__(self, ego_state_arr: np.ndarray, wp_x_arr: np.ndarray, wp_y_arr: np.ndarray, obstacle_arr: np.ndarray)
```
래퍼(wrapper) 및 메모리 포인터를 초기화합니다.
- **`ego_state_arr`**: 차량 상태 `[x, y, yaw, vx, vy, yaw_rate]`를 포함하는 배열입니다.
- **`wp_x_arr`, `wp_y_arr`**: 기준 웨이포인트(waypoints)의 전역 좌표를 포함하는 배열입니다.
- **`obstacle_arr`**: 장애물당 동적 장애물 데이터 `[s, d, r, vs, vd]`를 포함하는 1차원(flattened) 배열입니다.

### `set_target_speed`
```python
def set_target_speed(self, speed: float) -> None
```
제어기의 목표 순항 속도(`config.target_vx`)를 업데이트합니다.

### `update_config`
```python
def update_config(self, opt_dict: dict) -> None
```
런타임에 비용 함수 가중치와 솔버 매개변수를 동적으로 업데이트합니다. 지원되는 키(keys)는 다음과 같습니다:
- `"dt"`: 이산화 시간 스텝(Discretization time step).
- `"Q_D"`, `"Q_mu"`, `"Q_Vx"`, `"Q_Vy"`: 상태 추적 페널티.
- `"R_Steer"`, `"R_Accel"`: 제어 노력(Control effort) 페널티.
- `"R_SteerRate"`, `"R_AccelRate"`: 제어 변화율(Control rate) 페널티.

### `solve`
```python
def solve(self, num_wp: int) -> tuple[dict, tuple[float, float]]
```
NMPC 최적화의 단일 스텝을 실행합니다.
1. 제공된 웨이포인트를 사용하여 `StaticCubicSpline2D`를 구성합니다.
2. 자아 차량(ego vehicle)의 전역 상태를 Frenet 프레임(s, d, $\mu$)에 투영(project)합니다.
3. 예측 구간(prediction horizon) 전체에 걸쳐 목표 궤적 곡률(`kappa_ref`)을 업데이트합니다.
4. 내점법(Interior Point Method)을 통해 비선형 계획법 문제를 풉니다 (`solve_ipm`).
5. 진단 데이터와 최적 제어값을 포함하는 튜플(tuple)을 반환합니다.

**반환값 (Returns):**
- `monitor_data` (dict): 솔버 지표를 포함합니다:
  - `"status"`: 완료 상태 메시지.
  - `"kkt_error"`: 최대 KKT 허용 오차(tolerance error).
  - `"sqp_iter"`: 실행된 SQP 반복 횟수.
  - `"min_slack"`, `"max_lambda"`: 물리적 제약 조건에 대한 진단 값.
- `u_opt` (tuple): 최적 제어 명령 `(가속도, 조향각)`.
