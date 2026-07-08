# SparseNMPC_IPM API 레퍼런스 (API Reference)

!!! abstract "개요 (Overview)"
    이 문서는 `SparseNMPC_IPM.hpp` 헤더에 대한 로우 레벨 API 레퍼런스를 제공합니다. [Sparse NMPC 개요](Overview.md)가 시스템 아키텍처와 이론적인 알고리즘 흐름을 제공한다면, 이 페이지는 정확한 C++ 데이터 구조, 구성 파라미터 및 클래스 메서드를 자세히 설명합니다.

## :material-tune-vertical: 구성 (Configuration): `NMPCTuningConfig`

`NMPCTuningConfig` 구조체는 솔버를 위한 모든 튜닝 가능한 가중치, 한계 및 기준 파라미터를 보관합니다.

=== "상태 페널티 (State Penalties, Q)"
    | 파라미터 | 기본값 | 설명 |
    | :--- | :--- | :--- |
    | `Q_D` | 200.0 | 기준 경로로부터의 횡방향 편차에 대한 페널티 |
    | `Q_mu` | 50.0 | 경로에 대한 헤딩 오차에 대한 페널티 |
    | `Q_Vx` | 50.0 | 종방향 속도 오차에 대한 페널티 |
    | `Q_Vy` | 1.0 | 횡방향 속도 (사이드 슬립)에 대한 페널티 |
    | `Q_r` | 1.0 | 요 레이트(yaw rate) 편차에 대한 페널티 |
    | `Q_alpha_f`, `Q_alpha_r` | 10.0 | 전륜 및 후륜 타이어 슬립각에 대한 페널티 |

=== "입력 페널티 (Input Penalties, R & Rate)"
    | 파라미터 | 기본값 | 설명 |
    | :--- | :--- | :--- |
    | `R_Steer` | 5000.0 | 절대 조향 크기에 대한 페널티 |
    | `R_Accel` | 10.0 | 절대 가속도 크기에 대한 페널티 |
    | `R_SteerRate` | 1500.0 | 조향 변화율 (저크/슬루 레이트)에 대한 페널티 |
    | `R_AccelRate` | 100.0 | 가속도 변화율 (저크/슬루 레이트)에 대한 페널티 |

=== "제약 조건 및 한계 (Constraints & Bounds)"
    | 파라미터 | 기본값 | 설명 |
    | :--- | :--- | :--- |
    | `d_max`, `d_min` | 3.5, -3.5 | 최대 및 최소 횡방향 경계 (도로 한계) |
    | `u_max`, `u_min` | {0.6, 10}, {-0.6, -10} | {조향, 가속도}의 한계값 |
    | `Obstacle_Margin` | 1.5 | 동적 장애물 주변에 추가되는 안전 여유 반경 |

## :material-cube-outline: 핵심 클래스 (Core Class): `SparseNMPC_IPM`

```cpp
template <std::size_t H, typename PlantModel, std::size_t Nx_mem, std::size_t Nx_active, std::size_t Nu>
class SparseNMPC_IPM;
```

원시-쌍대 내점법(Primal-Dual Interior-Point Method)을 실행하는 메인 템플릿 클래스입니다.

### 1. 템플릿 파라미터 (Template Parameters)
- **`H`**: 예측 구간(Prediction horizon) 길이.
- **`PlantModel`**: 차량 동역학 모델 (기본값: `RealTimeDynamicsModel`).
- **`Nx_mem`**: 메모리에 있는 전체 상태 차원.
- **`Nx_active`**: 솔버에 의해 최적화되는 활성 상태 차원.
- **`Nu`**: 제어 입력의 개수.

### 2. 내부 구조체 (Internal Structures)
```cpp
struct ConstraintState {
    double s = 1.0;     // 여유 변수 (Slack variable, > 0 이어야 함)
    double lam = 1.0;   // 쌍대 변수 (Dual variable, 승수)
    double ds = 0.0;    // 여유 변수에 대한 뉴턴 스텝
    double dlam = 0.0;  // 쌍대 변수에 대한 뉴턴 스텝
};
```
각 제약 조건(예: $u_{max}$, $d_{min}$, 장애물)은 `ConstraintState` 인스턴스를 유지합니다. 이들은 예측 구간(horizon)을 따라 `IPMDuals` 배열로 그룹화됩니다.

### 3. 주요 공개 메서드 (Key Public Methods)

#### `shift_sequence()`
```cpp
inline void shift_sequence();
```
- **목적**: 예측 시퀀스를 하나의 시간 스텝 $\Delta t$ 만큼 앞으로 밉니다.
- **동작**: 궤적 데이터를 $k+1 \rightarrow k$로 복사하고, 안정성을 위해 마지막 제어 입력을 절반으로 줄이며, Runge-Kutta 4 (RK4) 적분을 사용하여 새로운 종료 상태 $X_{H}$를 예측합니다.

#### `solve_ipm()`
```cpp
NMPCResult solve_ipm(const matrix::StaticVector<double, Nx_mem>& x_curr, const NMPCTuningConfig& config);
```
- **목적**: NMPC 최적화를 실행하는 메인 진입점.
- **흐름 (Flow)**:
    1. 초기 상태를 검증합니다 (`NaN` 체크).
    2. 순방향 패스(forward pass)를 실행하여 여유 변수와 쌍대 변수를 초기화합니다.
    3. 메인 SQP 루프에 진입합니다 (최대 `ipm_max_iter`회).
    4. 자동 미분(AD)을 통해 자코비안을 계산합니다.
    5. KKT 시스템(`Q, R, q, r`)을 구성하고 `RiccatiSolver`를 사용하여 풉니다.
    6. 쌍대 스텝(dual steps)을 추출하고 장점 기반 백트래킹 라인 서치(Merit-based Backtracking Line Search)를 수행합니다.
    7. 수렴 또는 발산에 대해 KKT 잔차를 확인합니다.
- **반환값**: 수렴 상태, 반복 횟수 및 최종 KKT 오차를 포함하는 `NMPCResult`를 반환합니다.

#### `execute_fallback()`
```cpp
inline NMPCResult execute_fallback(NMPCResult& res, const std::string& reason, const NMPCTuningConfig& config);
```
- **목적**: IPM이 발산하거나 `NaN`이 발생하거나 리카티 시스템을 인수분해하지 못할 때 트리거됩니다.
- **동작**: 연속적이고 충돌 없는 제어 출력을 보장하기 위해 `SafeTrajectoryBuffer`에서 마지막으로 알려진 안전한 궤적을 자동으로 가져옵니다.

#### `evaluate_merit()`
```cpp
double evaluate_merit(double alpha, const NMPCTuningConfig& config, const matrix::StaticVector<double, Nx_mem>& x_init, double current_mu);
```
- **목적**: 주어진 스텝 크기 `alpha`에 대해 스칼라 장점 함수(Merit function) 값을 계산합니다.
- **구성**: 상태/입력 추적 비용(이차 형식), 부등식 제약 조건에 대한 로그 장벽 페널티(Log-Barrier penalties), 그리고 동역학 제약 조건 위반에 대한 정확한 L1 페널티(Exact L1 penalties)의 균형을 맞춥니다.

---

## :material-shield-car: 핵심 클래스 (Core Class): `SafeTrajectoryBuffer`

```cpp
template <std::size_t H, std::size_t Nx, std::size_t Nu>
class SafeTrajectoryBuffer;
```

IPM 발산 시 폴백(fallback)으로 작동할 수 있도록 마지막으로 실행 가능한(feasible) 시퀀스를 유지합니다.

### 주요 공개 메서드 (Key Public Methods)

#### `commit()`
```cpp
void commit(const std::array<matrix::StaticVector<double, Nx>, H + 1>& X,
            const std::array<matrix::StaticVector<double, Nu>, H>& U);
```
- **목적**: 성공적으로 수렴된 상태 및 제어 시퀀스를 캐시합니다.
- **동작**: `X` 및 `U`를 `X_safe` 및 `U_safe`에 복사하고 `has_valid_trajectory = true`로 설정합니다.

#### `generate_fallback_control()`
```cpp
matrix::StaticVector<double, Nu> generate_fallback_control(double braking_acceleration) const;
```
- **목적**: 결정론적인 비상 제동 명령을 생성합니다.
- **인수**: `braking_acceleration` ($0.0$ 이하여야 함).
- **반환값**: 조향은 $0.0$(직진)이고 가속도는 지정된 제동 값인 제어 입력을 반환합니다.
