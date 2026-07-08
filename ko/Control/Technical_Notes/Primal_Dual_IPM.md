# 실시간 NMPC를 위한 원시-쌍대 내점법 (Primal-Dual IPM for Real-Time NMPC)

!!! abstract "개요 (Overview)"
    내점법(Interior-Point Method, IPM)은 물리적 제약 조건(예: 조향각 한계, 장애물 회피)이 있는 비선형 계획법(Nonlinear Programming, NLP) 문제를 해결하기 위한 가장 강력한 엔진입니다. 그러나 자율 주행 차량의 실시간 제어를 위한 우리의 아키텍처는 무한한 계산 시간을 가정하는 '고전적인 IPM(Classical IPM)'에 의존하지 않습니다. 대신, 속도와 견고성(robustness)을 극대화하는 **현대적인 실행 불가능한 원시-쌍대 내점법(Infeasible Primal-Dual IPM)**을 채택합니다.

## :material-compare: 장벽 vs. 페널티 (Barrier vs. Penalty)

IPM의 핵심은 제약 조건을 목적 함수(objective function)에 흡수하는 데 있습니다. 페널티(penalty)와 장벽(barrier)을 엄격하게 구별하는 것이 중요합니다.

<div class="grid cards" markdown>

- :material-chart-bell-curve-cumulative:
    **이차 페널티 (Quadratic Penalty)** ($c(x)^2$)
    
    경계를 넘어서면 위반 정도의 제곱에 비례하는 페널티를 부과하는 '고무줄'과 같습니다. 일시적으로 경계 위반을 허용합니다.

- :material-wall:
    **로그 장벽 (Logarithmic Barrier)** ($-\mu \log(s)$)
    
    변수가 경계($0$)에 접근함에 따라 무한대($\infty$)로 치솟아 어떤 대가를 치르더라도 선을 넘지 못하게 하는 '콘크리트 벽'입니다.

</div>

- **전략 (Strategy)**: 여유 변수(slack variable)를 도입함으로써, 부등식 제약 조건(예: $g(x) \le 0$)은 $s > 0$으로 변환됩니다. 이 $s$에 로그 장벽을 적용하면 장애물 및 물리적 한계에 대해 '절대적인 안전 지대(내부, Interior)'가 강제됩니다.

## :material-math-integral: 수학적 핵심: KKT 조건 및 중심 경로 (Mathematical Heart: KKT Conditions & Central Path)

우리 솔버는 고전적인 방법처럼 목적 함수를 변환한 후 맹목적으로 수렴을 기다리지 않습니다.

### A. 수정된 KKT 조건 (Modified KKT Conditions)

최적해는 기울기가 0이 되는 Karush-Kuhn-Tucker (KKT) 조건을 만족해야 합니다. 우리는 장벽 매개변수 $\mu$를 포함하도록 상보성(Complementarity) 조건을 수정합니다.

!!! math "수정된 상보성 (Modified Complementarity)"
    $$
    s_{i} \cdot \lambda_{i} = \mu
    $$
    
*(여유 변수 $s$와 쌍대 변수 $\lambda$의 곱은 $\mu$와 같아야 합니다)*

### B. 뉴턴 방법을 통한 직접 풀이 (원시-쌍대 접근법) (Direct Solution via Newton's Method)

우리는 가우스-뉴턴(Gauss-Newton) 근사화된 **리카티 솔버(Riccati Solver, $\mathcal{O}(H)$ 시간 복잡도)** 를 사용하여 이 수정된 전체 KKT 시스템을 직접 풉니다. 상태 변수($x$, 원시/Primal)와 쌍대 변수($\lambda$, 쌍대/Dual)를 동시에 업데이트하여 해를 찾습니다.

### C. 동적 $\mu$ 업데이트 (중심 경로 추적) (Dynamic $\mu$ Update)

$25\text{ms}$ 제한 내에 해를 찾기 위해, 우리는 매 반복마다 중심 경로(central path)를 따라 $\mu$를 기계적으로 깎아냅니다. 현재의 평균 갭(gap)을 기반으로 코드는 한 번의 스텝에서 $\mu$를 $0.2$ 배수로 공격적으로 감소시켜 최적해($\mu \to 0$)에 빠르게 접근합니다.

## :material-door-open: 실행 불가능한 초기화 허용 (Allowing Infeasible Initialization)

이것이 1960년대 교과서와 현대 자율 주행 컨트롤러를 구분 짓는 가장 결정적인 차이점입니다.

!!! warning "고전적 한계 (Classical Limitation)"
    알고리즘이 시작되기 전부터 초기 추측치 $x^{(0)}$는 모든 제약 조건을 만족하는 완벽한 '안전한 내부(safe interior)'에 있어야 합니다. 만약 컨트롤러가 켜질 때 차량이 장애물 구역을 약간 침범했거나 차선을 밟고 있다면, 솔버는 즉시 발산합니다.

!!! success "실행 불가능한 원시-쌍대 내점법 (Infeasible Primal-Dual IPM)"
    차량의 물리적 상태($x$)나 제어 입력($u$)이 일시적으로 제약 조건을 위반하더라도(**실행 불가능, Infeasible**), 솔버는 멈추지 않습니다. 순전히 수학적인 내부 여유 변수($s$)와 쌍대 변수($\lambda$)가 엄격하게 양수($> 0$, **내부, Interior**)로 강제 초기화되기만 하면, 솔버는 물리적 궤적($x$)의 멱살을 잡고 실행 가능한 영역(feasible region) 안으로 효과적으로 끌고 들어옵니다.

## :material-map-legend: 최적 제어 문제(OCP)와의 매핑 (Mapping to Optimal Control Problem)

우리 코드베이스에서 IPM의 수학적 요소들은 물리적 현실과 다음과 같이 직접 연결됩니다:

| 수학적 개념 (Mathematical Concept) | 동역학 시스템 및 코드 구현 (Dynamics System & Code Implementation) | 아키텍처에서의 의미 (Meaning in Architecture) |
| :--- | :--- | :--- |
| **원시 변수 (Primal Variables)** | 상태 벡터 ($x_k$), 제어 입력 ($u_k$) | 물리적 제어 목표 (조향각, 가속도, 위치, 속도 등) |
| **쌍대 변수 (Dual Variables)** | 제약 조건 승수 ($\lambda$), 여유 변수 ($s$) | 제약 조건을 위반하려고 할 때 발생하는 척력(repulsive force)의 수학적 크기 |
| **목적 (Objectives)** | 행렬 $Q, R$에 기반한 단계 비용 ($L$) | 추적 오차 최소화와 승차감 확보(조향 억제) 사이의 근본적인 타협 |
| **제약 조건 (Constraints)** | 동역학 ($x_{k+1} = Ax + Bu$), 장애물 회피 | 선형 적분기(integrator) 예측 및 장애물 여유 거리 (`Obstacle_Margin`) |
| **장점 함수 (Merit Function)** | L1 잔차(residual) + $\log$ 장벽 페널티 | 물리적 예측 가능성, 제약 조건 준수 및 비용 최소화를 단일 스칼라 값으로 평가하는 "최종 심판"이며, 라인 서치의 브레이크 역할을 합니다. |
