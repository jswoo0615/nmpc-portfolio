# 솔버 수학적 기초 (Solver Mathematical Foundations)

!!! abstract "개요 (Overview)"
    이 문서는 NMPC 뉴턴 스텝에서 도출된 LQR(Linear Quadratic Regulator) 하위 문제를 해결하는 데 사용되는 핵심 최적화 알고리즘과 수렴의 수학적 검증을 간략하게 설명합니다.

---

## 1. 이산 시간 리카티 재귀 (Discrete-time Riccati Recursion)

예측 구간 $H$에 걸쳐 희소 등식 제약 조건이 있는 이차 계획법(QP, Quadratic Programming) 문제를 효율적으로 풀기 위해 NMPC는 리카티 방정식(Riccati Equation)을 통한 **동적 프로그래밍(Dynamic Programming)** 을 활용합니다. 이는 밀집 역행렬(dense inversion)에 필요한 $\mathcal{O}(H^3)$의 시간 복잡도를 $\mathcal{O}(H)$로 줄여줍니다.

### 1.1 문제 공식화 (Problem Formulation)

선형화된 시스템 동역학(linearized system dynamics)이 다음과 같이 주어졌을 때:
$$ \delta x_{k+1} = A_k \delta x_k + B_k \delta u_k + d_k $$

이차 비용 함수 근사치(quadratic cost function approximation)는 다음과 같습니다:
$$ J = \frac{1}{2} \delta x_H^T Q_H \delta x_H + q_H^T \delta x_H + \sum_{k=0}^{H-1} \left( \frac{1}{2} \delta x_k^T Q_k \delta x_k + q_k^T \delta x_k + \frac{1}{2} \delta u_k^T R_k \delta u_k + r_k^T \delta u_k \right) $$

$k$ 단계에서의 최적 가치 함수(Optimal Value Function, Cost-to-Go)를 정의합니다:
$$ V_k(\delta x) = \frac{1}{2} \delta x^T P_k \delta x + p_k^T \delta x $$

### 1.2 후방 패스 (가치 반복) (Backward Pass / Value Iteration)

종료 상태인 $P_H = Q_H$와 $p_H = q_H$부터 시작하여 거꾸로 재귀하여 ($k = H-1 \dots 0$) 최적 피드백 정책을 계산합니다:

1. **제어 헤시안 및 기울기 (Control Hessian & Gradient)**:
   $$ Q_{uu} = R_k + B_k^T P_{k+1} B_k + \lambda_{LM} I $$
   $$ Q_{ux} = B_k^T P_{k+1} A_k $$
   $$ q_u = r_k + B_k^T (p_{k+1} + P_{k+1} d_k) $$

   *(참고: $\lambda_{LM}$은 양의 정부호성을 보장하기 위한 Levenberg-Marquardt 정규화 계수입니다).*

2. **최적 정책 게인 (Optimal Policy Gains)**:
   $Q_{uu}$에 대해 LDLT 인수분해를 사용하여 피드백 게인($K_k$)과 피드포워드 게인($k_{ff, k}$)을 풉니다:
   $$ K_k = -Q_{uu}^{-1} Q_{ux} $$
   $$ k_{ff, k} = -Q_{uu}^{-1} q_u $$

3. **가치 함수 업데이트 (Value Function Update)**:
   $$ P_k = Q_k + A_k^T P_{k+1} A_k + K_k^T Q_{ux} $$
   $$ p_k = q_k + A_k^T (p_{k+1} + P_{k+1} d_k) + Q_{ux}^T k_{ff, k} $$

### 1.3 전방 패스 (궤적 시뮬레이션) (Forward Pass / Trajectory Simulation)

게인이 계산되면 초기 상태 $\delta x_0 = 0$부터 최적 궤적을 전방으로 시뮬레이션합니다:

$$ \delta u_k = K_k \delta x_k + k_{ff, k} $$
$$ \delta x_{k+1} = A_k \delta x_k + B_k \delta u_k + d_k $$

---

## 2. KKT 수렴 지표 (KKT Convergence Metrics)

비선형 최적화가 성공적으로 수렴했는지 확인하기 위해 `KKTMonitor`는 4가지 Karush-Kuhn-Tucker 조건을 확인합니다. 모든 $L_\infty$ 노름(norms)이 `TOLERANCE` $\tau$ (예: $10^{-6}$)보다 작으면 해가 최적(optimal)인 것으로 간주됩니다.

1. **정류성 (Stationarity / 라그랑지안의 기울기가 0임)**:
   $$ \| \nabla L(x, u, \lambda, \mu) \|_\infty \le \tau $$

2. **원시 타당성 (Primal Feasibility / 제약 조건이 만족됨)**:
   $$ \| x_{k+1} - f(x_k, u_k) \|_\infty \le \tau $$

3. **쌍대 타당성 (Dual Feasibility / 부등식 승수가 음수가 아님)**:
   $$ \max(0, -\mu) \le \tau $$

4. **상보적 여유성 (Complementary Slackness / 활성 제약 조건 정렬)**:
   내점법(Interior Point Method)에서는 이것이 장벽 매개변수 $\mu_{target}$에 의해 수정됩니다:
   $$ \| S \mu - \mu_{target} \mathbf{1} \|_\infty \le \tau $$
   *(여기서 $S$는 여유 변수들의 대각 행렬입니다).*
