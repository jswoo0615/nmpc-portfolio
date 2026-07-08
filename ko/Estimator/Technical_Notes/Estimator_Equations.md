# 상태 추정 수학적 기반 (Estimator Mathematical Foundations)

!!! abstract "개요 (Overview)"
    이 문서는 `Estimator` 모듈의 기반이 되는 수학적 공식, 특히 확장 칼만 필터(EKF, Extended Kalman Filter) 및 희소 이동 구간 추정기(Sparse MHE, Moving Horizon Estimator)에 대해 설명합니다.

---

## 1. 확장 칼만 필터 (Extended Kalman Filter, EKF)

확장 칼만 필터는 비선형 동적 시스템에 대한 재귀적 상태 추정을 제공합니다. 시스템 및 측정 모델은 다음과 같이 정의됩니다:

$$
\begin{aligned}
x_k &= f(x_{k-1}, u_k) + w_k, \quad w_k \sim \mathcal{N}(0, Q) \\
z_k &= h(x_k) + v_k, \quad v_k \sim \mathcal{N}(0, R)
\end{aligned}
$$

여기서 $x_k \in \mathbb{R}^{N_x}$는 상태, $u_k \in \mathbb{R}^{N_u}$는 제어 입력, $z_k \in \mathbb{R}^{N_z}$는 측정값입니다. 우리의 아키텍처에서는 전체 상태를 직접 측정할 수 있으므로 $h(x_k) = x_k$입니다.

### 1.1 예측 단계 (Time Update / Prediction Step)

상태 추정치 및 공분산이 시간에 따라 앞으로 전파됩니다:

$$
\begin{aligned}
\hat{x}_{k|k-1} &= f(\hat{x}_{k-1|k-1}, u_k) \\
P_{k|k-1} &= F_k P_{k-1|k-1} F_k^T + Q
\end{aligned}
$$

여기서 자코비안 행렬 $F_k$는 자동 미분(Automatic Differentiation, AD)을 사용하여 정확하게 계산됩니다:

$$
F_k = \left. \frac{\partial f(x, u)}{\partial x} \right|_{x=\hat{x}_{k-1|k-1}, u=u_k}
$$

!!! tip "아키텍트의 노트: SIMD SAXPY 최적화 (Architect's Note)"
    `EKF.hpp`에서 $F P F^T$를 단순하게(naive) 계산하려면 임시 전치 행렬 $F^T$를 메모리에 할당해야 합니다. 이러한 오버헤드를 제거하기 위해 엔진은 **Zero-Copy SIMD SAXPY** 연산을 수행하여 스칼라 루프를 물리적으로 언롤링함으로써 $P_{k|k-1}$의 요소를 계산합니다.

### 1.2 업데이트 단계 (Measurement Update / Update Step)

새로운 측정값 $z_k$가 도착하면 이노베이션(innovation, 잔차)을 계산합니다. 요(yaw) 각도 잔차는 $[-\pi, \pi]$로 정규화됩니다:

$$
y_k = z_k - \hat{x}_{k|k-1}
$$

이노베이션 공분산 $S_k$와 칼만 게인(Kalman Gain) $K_k$를 평가합니다:

$$
\begin{aligned}
S_k &= P_{k|k-1} + R \\
K_k &= P_{k|k-1} S_k^{-1}
\end{aligned}
$$

마지막으로 상태와 공분산을 업데이트합니다:

$$
\begin{aligned}
\hat{x}_{k|k} &= \hat{x}_{k|k-1} + K_k y_k \\
P_{k|k} &= (I - K_k) P_{k|k-1}
\end{aligned}
$$

수치적 안정성을 보존하기 위해 $P_{k|k}$는 명시적으로 대칭이 되도록 강제됩니다:

$$
P_{k|k} \leftarrow \frac{1}{2} (P_{k|k} + P_{k|k}^T)
$$

---

## 2. 희소 이동 구간 추정기 (Sparse Moving Horizon Estimator, MHE)

가장 최근의 측정값만 엄격하게 사용하는 EKF와 달리, MHE는 이동하는 윈도우(horizon) $HE$에 걸친 상태 추정을 최적화 문제로 공식화합니다. 이는 심한 비선형성과 경계 제약 조건을 훨씬 더 잘 처리합니다.

### 2.1 MHE 비용 함수 (MHE Cost Function)

목표는 다음 비용 함수(cost functional)를 최소화하는 상태 시퀀스 $X = \{ x_0, x_1, \dots, x_{HE} \}$를 찾는 것입니다:

$$
\min_{X} J(X) = J_{\text{arrival}} + J_{\text{meas}} + J_{\text{dyn}}
$$

여기서:
1. **도달 비용 (Arrival Cost)**: 구간 시작 시의 이전 상태 추정치 $\bar{x}_0$ 로부터의 편차에 페널티를 부여합니다.
   $$ J_{\text{arrival}} = \frac{1}{2} \left\| W_{\text{arr}} (x_0 - \bar{x}_0) \right\|_2^2 $$

2. **측정 비용 (Measurement Cost)**: 추정된 상태와 실제 센서 측정값 $z_k$ 간의 편차에 페널티를 부여합니다.
   $$ J_{\text{meas}} = \frac{1}{2} \sum_{k=1}^{HE} \left\| W_{\text{meas}} (z_k - x_k) \right\|_2^2 $$

3. **동적 타당성 비용 (Dynamic Feasibility Cost / Multiple Shooting)**: 차량 동역학 $f(x, u)$ 위반에 페널티를 부여합니다.
   $$ J_{\text{dyn}} = \frac{1}{2} \sum_{k=0}^{HE-1} \left\| W_{\text{dyn}} \Big( x_{k+1} - f(x_k, u_k) \Big) \right\|_2^2 $$

### 2.2 가우스-뉴턴 공식화 (Gauss-Newton Formulation)

실시간 한계 내에서 이 비선형 최소제곱 문제를 효율적으로 풀기 위해 가우스-뉴턴(Gauss-Newton) 스텝을 적용합니다. 동적 잔차(dynamic residual) $r_k$를 정의합니다:

$$
r_k = x_{k+1} - f(x_k, u_k)
$$

시스템 자코비안 $J_k = \frac{\partial f}{\partial x_k}$를 사용하여 비용 함수를 선형화합니다. 결과적인 헤시안(Hessian) $H$와 그래디언트(gradient) $g$는 블록 단위로 조립됩니다:

#### 그래디언트 누적 (Gradient Accumulation)
도달 노드 $x_0$에 대해:
$$ g_0 \mathrel{+}= W_{\text{arr}}^2 (x_0 - \bar{x}_0) $$

측정 노드들에 대해:
$$ g_k \mathrel{-}= W_{\text{meas}}^2 (z_k - x_k) $$

동적 시퀀스의 경우 잔차를 전개하면 노드 $x_k$와 $x_{k+1}$에 대한 기여가 생성됩니다:
$$ 
\begin{aligned}
g_{k+1} &\mathrel{+}= W_{\text{dyn}}^2 r_k \\
g_k &\mathrel{-}= J_k^T W_{\text{dyn}}^2 r_k
\end{aligned}
$$

#### 헤시안 누적 (Hessian Accumulation)
헤시안은 극도로 희소(블록 삼중 대각, Block Tridiagonal)합니다.
- **주대각 블록 (Main Diagonal Blocks)**:
  $$
  \begin{aligned}
  H_{0,0} &\mathrel{+}= W_{\text{arr}}^2 \\
  H_{k,k} &\mathrel{+}= W_{\text{meas}}^2 \\
  H_{k+1,k+1} &\mathrel{+}= W_{\text{dyn}}^2 \\
  H_{k,k} &\mathrel{+}= J_k^T W_{\text{dyn}}^2 J_k
  \end{aligned}
  $$

- **비대각 블록 / 교차 항 (Off-Diagonal Blocks / Cross Terms)**:
  $$
  \begin{aligned}
  H_{k+1,k} &\mathrel{-}= W_{\text{dyn}}^2 J_k \\
  H_{k,k+1} &\mathrel{-}= J_k^T W_{\text{dyn}}^2
  \end{aligned}
  $$

### 2.3 LDLT 분해 및 원시 업데이트 (LDLT Factorization and Primal Update)

$H$와 $g$가 완전히 조립되면 최소한의 Tikhonov 정규화(regularization)가 적용됩니다:
$$ H \leftarrow H + \lambda I \quad (\lambda = 10^{-3}) $$

선형 시스템을 풀어서 하강 방향(descent direction) $\Delta X$를 계산합니다:
$$ H \Delta X = -g $$

!!! tip "아키텍트의 노트: 행렬 역산 우회 (Architect's Note)"
    $O(N^3)$ 가우스-조던(Gauss-Jordan) 행렬 역산은 결정론적 실행에 매우 해롭습니다. `SparseMHE.hpp`는 인라인 **LDLT 분해(Decomposition) 솔버**를 활용하여 파이프라인을 보호하고 특이점(singularities)을 완전히 피합니다.

그런 다음 수치적 발산을 방지하기 위해 제한된 스텝 크기(clamped step size)로 상태를 업데이트합니다:
$$ x_k \leftarrow x_k + \text{clamp}(\Delta x_k, -5.0, 5.0) $$
