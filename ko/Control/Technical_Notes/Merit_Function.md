# 장점 함수 평가 (Merit Function Evaluation)

!!! abstract "개요 (Overview)"
    `evaluate_merit(alpha, ...)`는 내점법(IPM)의 라인 서치(line-search)에서 사용되는 **스칼라 장점 함수(scalar merit function)** 를 평가합니다. 이는 추적 성능(tracking performance), 제어 노력(control effort), 제약 조건 만족(로그 장벽을 통해), 그리고 모델 충실도(model fidelity) 간의 균형을 맞춥니다.
    
    IPM은 이 값을 사용하여 백트래킹 라인 서치를 수행하는 동안 솔버가 뉴턴 방향(Newton direction, $\alpha$)을 따라 얼마나 멀리 이동할 수 있는지 결정합니다.

## :material-function: 장점 함수 구성 요소 (Merit Function Components)

장점 함수는 네 가지 주요 구성 요소의 가중합(weighted sum)입니다. 모든 페널티 및 장벽 항은 큰 상수(`L1_WEIGHT = 1000.0`)와 곱해져 라인 서치를 지배하게 되며, 이차(quadratic) 추적 비용은 일반적인 가중치를 가집니다.

<div class="grid cards" markdown>

- :material-target:
    **상태 편차 (State Deviation)**
    
    초기 상태로부터의 L1 페널티입니다.

- :material-cash:
    **단계 비용 (Stage Cost)**
    
    추적 및 제어 노력을 위한 이차 항(Quadratic terms)입니다.

- :material-wall:
    **장벽 페널티 (Barrier Penalty)**
    
    부등식 제약 조건에 대한 로그 장벽(Log-barrier)입니다.

- :material-compare:
    **동역학 일관성 (Dynamics Consistency)**
    
    예측과 실제 시뮬레이션 간의 페널티입니다.

</div>

## :material-math-integral: 전체 장점 수식 (Full Merit Expression)

!!! math "완전한 장점 함수 (Complete Merit Function)"
    $$
    \begin{aligned}
    \Phi(\alpha) =\;&
    \underbrace{\sum_{i=1}^{N_x}w_{\text{L1}}\bigl|\,x_{\text{curr},i}(\alpha)-x_{\text{init},i}\bigr|}_{\text{상태 편차 (State Deviation)}}\\
    &+\sum_{k=0}^{H-1}\Biggl[
    \underbrace{\frac{1}{2}\,x_{\text{curr},k}^\top Q_k x_{\text{curr},k}
    +\frac{1}{2}\,u_{\text{cand},k}^\top R_k u_{\text{cand},k}
    }_{\text{이차 비용 (Quadratic Cost)}}\\
    &\quad+\underbrace{\sum_{\text{constr}}\bigl(-\mu\,\log(s_{\text{cand}})+w_{\text{L1}}\bigl|c+s_{\text{cand}}\bigr|\bigr)}_{\text{장벽 항 (Barrier Terms)}}\\
    &\quad+\underbrace{w_{\text{L1}}\sum_{i=1}^{N_x}\bigl|(1-\alpha)d_k(i)\bigr|}_{\text{동역학 일관성 (Dynamics Consistency)}}
    \Biggr]
    \end{aligned}
    $$

| 변수 | 설명 |
| -------- | ----------- |
| $x_{\text{curr},k}(\alpha)$ | $x_{\text{pred},k} + \alpha\,\Delta x_k$ (후보 상태, Candidate state) |
| $u_{\text{cand},k}(\alpha)$ | $u_{\text{pred},k} + \alpha\,\Delta u_k$ (후보 제어, Candidate control) |
| $s_{\text{cand}}$ | $s + \alpha\,ds$ (각 제약 조건에 대한 후보 여유 변수, Candidate slack) |
| $\mu$ | 장벽 매개변수 (IPM 반복 동안 감소됨) |
| $w_{\text{L1}}$ | `L1_WEIGHT = 1000.0` |

## :material-cogs: 구현 세부 사항 (Implementation Details)

=== "1. 준비 (Preparation)"
    
    ```cpp
    matrix::StaticVector<double, Nx> x_curr = X_pred[0];
    x_curr.saxpy(alpha, riccati.dx[0]);
    
    for (std::size_t i = 0; i < Nx; ++i) { // (1)
        merit += L1_WEIGHT * std::abs(x_curr(i) - x_init(i));
    }
    ```
    
    1. **상태 편차 페널티**: $\text{pen}_{\text{init}}(\alpha)=\sum_{i=1}^{N_x} w_{\text{L1}}\,\bigl|\,x_{\text{curr},i}-x_{\text{init},i}\bigr|$ 를 평가합니다. 큰 가중치는 라인 서치가 궤적을 실제 초기 상태 `x_init`에 가깝게 유지하도록 강제합니다.

=== "2. 단계 비용 (Stage Cost)"

    ```cpp
    double err_d = x_curr(1) - config.target_d[k];
    double err_mu = x_curr(2);
    // ...
    merit += 0.5 * (config.Q_D * err_d * err_d + /*...*/); // (1)
    merit += 0.5 * (config.R_Steer * u_cand(0) * u_cand(0) + /*...*/); // (2)
    ```
    
    1. **상태 오차**: 기준 궤적으로부터의 편차(예: 횡방향 오프셋 $d$, 속도 $v$, 요 레이트 $r$)에 페널티를 줍니다.
    2. **제어 노력**: 조향 및 가속도 사용에 페널티를 줍니다.

=== "3. 장벽 항 (Barrier Terms)"

    ```cpp
    auto eval_barrier = [&](double c, double s, double ds) {
        double s_cand = s + alpha * ds;
        if (s_cand <= 1e-8) s_cand = 1e-8; // (1)
        return -current_mu * std::abs(s_cand) + L1_WEIGHT * std::abs(c + s_cand);
    };
    ```

    1. **수치적 안전성**: 여유 변수가 0이 되거나 음수가 되어 $\log(s)$가 정의되지 않는 상황을 방지합니다.

=== "4. 장애물 회피 (Obstacle Avoidance)"

    ```cpp
    double ds = x_curr(0) - obs_pred_s;
    if (std::abs(ds) > 20.0) continue; // (1)
    
    // ... calculate c_obs ...
    merit += eval_barrier(c_obs, duals[k].obs[i].s, duals[k].obs[i].ds);
    ```

    1. **최적화**: 종방향으로 너무 멀리 있는($> 20\text{m}$) 장애물에 대해서는 장벽 평가를 건너뜁니다.

=== "5. 동역학 일관성 (Dynamics Consistency)"

    ```cpp
    for (std::size_t i = 0; i < Nx; ++i) { // (1)
        merit += L1_WEIGHT * std::abs((1.0 - alpha) * riccati.d[k](i));
    }
    ```
    
    1. **아핀 잔차 (Affine Residual)**: $\text{pen}_{\text{dyn},k}(\alpha)= w_{\text{L1}}\;\bigl|(1-\alpha)\,d_k\bigr|$. 선형화된 동역학 예측을 위반하는 스텝을 밟는 것을 방지합니다.

## :material-shield-alert: 평가된 제약 조건 (Evaluated Constraints)

다음 제약 조건들은 로그 장벽 함수를 사용하여 평가됩니다. 각각에 대해 대응하는 `ds`는 쌍대 구조체(`duals[k].*`)에 의해 제공됩니다.

| 제약 조건 (Constraint) | 수식 `c` | 여유 변수 `s` & 업데이트 `ds` |
| :--- | :--- | :--- |
| **횡방향 상한** ($d \le d_{\max}$) | `x_curr(1) - config.d_max` | `duals[k].d_max` |
| **횡방향 하한** ($d \ge d_{\min}$) | `config.d_min - x_curr(1)` | `duals[k].d_min` |
| **입력 상한** ($u_{i} \le u_{\max, i}$) | `u_cand(i) - config.u_max[i]` | `duals[k].u_max[i]` |
| **입력 하한** ($u_{i} \ge u_{\min, i}$) | `config.u_min[i] - u_cand(i)` | `duals[k].u_min[i]` |
| **장애물 회피** | `safety_margin^2 - dist^2` | `duals[k].obs[i]` |

## :material-star: 설계 철학 (Design Philosophy)

!!! success "왜 이러한 특정한 형태인가? (Why this specific form?)"
    - **타당성 우선 (Feasibility First)**: 로그 장벽 및 L1 항이 지배적이어서 수락된 스텝이 모든 제약 조건을 유지하도록 보장합니다.
    - **목표 보존 (Objective Preservation)**: 추적 및 제어 노력을 위한 이차 항은 정상적으로 가중치가 부여되어 실행 가능한 영역 내에서 최적의 동작을 보장합니다.
    - **알고리즘 안정성 (Algorithmic Stability)**: 동역학 일관성 페널티 `(1 - alpha) * d_k`는 선형화된 모델의 유효한 영역 밖으로 너무 멀리 나가는 것을 방지합니다.
    - **계산 효율성 (Computational Efficiency)**: 장점 함수는 평가 비용이 극히 저렴하며(단계당 몇 번의 벡터-스칼라 연산뿐임), 이는 실시간 라인 서치에 필수적입니다.
    - **수치적 안전성 (Numerical Safety)**: 하드 클램프(`if (s_cand <= 1e-8)`) 및 공간적 컬링(`abs(ds) > 20`)은 오버플로우, 무한 장벽 및 불필요한 계산을 방지합니다.