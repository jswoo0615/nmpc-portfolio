# 솔버 API 레퍼런스 (Solver API Reference)

!!! info "API 문서 (API Documentation)"
    이 문서는 최적화 알고리즘, 안전성 모니터링 및 수학적 풀이 엔진에 대한 API 표면을 제공합니다.

---

## :material-math-integral: 1. 리카티 솔버 (Riccati Solver)

### :material-cube-outline: `RiccatiSolver`
```cpp
template <std::size_t H, std::size_t Nx, std::size_t Nu>
class RiccatiSolver;
```
이산 시간 유한 구간(discrete-time finite-horizon) LQR 문제를 $\mathcal{O}(H)$로 푸는 핵심 엔진입니다. 사용자가 (또는 상위 레벨의 NMPC 엔진이) 솔버를 호출하기 전에 선형화된 동역학 배열(`A`, `B`, `d`)과 이차 비용 배열(`Q`, `R`, `q`, `r`)을 채울 것으로 기대합니다.

### 메서드: `solve`
```cpp
SolverStatus solve(double reg_u = 1e-6, double reg_x = 0.0);
```
후방(backward) 및 전방(forward) 패스를 실행합니다.
- **`reg_u`**: 양의 정부호성(positive definiteness)을 보장하기 위해 제어 헤시안(Hessian) $Q_{uu}$에 추가되는 Levenberg-Marquardt 정규화(regularization) 요소입니다.
- **반환값**: 인수분해(factorization)가 성공했는지 또는 특이점(singularity)이 발생했는지(`MATH_ERROR`)를 나타내는 `SolverStatus` 열거형을 반환합니다.

---

## :material-math-integral: 2. 탐색 및 수렴 (Search & Convergence)

### :material-cube-outline: `MeritLineSearch`
```cpp
class MeritLineSearch {
    static constexpr double BETA = 0.5;
    static constexpr double C1 = 1e-4;
    static constexpr int MAX_ITER = 6;
    
    template <typename Evaluator>
    [[nodiscard]] static inline double run(Evaluator evaluator, double current_merit, double directional_derivative = 0.0);
};
```
Armijo 백트래킹 라인 서치를 실행합니다. `evaluator` 람다(lambda)는 충분한 감소 조건(`C1`)이 충족될 때까지 제안된 뉴턴 스텝의 여러 비율(`alpha`)에 대해 비선형 물리 스텝을 시뮬레이션합니다.

### :material-cube-outline: `KKTMonitor`
```cpp
template <std::size_t N_vars, std::size_t N_cons>
class KKTMonitor {
    static KKT_Metrics evaluate_IPM(const matrix::StaticVector<double, N_vars>& grad_L_res,
                                    const matrix::StaticVector<double, N_cons>& primal_res,
                                    const matrix::StaticVector<double, N_ineq>& s_ineq,
                                    const matrix::StaticVector<double, N_ineq>& lambda_ineq,
                                    double target_mu);
};
```
벡터화된 SIMD (AVX2/NEON) 명령어를 사용하여 Karush-Kuhn-Tucker 잔차의 무한대 노름(infinity norms)을 추출합니다.

---

## :material-math-integral: 3. 견고성 및 폴백 (Robustness & Fallback)

### :material-cube-outline: `FallbackControl`
```cpp
template <typename T>
inline FallbackTriggerState<T> evaluateKKTAndFallback(const KKTMonitorParams<T>& params, const SolverKKTState<T>& state);

template <typename T>
inline ControlOutput<T> applyFallbackStrategy(const ControlOutput<T>& nmpc_optimal_u, 
                                              const FallbackTriggerState<T>& trigger_state,
                                              const T safe_deceleration_cmd,
                                              const T pure_pursuit_delta);
```
만약 `evaluateKKTAndFallback`이 막대한 여유 변수 위반이나 쌍대 변수 폭발을 감지하면, 안전 기동(`safe_deceleration_cmd` 및 `pure_pursuit_delta`)으로 `nmpc_optimal_u`를 무시(override)합니다.
