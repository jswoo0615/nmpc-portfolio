# 시뮬레이션 API 레퍼런스 (Simulation API Reference)

!!! info "API 문서 (API Documentation)"
    이 문서는 수치 적분(numerical integration) 모듈에 대한 API 시그니처를 간략히 설명합니다.

---

## :material-cube-outline: `step_rk4`

```cpp
template <size_t Nx, size_t Nu, typename Model, typename T>
inline matrix::StaticVector<T, Nx> step_rk4(const Model& model, 
                                            const matrix::StaticVector<T, Nx>& x, 
                                            const matrix::StaticVector<T, Nu>& u, 
                                            double dt_double);
```
고전적인 Runge-Kutta 4차(4th Order) 방법을 사용하여 `dt_double` 간격 동안 단일 이산 적분 단계를 수행합니다. 비선형 연속 모델을 4번 평가하여 미래 상태에 대한 매우 정확한 예측을 보장합니다.

모든 중간 벡터 배열(`k1`부터 `k4`까지)은 스택에 안전하게 인스턴스화됩니다.

---

## :material-cube-outline: `IntegratorEngine`

```cpp
template <size_t Nx, size_t Nu, typename Model, typename T>
struct IntegratorEngine;
```
차량 동역학 시뮬레이션을 위한 기본 인터페이스 역할을 하는 정적 래퍼(static wrapper)입니다. 구체적인 RK4 단계 로직을 통합된 `compute` 메서드 아래에 바인딩합니다.

### 메서드: `compute`
```cpp
static inline matrix::StaticVector<T, Nx> compute(const Model& model, 
                                                  const matrix::StaticVector<T, Nx>& x, 
                                                  const matrix::StaticVector<T, Nu>& u, 
                                                  double dt);
```
수치 적분을 실행합니다. 이 정적 메서드는 MHE 예측기, NMPC 동적 제약 조건 평가 및 독립형 시뮬레이션 루프에 의해 호출됩니다.
