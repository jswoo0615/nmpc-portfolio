# 시뮬레이션 수학적 기초 (Simulation Mathematical Foundations)

!!! abstract "개요 (Overview)"
    이 문서는 NMPC 프레임워크 내부에서 연속 시간(continuous-time) 물리 모델을 이산화(discretize)하는 데 사용되는 수치 적분(numerical integration) 기법을 간략하게 설명합니다.

---

## 1. 이산화 문제 (The Discretization Problem)

차량 또는 동적 시스템(플랜트, Plant)은 연속 시간 상미분 방정식(ODE, Ordinary Differential Equation)으로 정의됩니다:
$$ \dot{x}(t) = f(x(t), u(t)) $$

NMPC 및 이동 구간 추정(Moving Horizon Estimation)과 같은 디지털 제어 방식을 구현하려면, 이 연속 모델을 일정한 시간 간격 $\Delta t$로 정의된 일련의 스텝으로 이산화해야 합니다:
$$ x_{k+1} = F(x_k, u_k) $$

가장 간단한 접근법은 오일러 적분(Euler Integration) ($x_{k+1} = x_k + f(x_k, u_k) \Delta t$)이지만, 이는 막대한 절단 오차(truncation errors)를 누적시키고 Pacejka 타이어 모델과 같이 고도로 비선형적인 동역학을 다룰 때 불안정성을 초래합니다. 따라서 더 견고한 적분기가 필수적입니다.

---

## 2. Runge-Kutta 4차 (RK4) (Runge-Kutta 4th Order)

RK4 방법은 계산 복잡도와 정확도 사이의 이상적인 균형 덕분에 수치 적분의 표준으로 간주됩니다. 이는 네 가지 다른 기울기($\mathbf{k}_1, \dots, \mathbf{k}_4$)를 계산하고 그들의 가중 평균을 구함으로써 $\Delta t$ 간격에 걸친 궤적을 추정합니다.

### 2.1 기울기 공식화 (Slope Formulation)

현재 상태 $x_k$와 제어 입력 $u_k$가 주어졌을 때 (디지털 제어기의 영차 유지(Zero-Order Hold) 특성으로 인해 $\Delta t$ 간격 동안 일정하다고 가정함):

1. **첫 번째 기울기 ($\mathbf{k}_1$)**: 구간 시작점에서의 도함수.
   $$ \mathbf{k}_1 = f(x_k, u_k) $$

2. **두 번째 기울기 ($\mathbf{k}_2$)**: 구간 중간점에서의 도함수로, $\mathbf{k}_1$을 사용하여 평가됨.
   $$ \mathbf{k}_2 = f\left( x_k + \mathbf{k}_1 \frac{\Delta t}{2}, u_k \right) $$

3. **세 번째 기울기 ($\mathbf{k}_3$)**: 중간점에서의 정제된 도함수로, $\mathbf{k}_2$를 사용하여 평가됨.
   $$ \mathbf{k}_3 = f\left( x_k + \mathbf{k}_2 \frac{\Delta t}{2}, u_k \right) $$

4. **네 번째 기울기 ($\mathbf{k}_4$)**: 구간 끝점에서의 도함수로, $\mathbf{k}_3$을 사용하여 평가됨.
   $$ \mathbf{k}_4 = f(x_k + \mathbf{k}_3 \Delta t, u_k) $$

### 2.2 상태 업데이트 (State Update)

최종 예측된 상태 $x_{k+1}$은 네 가지 기울기를 결합하여 계산됩니다. 중간점 기울기($\mathbf{k}_2$ 및 $\mathbf{k}_3$)는 궤적 곡률의 가장 좋은 추정치를 나타내므로 두 배의 가중치가 부여됩니다:

$$ x_{k+1} = x_k + \frac{\Delta t}{6} \Big( \mathbf{k}_1 + 2\mathbf{k}_2 + 2\mathbf{k}_3 + \mathbf{k}_4 \Big) $$

!!! tip "오차 특성 (Error Characteristics)"
    RK4는 $\mathcal{O}(\Delta t^5)$의 국소 절단 오차(local truncation error)와 $\mathcal{O}(\Delta t^4)$의 전역 누적 오차(global accumulated error)를 제공합니다. 이는 최적화 구간(optimization horizon) 전체에 걸친 NMPC 예측이 실제 물리 엔진에 충실하게 가깝게 유지되도록 보장하여 제어 안정성을 확보합니다.
