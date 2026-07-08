# 시뮬레이션 모듈 (Simulation Module)

!!! abstract "개요 (Overview)"
    `Simulation` 모듈은 이산 시간 단계(discrete time steps)에 걸쳐 물리 시스템(플랜트)의 연속 시간 상태(continuous-time states)를 시뮬레이션하고 예측하기 위한 수치 적분(numerical integration) 도구를 제공합니다. 연속 동역학 $ \dot{x} = f(x, u) $ 와 NMPC 및 추정기(Estimator) 모듈에서 요구하는 이산 공식화 간의 격차를 해소합니다.

## :material-shield-check: 설계 철학 (Design Philosophy)

1. **할당 제로 (Zero-Allocation)**: 적분 단계 중에 힙(heap) 할당이 발생하지 않습니다. 모든 상태 벡터와 중간 기울기 계산($\mathbf{k_1}, \dots, \mathbf{k_4}$)은 `StaticVector`를 사용하여 스택에 인스턴스화됩니다.
2. **결정론적 실행 (Deterministic Execution)**: 고정 스텝(fixed-step) 적분 알고리즘을 사용하면 예측 구간의 실행 시간이 엄격하게 일정하게 유지되며, 이는 실시간 최적 제어에 있어 필수적인 요구 사항입니다.
3. **템플릿 메타프로그래밍 (Template Metaprogramming)**: 함수 포인터나 `std::function` 래퍼를 사용하지 않고 동적 모델을 적분 루프에 직접 컴파일함으로써 높은 성능을 달성합니다.

## :material-file-tree: 핵심 구성 요소 (Core Components)

- **`Integrator.hpp`**: Runge-Kutta 4차(RK4) 적분 로직과 `IntegratorEngine` 인터페이스를 수용합니다.
