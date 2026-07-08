# 솔버 모듈 (Solver Module)

!!! abstract "개요 (Overview)"
    `Solver` 모듈은 NMPC의 제약이 있는 최적화 문제(constrained optimization problem)가 해결되는 계산의 핵심입니다. 여기에는 최적의 제어 입력 시퀀스를 찾는 수치 알고리즘(예: 리카티 재귀, Riccati Recursion)이 포함되어 있습니다.

## :material-shield-check: 설계 철학 (Design Philosophy)

1. **결정론적 풀이 (Deterministic Resolution)**: 최적의 제어 시퀀스는 엄격한 실시간 데드라인(예: 차량의 경우 < 10ms) 내에 계산되어야 합니다. 여기에 있는 모든 알고리즘은 엄격하게 제한되어 있습니다(예: 최대 반복 횟수 제한, 라인 서치 제한).
2. **동적 할당 제로 데이터 경로 (Zero-Allocation Data Paths)**: 리카티 재귀는 `include/Optimization/Matrix/Core`에 정의된 스택 메모리 구조만을 통해 궤적을 앞뒤로 완전히 처리합니다.
3. **안전 및 폴백 우선 (Safety & Fallback First)**: 최적 제어 이론은 이론적인 보장을 제공하지만, 현실 세계의 물리는 이를 깨뜨립니다. 솔버에는 수치적 붕괴(numerical collapse)나 실행 불가능(infeasibility) 상태를 감지했을 때 즉각적으로 페일 세이프 로직(fail-safe logic, 예: 비상 제동)을 주입하는 `KKTMonitor`와 `FallbackControl`이 통합되어 있습니다.

## :material-file-tree: 핵심 구성 요소 (Core Components)

- **`RiccatiSolver.hpp`**: 뉴턴 스텝에서 LQR(Linear Quadratic Regulator) 하위 문제(subproblem)를 푸는 $\mathcal{O}(N)$ 동적 프로그래밍 엔진입니다.
- **`KKTMonitor.hpp`**: SIMD 가속을 사용하여 Karush-Kuhn-Tucker 잔차의 무한대 노름(infinity norms)을 계산하여 수렴을 결정하거나 쌍대 변수의 폭발을 감지합니다.
- **`MeritLineSearch.hpp`**: 고도로 비선형적인 영역에서 뉴턴 스텝이 목표를 지나치게 뛰어넘는 것(overshooting)을 방지하는 Armijo 백트래킹 라인 서치입니다.
- **`FallbackControl.hpp`**: 실행 불가능한 `SolverStatus` 신호를 포착하여 제어 출력을 무시(override)하고 최종적인 안전망 역할을 합니다.
- **`SolverStatus.hpp`**: 절대적인 상태 머신 열거형(enumerations)을 정의합니다 (예: `SUCCESS`, `MATH_ERROR`, `INFEASIBLE`).
