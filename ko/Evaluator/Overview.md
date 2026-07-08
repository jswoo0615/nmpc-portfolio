# 평가기 모듈 (Evaluator Module)

!!! abstract "개요 (Overview)"
    이 모듈은 최적화 솔버(optimization solvers)에 필요한 평가 엔진(evaluation engines)을 제공합니다. 
    결정론적이고 메모리 안전성(memory-safe)을 갖춘 기준 경로(reference paths)를 생성하고, 비용(costs) 및 제약 조건(constraints)을 실시간으로 평가하는 데 중점을 둡니다.

## :material-shield-check: 설계 철학 (Design Philosophy)

1. **동적 할당 제로 (Zero-Allocation)**: 동적 메모리 할당(예: `new` 또는 `malloc`)을 완전히 제거하여 최적화 루프 내에서 결정론적인 실행 시간을 보장합니다. `std::array`와 같은 스택 할당(stack-allocated) 구조를 적극적으로 활용합니다.
2. **예측 가능한 복잡도 (Predictable Complexity)**: 스플라인 보간기(spline interpolator)와 같은 데이터 구조는 $\mathcal{O}(1)$ 또는 결정론적 복잡도 한계를 유지하기 위해 허용되는 최대 포인트 수(`MaxPoints`)를 미리 할당합니다.
3. **수치적 안전성 (Numerical Safety)**: 곡률(curvature)과 같은 모든 기하학적 평가기에는 수치적 한계에 도달했을 때 최적화가 발산(divergence)하는 것을 방지하기 위해 0으로 나누기 방지(zero-division guards)가 포함되어 있습니다.

## :material-file-tree: 핵심 구성요소 (Core Components)

- **`StaticCubicSpline1D`**: 오직 정적 메모리(static memory)에만 기반한 1D 3차 스플라인 보간기입니다.
- **`StaticCubicSpline2D`**: 누적 거리(Arc length)를 추적하는 2D 경로 생성기입니다.
