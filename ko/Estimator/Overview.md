# 상태 추정기 모듈 (Estimator Module)

!!! abstract "개요 (Overview)"
    이 모듈은 확장 칼만 필터(EKF, Extended Kalman Filter) 및 희소 이동 구간 추정기(Sparse MHE, Moving Horizon Estimator)를 포함하여 고도로 최적화된 상태 추정 알고리즘을 포함합니다. 이러한 추정기들은 임베디드 하드웨어의 한계를 극복하도록 명시적으로 설계되었으며, 동적 할당 오버헤드 제로(zero-allocation) 및 결정론적인(deterministic) 실행 시간을 보장합니다.

## :material-shield-check: 설계 철학 (Design Philosophy)

1. **동적 할당 제로 (Zero-Allocation)**: 추정 루프 중에는 어떠한 동적 메모리 할당(`new` 또는 `malloc`)도 발생하지 않습니다.
2. **ALU 최적화**: 모듈로(`%`) 연산을 피하고 조건부 선택(CSEL)을 활용하여 ARM 프로세서에서의 분기 예측 실패(branch prediction misses)를 방지하는 등 고급 최적화 기법을 적용했습니다.
3. **SIMD 및 레지스터 레벨 연산 (SIMD & Register Level Operations)**: MHE의 행렬 역산과 EKF의 공분산 전파는 $O(N^3)$ 연산을 피하기 위해 LDLT 분해 파이프라인 및 물리적 루프 언롤링(loop unrolling)으로 대체되었습니다.
4. **예측 가능성 (Predictability)**: 하드 실시간(hard real-time) 실행 한계를 보장하기 위해 최적화 반복 횟수에 엄격한 상한선(예: MHE의 경우 `MAX_ITER = 2`)을 적용합니다.

## :material-file-tree: 디렉토리 구조 (Directory Structure)

- `EKF.hpp`: 스칼라 역산을 피하고 Zero-Copy SIMD를 사용하는 초고속 확장 칼만 필터입니다.
- `HistoryBuffer.hpp`: 모듈로 연산자 없이 $O(1)$ SIMD 복사를 보장하는 순환 버퍼(circular buffer)입니다.
- `SparseMHE.hpp`: SIMD 최적화 및 LDLT 분해를 특징으로 하는 경량화된 희소 이동 구간 추정기입니다.
