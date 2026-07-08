# 행렬 모듈 (Matrix Module)

!!! abstract "개요 (Overview)"
    `Matrix` 모듈은 NMPC 프레임워크를 위한 가장 낮은 수준의 수학적 뼈대 역할을 합니다. 이는 Jetson Nano와 같은 임베디드 실시간 타겟에 배포될 때 일반화된 라이브러리(예: Eigen 또는 Armadillo)와 일반적으로 관련된 오버헤드, 메모리 단편화 및 지연 시간 문제를 우회하기 위해 완전히 맞춤 제작되었습니다.

## :material-shield-check: 설계 철학 (Design Philosophy)

1. **할당 제로 (스택 전용) (Zero-Allocation / Stack Only)**: 전체 라이브러리는 힙(heap) 할당을 엄격하게 피합니다. 메모리 레이아웃은 템플릿을 사용하여 컴파일 타임에 고정되어 `malloc` 지연 시간을 완전히 제거하고 결정론적인 최악 실행 시간(WCET, Worst-Case Execution Time)을 보장합니다.
2. **SIMD 및 레지스터 최적화 (SIMD & Register Optimization)**: 선형 대수, 자동 미분 및 메모리 전송에 걸친 연산은 64바이트 정렬된 메모리 버퍼(`alignas(64)`)와 함께 플랫폼별 명령어(ARM NEON / Intel AVX2)를 사용하여 수동으로 전개(unrolled)되거나 벡터화됩니다.
3. **무(無)전치 정책 (No-Transpose Policy)**: 행렬 전치(transpose)와 관련된 $O(N^2)$ 메모리 복사를 적극적으로 억제합니다. 대신 알고리즘은 가상으로 메모리를 열(column) 방향이나 행(row) 방향으로 탐색합니다.
4. **CUDA 준비 (CUDA Ready)**: 모든 핵심 데이터 구조와 기본 함수는 GPU 커널 내부에서의 투명한 실행을 지원하기 위해 `CUDA_CALLABLE`로 데코레이션됩니다.

## :material-file-tree: 디렉토리 구조 (Directory Structure)

- **`Core/`**: 기본적인 `StaticMatrix`, `StaticMatrixView` 및 `MathTraits`를 포함합니다. 이러한 클래스들은 SIMD 복사 연산자와 산술 기본 요소들을 구현합니다.
- **`AD/`**: 자동 미분(Automatic Differentiation) 엔진(`DualScalar`, `DualVec`)을 포함합니다. 이는 유한 차분법(finite differences)의 필요성을 제거하고 원시 변수(primal) 평가와 동시에 쌍대 수(dual numbers)를 통해 기울기(gradients)를 전파합니다.
- **`Linalg/`**: 선형 대수 처리 장치입니다. 네이티브 SIMD 수평 덧셈(horizontal addition)으로 작성된, 고도로 맞춤화된 제자리(in-place) 인수분해(Cholesky, LU, LDLT, QR) 기능을 갖추고 있습니다.
