# 유틸리티 모듈 (Utils Module)

!!! abstract "개요 (Overview)"
    `Utils` 모듈은 NMPC 인프라를 지원하는 보조 구성 요소와 매크로를 포함합니다. 환경 평가 로직과 플랫폼별 컴파일 지시어(directives)를 제공합니다.

## :material-shield-check: 설계 철학 (Design Philosophy)

1. **플랫폼 이식성 (Platform Portability)**: 추상화 매크로(`CUDAMacros.hpp` 등)를 사용하여 코드베이스는 핵심 로직을 수정하지 않고도 동일한 수학 엔진을 표준 CPU 실행 및 GPU 가속 환경 모두에 대해 컴파일할 수 있도록 보장합니다.
2. **환경 통합 (Environment Integration)**: 이 모듈은 최적화 솔버가 쉽게 사용할 수 있는 형식으로 동적 장애물과 같은 외부 제약 조건을 평가하고 모델링하는 로직을 캡슐화합니다.

## :material-file-tree: 핵심 구성 요소 (Core Components)

- **`CUDAMacros.hpp`**: GPU를 타겟팅할 때 NVCC 컴파일을 위해 함수에 태그를 지정하는 `CUDA_CALLABLE` 매크로를 정의합니다.
- **`EnvironmentEvaluator.hpp`**: Frenet 프레임 내에서 움직이는 장애물을 모델링하고 평가하여 NMPC 솔버에 제약 조건 값과 자코비안을 제공하는 로직을 포함합니다.
