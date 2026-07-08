# 차량 동역학 모듈 (Vehicle Dynamics Module)

!!! abstract "개요 (Overview)"
    이 모듈은 실시간 NMPC 솔버와 고정밀 시뮬레이터 모두에서 사용되는 물리적 차량 모델과 수학적 유틸리티를 포함합니다.

## :material-shield-check: 설계 철학 (Design Philosophy)

동역학 모듈은 물리적인 운동 방정식과 수치 해석 솔버의 로직을 엄격하게 분리합니다. C++ 템플릿을 적극적으로 활용함으로써, 단일한 물리 방정식 세트를 일반 스칼라 연산(순방향 시뮬레이션용)과 자동 미분(NMPC의 정확한 자코비안 및 헤시안 생성용) 모두에 대해 컴파일할 수 있습니다. 모든 함수는 향후 이기종(GPU/CPU) 컴퓨팅 아키텍처를 지원하기 위해 `CUDA_CALLABLE`로 표시되어 있습니다.

## :material-layers: 핵심 구성요소 (Core Components)

### 1. `VehicleParams.hpp`
차량 파라미터를 위한 핵심적이고 엄격하게 타입이 지정된 데이터 구조를 정의합니다.

- **`VehicleDynamicsParams`**: 기본적인 차량 속성 (질량, 요 관성모멘트, 치수, 무게중심 높이 등).
- **`TireLoadParams`**: 하중에 따른 마찰력 감소(friction decay)를 포함하는 Pacejka Magic Formula 타이어 계수.
- **`SuspensionParams`**: 고정밀 시뮬레이션을 위한 서스펜션 기하학 파라미터 (롤 센터 높이, 캠버 게인, 강성 등).

### 2. `FastMath.hpp`
실시간 실행에 맞춰 최적화된 고성능 수학 유틸리티 모듈입니다.

- `sin` 및 `atan` 근사를 위해 고도로 최적화되고 캐시 친화적인 룩업 테이블(LUT) 기반의 `FastTrig` 클래스를 구현합니다.
- C++ 템플릿 특수화를 사용하여 수학 연산을 스마트하게 라우팅합니다:
  - 표준 `double` 평가는 빠른 LUT 근사를 사용하여 실시간 순방향 적분 속도를 크게 향상시킵니다.
  - `Dual` 및 `DualVec` 자동 미분 타입은 안전하게 표준 라이브러리나 AD 전용 수학 구현으로 폴백(fallback)하여, 솔버의 미분 체인이 100% 정확하게 유지되고 LUT 양자화 오류에 의해 손상되지 않도록 보장합니다.

### 3. `VehiclePhysicsCore.hpp`
상태 공간 상미분 방정식(ODE)과 분리된 기초적인 물리 계산을 포함합니다.

- **하중 이동 (Load Transfer)**: 실시간 NMPC를 위한 계산량이 적은 준정적(quasi-static) 하중 이동과 서스펜션 기하학적 하중 이동(피치, 롤 및 롤 센터 높이 고려)을 모두 구현합니다.
- **타이어 물리 (Tire Physics)**: 완전한 비선형, 하중 의존적인 Pacejka Magic Formula를 사용하여 횡력을 계산합니다. 섀시 롤에 의해 구동되는 동적 캠버 추력(camber thrust) 계산을 포함합니다.

### 4. `RealTimeDynamicsModel.hpp`
실시간 NMPC 솔버를 위해 세심하게 최적화된 8-상태 Frenet 프레임 차량 모델입니다.

- SQP 루프 내의 계산 오버헤드를 최소화하기 위해 준정적 하중 이동을 사용합니다.
- 고속에서의 과도 응답(transient) 안정성 한계를 포착하는 데 필수적인 1차 이완 길이(relaxation length) ODE를 사용하여 타이어 힘 생성 지연을 모델링합니다.
- `AD` 모듈과 완벽하게 호환되어, NMPC 솔버가 RK4 적분기를 통해 정확한 닫힌 형태(closed-form)의 자코비안을 자동으로 추출할 수 있게 합니다.

### 5. `HighFidelityDynamicsModel.hpp`
견고한 시뮬레이션 및 알고리즘 검증(플랜트 모델)을 위해 설계된 포괄적인 8-상태 Frenet 프레임 모델입니다.

- 완전한 4륜 독립 하중 이동, 서스펜션 롤/피치 기하학, 공기 역학적 항력 및 동적 캠버 추력을 통합합니다.
- 전용 `extractJacobians` 함수를 포함합니다. `Dual` 타입을 수동으로 인스턴스화함으로써, 엔지니어가 연속 시간(continuous-time) $A$ 및 $B$ 상태 공간 행렬을 직접 추출할 수 있도록 합니다. 이는 고전적인 선형 안정성 분석, 고유값(eigenvalue) 검사 또는 LQR 벤치마킹에 매우 유용합니다.
