# 파이썬 스크립트 모듈 (Python Scripts Module)

!!! abstract "개요 (Overview)"
    파이썬 스크립트 계층은 NMPC 생태계를 위한 운영 프론트엔드(front-end)를 구성합니다. 고도로 최적화된 C++ 수학적 코어를 CARLA 자율 주행 시뮬레이터에 연결하는 로직을 제공하여, 견고한 궤적 생성, 시나리오 분석 및 자동화된 스트레스 테스트를 용이하게 합니다.

## :material-shield-check: 설계 철학 (Design Philosophy)

1. **시뮬레이션 통합 (Simulation Integration)**: C++에서 최적화된 추상적인 물리 모델을 CARLA의 복잡한 3D 시뮬레이션 환경에 원활하게 연결하여, 센서 입력과 제어 출력이 물리적 현실과 일치하도록 보장합니다.
2. **견고한 테스트 및 검증 (Robust Testing & Validation)**: 자율 시스템은 광범위한 검증이 필요합니다. `nmpc_stress_test.py` 및 `analyze_stress_results.py`와 같은 스크립트는 엣지 케이스(edge cases, 예: 급격한 교차로)를 자동으로 찾아내고 제어기에 집중적인 테스트(bombard)를 가하여 수학적 안정성(KKT 수렴)과 물리적 안전성을 증명합니다.
3. **적응형 곡률 추적 (Adaptive Curvature Tracking)**: 상위 레벨 로컬 플래너(`reference_generator.py`)는 정적인 경로뿐만 아니라 동적 컨텍스트(차선 변경 및 좁은 코너 등)를 제공하여, NMPC 솔버가 즉석에서 동적으로 비용 행렬을 변경할 수 있도록 합니다.

## :material-file-tree: 핵심 스크립트 (Core Scripts)

- **`reference_generator.py`**: 경로 스무딩(path smoothing) 및 적응형 페널티 생성기입니다. 웨이포인트(waypoint) 불연속성 완화를 위해 라플라시안 스무딩(Laplacian smoothing)을 사용하며, 즉각적인 도로 곡률을 기반으로 적응형 Q 및 R 가중치를 솔버에 제공합니다.
- **`nmpc_core_planner.py`**: 메인 인터페이스 루프입니다. CARLA 위치 데이터(localization data)를 소비하고, C++ `SparseNMPCWrapper`를 호출하며, 폴백(fallback) 오버라이드를 처리하고, 조향/가속 명령을 다시 CARLA로 보냅니다.
- **`carla_route_inspector.py`**: 맵 스폰 지점(spawn points)을 검사하고, 토폴로지 구조 교차로를 분석하며, 사용자 지정 경로를 JSON으로 내보내는 Pygame 기반의 시각적 도구입니다.
- **`nmpc_carla_runner.py`**: 내보낸 특정 JSON 경로 또는 무작위 스폰 지점에서 NMPC 차량을 실행하고 성능 지표를 기록하는 헤드리스(headless) 실행 엔진입니다.
- **`nmpc_stress_test.py`**: CI/CD 스타일의 자동화된 테스트 제품군(suite)입니다. 맵에서 가장 위험한 교차로(예: 60° 이상의 회전)를 찾아 차량을 시뮬레이션하여 수렴 신뢰성을 테스트하고 실패 시나리오를 격리합니다.
- **`analyze_stress_results.py`**: 스트레스 테스트의 JSON 출력을 구문 분석(parse)하여 실패 원인(시간 초과, 충돌, 경로 이탈, KKT 솔버 붕괴)을 분류하고 종합적인 최종 보고서로 작성합니다.
