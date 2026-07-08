# 파이썬 API 모듈 (래퍼) (Python API Module - Wrapper)

!!! abstract "개요 (Overview)"
    `src/nmpc_wrapper.cpp`는 고도로 최적화된 C++ NMPC 코어를 파이썬 환경에 노출시키는 PyBind11 래퍼(wrapper)입니다. 시뮬레이션, 머신 러닝(예: 강화 학습 에이전트) 및 상위 레벨 파이썬 플래너가 실시간 C++ 솔버에 접근할 수 있도록 하는 가교 역할을 합니다.

## :material-shield-check: 설계 철학 (Design Philosophy)

1. **제로 카피 배열 (Zero-Copy Arrays)**: 래퍼는 원시 포인터(`py::buffer_info`)를 사용하여 numpy 배열을 C++ 메모리 공간으로 직접 전달하므로, 비용이 많이 드는 메모리 복사를 피합니다.
2. **상태 투영 처리 (State Projection Handling)**: 전역 데카르트 상태(x, y, yaw)를 NMPC에서 요구하는 Frenet 프레임(s, d, $\mu$)으로 변환하는 과정을 관리합니다. 전역 탐색 초기화를 위한 견고한 "콜드 스타트(Cold Start)" 감지 메커니즘을 특징으로 합니다.
3. **투명한 상태 보고 (Transparent Status Reporting)**: 최적 제어 입력뿐만 아니라 KKT 진단 지표(여유 변수, 쌍대 변수, 반복 횟수)를 `py::dict`를 통해 파이썬으로 다시 내보내어 지속적인 모니터링을 가능하게 합니다.

## :material-file-tree: 핵심 구성 요소 (Core Components)

- **`SparseNMPCWrapper`**: NMPC 솔버, 튜닝 구성(tuning configurations), 3차 스플라인 평가기(cubic spline evaluator), 그리고 동적 환경 평가기를 캡슐화하는 핵심 PyBind11 클래스입니다.
