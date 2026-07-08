# 파이썬 스크립트 API 레퍼런스 (Python Scripts API Reference)

!!! info "API 문서 (API Documentation)"
    이 문서는 CARLA에서 NMPC 시뮬레이션을 관리하는 파이썬 프론트엔드 스크립트의 주요 클래스와 실행 명령을 요약합니다.

---

## :material-language-python: 시뮬레이션 실행 (Simulation Execution)

### `nmpc_carla_runner.py`
단일 시뮬레이션 인스턴스를 실행합니다. 자아 차량(ego vehicle)을 `LocalPlanner`에 연결하고 목적지까지 주행시킵니다.
```bash
# 내보낸 특정 경로 시나리오 실행
python nmpc_carla_runner.py --scenario scenarios/Town04_Straight.json

# 특정 스폰 지점과 목표 속도로 실행
python nmpc_carla_runner.py --map Town04 --start 10 --end 45 --speed 80.0
```

### `carla_route_inspector.py`
시작 및 종료 스폰 지점을 대화형으로 선택할 수 있는 Pygame GUI 도구입니다. 경로 메타데이터(거리, 교차로 복잡도)를 생성하고 러너(runner)를 위해 JSON으로 내보냅니다.
```bash
python carla_route_inspector.py
# 조작 (Controls):
# 좌클릭: 웨이포인트 선택
# 스페이스: JSON으로 내보내기
```

---

## :material-language-python: 플래너 및 생성기 (Planners & Generators)

### `LocalPlanner` (`nmpc_core_planner.py`)
CARLA 상태 데이터와 C++ 래퍼를 연결하는 메인 인터페이스 클래스입니다.
- **`run_step(debug=True)`**: 시뮬레이션 틱(tick)마다 호출됩니다. 현재 Frenet 좌표를 계산하고, 초기 웨이포인트 소비를 처리하며, 목표 속도를 동적으로 변경(램핑, ramping)하고, NMPC 솔버를 호출하며, 피치(pitch)에 대한 중력 보상을 적용하고, `carla.VehicleControl`을 반환합니다.

### `ReferenceGenerator` (`reference_generator.py`)
부드러운 경로와 적응형 최적화 가중치를 생성합니다.
- **`generate_reference(ego_state, waypoint_buffer, vx)`**: 전방의 누적 곡률을 계산합니다. 부드러워진 웨이포인트(라플라시안 스무딩을 통해)와 `adaptive_opt` 딕셔너리(유연한 차선 변경이나 엄격한 코너링을 허용하기 위해 동적으로 수정된 `Q_D`, `Q_kappa_track`, `R_SteerRate` 등을 포함)를 반환합니다.

---

## :material-language-python: 검증 및 테스트 (Validation & Testing)

### `nmpc_stress_test.py`
복잡한 교차로를 자동으로 찾고 NMPC 솔버의 견고성을 테스트합니다.
```bash
# 곡률이 높은 경로에 초점을 맞춘 50개의 자동화된 테스트 실행
python nmpc_stress_test.py --map Town04 --runs 50 --min-angle 60.0 --output results.json
```

### `analyze_stress_results.py`
스트레스 테스트에서 생성된 결과를 분석합니다. 차량이 실패할 경우 '사망 진단서(Death Certificate)'를 분류합니다.
```bash
# 단일 세션에 대한 보고서 생성
python analyze_stress_results.py results/session_2026/session_summary.json

# 최적화 전후 비교 (Before & After)
python analyze_stress_results.py results/session_old/session_summary.json results/session_new/session_summary.json
```
