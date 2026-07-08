# 평가기 API 레퍼런스 (Evaluator API Reference)

!!! info "API 문서 (API Documentation)"
    이 문서는 기준 경로 평가 및 생성 모듈을 위한 API 시그니처를 간략히 설명합니다.

## :material-cube-outline: `StaticCubicSpline1D`

```cpp
template <std::size_t MaxPoints>
class StaticCubicSpline1D;
```
단일 변수에 대한 3차 스플라인(cubic spline) 계수 계산 및 보간(interpolation)을 수행하는 클래스입니다.

=== "주요 멤버 변수 (Key Member Variables)"
    - `std::array<double, MaxPoints> a, b, c, d`: 스플라인 다항식 각 항의 계수.
    - `std::array<double, MaxPoints> x`: 기준점들의 독립 변수 배열.
    - `std::size_t num_points`: 실제 유효한 데이터 포인트의 개수.

=== "멤버 함수 (Member Functions)"
    | 함수명 | 시그니처 | 설명 |
    | :--- | :--- | :--- |
    | `build` | `void build(const std::array<double, MaxPoints>& x_in, const std::array<double, MaxPoints>& y_in, std::size_t n)` | 입력된 포인트들을 바탕으로 선형 방정식을 풀어 스플라인 계수를 계산합니다. 유효성을 위해 최소 3개의 포인트가 필요합니다. |
    | `calc` | `double calc(double t) const` | 지정된 지점 $t$에서 보간된 스플라인 값 $S(t)$를 반환합니다. |
    | `calc_d1` | `double calc_d1(double t) const` | 지정된 지점 $t$에서 1차 미분값 $S'(t)$를 반환합니다. |
    | `calc_d2` | `double calc_d2(double t) const` | 지정된 지점 $t$에서 2차 미분값 $S''(t)$를 반환합니다. |
    | `search_index` | `std::size_t search_index(double t) const` | (`private`) 이진 탐색(binary search)을 사용하여 입력 $t$가 속한 다항식 구간의 인덱스를 도출합니다. 탐색 복잡도는 $\mathcal{O}(\log n)$입니다. |

---

## :material-cube-outline: `StaticCubicSpline2D`

```cpp
template <std::size_t MaxPoints>
class StaticCubicSpline2D;
```
누적 거리 $s$를 파라미터로 사용하는 2D 경로 스플라인 보간기입니다. 차량 동역학 기반 제어에서 궤적 추종(trajectory tracking)을 위한 연속적인 기준 상태(reference state)를 생성하는 데 사용됩니다.

=== "주요 멤버 변수 (Key Member Variables)"
    - `StaticCubicSpline1D<MaxPoints> sx, sy`: 파라미터 $s$에 대해 X 및 Y 좌표를 보간하는 내부 1D 스플라인 객체.
    - `std::array<double, MaxPoints> s`: 계산된 누적 거리(Arc Length) 배열.
    - `std::size_t num_points`: 실제 유효한 데이터 포인트의 개수.

=== "멤버 함수 (Member Functions)"
    | 함수명 | 시그니처 | 설명 |
    | :--- | :--- | :--- |
    | `build` | `void build(const std::array<double, MaxPoints>& x_in, const std::array<double, MaxPoints>& y_in, std::size_t n)` | 인접한 포인트 간의 유클리드 거리를 누적하여 파라미터 $s$ 배열을 생성하고, 이를 바탕으로 `sx` 및 `sy` 스플라인을 초기화합니다. |
    | `get_max_s` | `double get_max_s() const` | 생성된 궤적의 총 누적 길이(최대 $s$ 값)를 반환합니다. 종료 조건을 결정하는 데 사용됩니다. |
    | `calc_x` | `double calc_x(double t) const` | 누적 거리 $t$에서 보간된 X 좌표를 반환합니다. |
    | `calc_y` | `double calc_y(double t) const` | 누적 거리 $t$에서 보간된 Y 좌표를 반환합니다. |
    | `calc_yaw` | `double calc_yaw(double t) const` | $\text{arctan2}(y', x')$ 연산을 사용하여 누적 거리 $t$에서의 궤적 헤딩(Yaw)을 계산하고 반환합니다. |
    | `calc_curvature` | `double calc_curvature(double t) const` | 누적 거리 $t$에서의 곡률 $\kappa$를 계산합니다. 계산 안정성을 보장하기 위해 분모가 `1e-6`보다 작으면 $0.0$으로 클램프(clamp)합니다. |
