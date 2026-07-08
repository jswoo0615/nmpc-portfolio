# 수학적 노트: 정적 3차 스플라인 유도 (Mathematical Note: Static Cubic Spline Derivation)

!!! info "배경 (Background)"
    이 문서는 `StaticCubicSpline` 보간기에 대한 수학적 모델링, 미분 전개 및 삼중 대각 행렬(tridiagonal matrix) 공식을 간략히 설명합니다.
    코드베이스에 사용된 정확한 수치적 유도를 이해하기 위한 기술적 참조 자료로 사용됩니다.

## :material-numeric-1-box: 1. 1D 3차 스플라인 기본 정의 (1D Cubic Spline Basic Definition)

각 구간 $[x_{i}, x_{i+1}]$에서 스플라인 함수는 이산적으로 3차 다항식으로 정의되며, 시작점 $x_{i}$를 로컬 좌표계의 원점으로 삼습니다:

!!! math "다항식 정의 (Polynomial Definition)"
    $$S_{i}(x) = a_{i} + b_{i}(x - x_{i}) + c_{i}(x - x_{i})^{2} + d_{i}(x - x_{i})^{3}$$

계수 $a_{i}, b_{i}, c_{i}, d_{i}$는 다음 연속성 및 경계 조건을 만족하도록 계산됩니다:

- **초기 조건 (Initial Condition)**: $a_{i} = y_{i}$ (각 구간 시작점의 함수값)
- **구간 간격 (Interval Spacing)**: $h_{i} = x_{i+1} - x_{i}$
- **자연 스플라인 경계 조건 (Natural Spline Boundary Condition)**: 양 끝점에서의 곡률(2차 미분)은 0으로 가정합니다:
  $$S_{0}^{\prime \prime}(x_{0}) = 0, \quad S_{n-1}^{\prime \prime}(x_{n}) = 0$$

### 1.1. 미분 전개 (Derivative Expansion)

다항식을 미분하여 1차 및 2차 미분값을 유도합니다:

- **1차 미분 (First Derivative)**: $S_{i}^{\prime}(x) = b_{i} + 2c_{i}(x - x_{i}) + 3d_{i}(x - x_{i})^{2}$
- **2차 미분 (Second Derivative)**: $S_{i}^{\prime \prime}(x) = 2c_{i} + 6d_{i}(x - x_{i})$

## :material-numeric-2-box: 2. 2D 3차 스플라인으로의 확장 (Extension to 2D Cubic Spline)

차량 동역학 제어와 같은 2D 공간에서는 누적된 호의 길이(arc length) 파라미터 $s$를 기반으로 $X$ 및 $Y$ 좌표가 독립적인 1D 스플라인으로 보간됩니다.

<div class="grid cards" markdown>

- :material-map-marker-path: **위치 보간 (Position Interpolation)**
    $$X(s) = S_{x}(s), \quad Y(s) = S_{y}(s)$$

- :material-compass-outline: **헤딩(Yaw) 유도 (Heading Derivation)**
    $$\theta (s) = \text{arctan2}\left(\frac{dY}{ds}, \frac{dX}{ds}\right)$$

- :material-steering: **곡률 유도 (Curvature Derivation)**
    $$\kappa(s) = \frac{\frac{d^{2}Y}{ds^{2}}\frac{dX}{ds} - \frac{d^{2}X}{ds^{2}}\frac{dY}{ds}}{\left(\left(\frac{dX}{ds}\right)^{2} + \left(\frac{dY}{ds}\right)^{2}\right)^{3/2}}$$

</div>

## :material-numeric-3-box: 3. 핵심 계수 유도 및 삼중 대각 행렬 (Core Coefficient Derivation & Tridiagonal Matrix)

스플라인 보간법의 핵심은 **모든 구간이 경계점 $x_{i+1}$에서 부드럽게 연결되어야 한다**는 제약 조건을 통해 $c_{i}$를 풀기 위한 연립 방정식을 세우는 것입니다.

### 3.1. 연속성 조건의 이산적 해석 (Discrete Interpretation of Continuity Condition)

구간 $i$의 오른쪽 끝점과 구간 $i+1$의 왼쪽 끝점은 글로벌 좌표계에서 동일한 노드 $x_{i+1}$입니다:

- **구간 $i$의 오른쪽 끝점**: 로컬 좌표계에서 $(x - x_{i}) = h_{i}$
- **구간 $i+1$의 왼쪽 끝점**: 자체 로컬 좌표계에서 $(x - x_{i+1}) = 0$

따라서 연속성 조건은 다음과 같이 요약됩니다: "동일한 글로벌 노드 $x_{i+1}$에서 왼쪽 곡선 $S_{i}$의 끝 상태와 오른쪽 곡선 $S_{i+1}$의 시작 상태가 일치해야 합니다."

### 3.2. 미분 연속성을 통한 연립 방정식 (System of Equations via Derivative Continuity)

=== "1단계. 2차 미분 연속성 (Step 1. Second Derivative Continuity)"
    - $S_{i}^{\prime \prime}(x_{i+1}) = 2c_{i} + 6d_{i}h_{i}$
    - $S_{i+1}^{\prime \prime}(x_{i+1}) = 2c_{i+1} + 6d_{i+1}(0) = 2c_{i+1}$
    - 이를 같다고 놓으면: $2c_{i} + 6d_{i}h_{i} = 2c_{i+1} \implies d_{i} = \frac{c_{i+1} - c_{i}}{3h_{i}}$

=== "2단계. 함수값 연속성 (Step 2. Function Value Continuity)"
    - $S_{i}(x_{i+1}) = a_{i} + b_{i}h_{i} + c_{i}h_{i}^{2} + d_{i}h_{i}^{3} = a_{i+1}$
    - $b_{i}$에 대해 정리하면 (1단계에서 구한 $d_{i}$ 대입):
      $$b_{i} = \frac{a_{i+1} - a_{i}}{h_{i}} - \frac{h_{i}}{3}(c_{i+1} + 2c_{i})$$

=== "3단계. 1차 미분 연속성 및 $\alpha$ 유도 (Step 3. First Derivative Continuity & Deriving $\alpha$)"
    - $S_{i}^{\prime}(x_{i+1}) = b_{i} + 2c_{i}h_{i} + 3d_{i}h_{i}^{2} = b_{i+1}$
    - 1단계와 2단계에서 얻은 $b_{i}, b_{i+1}, d_{i}$를 이 방정식에 대입하고 $c$에 대해 내림차순으로 정리하면 다음과 같은 핵심 관계식이 유도됩니다:
      $$h_{i-1}c_{i-1} + 2(h_{i-1} + h_{i})c_{i} + h_{i}c_{i+1} = \frac{3}{h_{i}}(a_{i+1} - a_{i}) - \frac{3}{h_{i-1}}(a_{i} - a_{i-1})$$

### 3.3. 보조 항 $\alpha$의 정의 및 의미 (Definition and Meaning of the Auxiliary Term $\alpha$)

방정식의 우변을 $\alpha_{i}$로 정의합니다:

!!! math "알파 정의 (Alpha Definition)"
    $$\alpha_{i} = \frac{3}{h_{i}}(a_{i+1} - a_{i}) - \frac{3}{h_{i-1}}(a_{i} - a_{i-1})$$

**물리적 의미:**
$\frac{a_{i+1} - a_{i}}{h_{i}}$는 오른쪽 구간의 평균 기울기이고, $\frac{a_{i} - a_{i-1}}{h_{i-1}}$는 왼쪽 구간의 평균 기울기입니다. 즉, $\alpha_{i}$는 **두 구간 간의 기울기 변화 차이**를 스케일링한 값으로, 해당 노드에서 스플라인이 요구하는 '곡률(구부러짐)' 정도를 정량화합니다.

이로써 $c_{i}$를 미지수로 하는 $N \times N$ 삼중 대각 행렬(Tridiagonal Matrix) 시스템이 완성됩니다. 이는 Thomas 알고리즘(전진 소거 및 후진 대입)을 통해 $\mathcal{O}(N)$ 시간 복잡도로 결정론적 연산을 가능하게 합니다.

## :material-numeric-4-box: 4. 수치적 검증 예시 (Numerical Verification Example)

개념을 검증하기 위해 4개의 데이터 포인트에 대한 1D 스플라인 계수를 직접 계산해 보겠습니다.

- **주어진 포인트 (Given Points)**: $(0, 0), (1, 1), (2, 0), (3, 1)$
- **초기 설정 (Initial Setup)**: 
  - $h_{0} = 1, h_{1} = 1, h_{2} = 1$
  - $a_{0} = 0, a_{1} = 1, a_{2} = 0, a_{3} = 1$

### 4.1. $\alpha$ 계산 (Calculation of $\alpha$)

- $\alpha_{1} = \frac{3}{1}(0 - 1) - \frac{3}{1}(1 - 0) = -3 - 3 = -6$
- $\alpha_{2} = \frac{3}{1}(1 - 0) - \frac{3}{1}(0 - 1) = 3 - (-3) = 6$

### 4.2. 삼중 대각 행렬 시스템 구성 및 풀이 (Constructing and Solving the Tridiagonal Matrix System)

자연 스플라인(Natural Spline) 경계 조건으로 인해 $c_{0} = 0$ 및 $c_{3} = 0$입니다.

- $i = 1$ 일 때: $1 \cdot 0 + 2(1 + 1)c_{1} + 1 \cdot c_{2} = -6 \implies 4c_{1} + c_{2} = -6$
- $i = 2$ 일 때: $1 \cdot c_{1} + 2(1 + 1)c_{2} + 1 \cdot 0 = 6 \implies c_{1} + 4c_{2} = 6$

연립 방정식 풀이:

1. $c_{2} = -6 - 4c_{1}$을 두 번째 방정식에 대입:
   $$c_{1} + 4(-6 - 4c_{1}) = 6 \implies c_{1} - 24 - 16c_{1} = 6 \implies -15c_{1} = 30 \implies c_{1} = -2$$
2. $c_{2} = -6 - 4(-2) = 2$
3. 결과: $c_{0} = 0, c_{1} = -2, c_{2} = 2, c_{3} = 0$

### 4.3. 나머지 계수($b_{i}, d_{i}$) 계산 (Calculation of Remaining Coefficients)

공식 사용: $b_{i} = \frac{a_{i+1} - a_{i}}{h_{i}} - \frac{h_{i}}{3}(c_{i+1} + 2c_{i}), \quad d_{i} = \frac{c_{i+1} - c_{i}}{3h_{i}}$

=== "구간 0 (Interval 0): $[0, 1]$"
    - $b_{0} = \frac{1-0}{1} - \frac{1}{3}(-2+0) = 1 + \frac{2}{3} \approx 1.667$
    - $d_{0} = \frac{-2-0}{3} \approx -0.667$

=== "구간 1 (Interval 1): $[1, 2]$"
    - $b_{1} = \frac{0-1}{1} - \frac{1}{3}(2-4) = -1 + \frac{2}{3} \approx -0.333$
    - $d_{1} = \frac{2-(-2)}{3} \approx 1.333$

=== "구간 2 (Interval 2): $[2, 3]$"
    - $b_{2} = \frac{1-0}{1} - \frac{1}{3}(0+4) = 1 - \frac{4}{3} \approx -0.333$
    - $d_{2} = \frac{0-2}{3} \approx -0.667$

### 4.4. 최종 다항식 모델 (Final Polynomial Model)

수치적 연산을 기반으로 한 최종 스플라인 다항식은 다음과 같습니다:

!!! success "최종 스플라인 방정식 (Final Spline Equations)"
    - $S_{0}(x) = 1.667x - 0.667x^{3} \quad (x \in [0, 1])$
    - $S_{1}(x) = 1 - 0.333(x - 1) - 2(x - 1)^{2} + 1.333(x - 1)^{3} \quad (x \in [1, 2])$
    - $S_{2}(x) = -0.333(x - 2) + 2(x - 2)^{2} - 0.667(x - 2)^{3} \quad (x \in [2, 3])$
