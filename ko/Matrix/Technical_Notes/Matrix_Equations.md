# 행렬 수학적 기초 (Matrix Mathematical Foundations)

!!! abstract "개요 (Overview)"
    이 문서는 맞춤형 `Matrix` 모듈을 구동하는 핵심 수학적 공식을 요약합니다. 쌍대 수(Dual Numbers)를 통한 다차원 자동 미분의 대수적 기반과 최적화된 선형 대수 솔버 이면의 알고리즘에 중점을 둡니다.

---

## 1. 쌍대 수를 통한 자동 미분 (Automatic Differentiation via Dual Numbers)

전통적인 유한 차분법(finite difference methods)은 치명적인 자릿수 떨어짐(catastrophic cancellation)과 부동소수점 잘림 오차(floating-point truncation errors)로 인해 어려움을 겪습니다. 한 번의 평가(evaluation) 패스로 정확한 자코비안 $J = \frac{\partial f}{\partial x}$를 계산하기 위해, 우리는 **쌍대 수(Dual Numbers)** 를 사용합니다.

### 1.1 쌍대 수 대수 (Dual Number Algebra)

쌍대 수 $z$는 $z = v + g \epsilon$으로 정의되며, 여기서 $v$는 실수(원시/primal) 부분, $g$는 쌍대(기울기/gradient) 부분, $\epsilon$은 다음을 만족하는 무한소(infinitesimal) 단위입니다:
$$ \epsilon \neq 0, \quad \epsilon^2 = 0 $$

매끄러운 함수(smooth function) $f(x)$에 테일러 전개(Taylor expansion)를 수행하면:
$$ f(v + g \epsilon) = f(v) + f'(v)(g \epsilon) + \frac{f''(v)}{2}(g \epsilon)^2 + \dots $$
$\epsilon^2 = 0$이므로 모든 고차항(higher-order terms)은 즉시 사라져 정확한 해석적 도함수(analytical derivatives)가 생성됩니다:
$$ f(v + g \epsilon) = f(v) + f'(v) g \epsilon $$

### 1.2 다차원 전파 (`DualVec`) (Multidimensional Propagation)

`DualVec<T, N>`에서 기울기 $g$는 벡터 $\mathbf{g} \in \mathbb{R}^N$로 확장됩니다. 다변수 함수 $f: \mathbb{R} \to \mathbb{R}$를 적용하면:
$$ f(v + \mathbf{g} \epsilon) = f(v) + \nabla f(v) \cdot \mathbf{g} \epsilon $$

기본 연산에 대해 대수적 규칙은 SIMD 벡터화 실행에 직접 매핑됩니다:

- **덧셈 (Addition)**: $(v_1 + \mathbf{g}_1 \epsilon) + (v_2 + \mathbf{g}_2 \epsilon) = (v_1 + v_2) + (\mathbf{g}_1 + \mathbf{g}_2) \epsilon$
- **곱셈 (Multiplication)**: $(v_1 + \mathbf{g}_1 \epsilon)(v_2 + \mathbf{g}_2 \epsilon) = (v_1 v_2) + (v_1 \mathbf{g}_2 + v_2 \mathbf{g}_1) \epsilon$
- **나눗셈 (Division)**: $\frac{v_1 + \mathbf{g}_1 \epsilon}{v_2 + \mathbf{g}_2 \epsilon} = \frac{v_1}{v_2} + \frac{v_2 \mathbf{g}_1 - v_1 \mathbf{g}_2}{v_2^2} \epsilon$

---

## 2. 선형 대수 인수분해 (Linear Algebra Factorizations)

힙(heap) 할당 없이 선형 연립 방정식 $Ax = b$를 효율적으로 푸는 것이 `Linalg` 모듈의 주요 목표입니다.

### 2.1 숄레스키 분해 (Cholesky Decomposition) ($A = LL^T$)

대칭 양의 정부호 행렬(symmetric positive-definite matrix) $A$에 대해, 숄레스키 인수분해는 하삼각 행렬(lower triangular matrix) $L$을 계산합니다:
$$ A = L L^T $$

$L$의 원소들은 순차적으로 도출됩니다. 대각선 요소의 경우:
$$ L_{j,j} = \sqrt{A_{j,j} - \sum_{k=0}^{j-1} L_{j,k}^2} $$

비대각 요소($i > j$)의 경우:
$$ L_{i,j} = \frac{1}{L_{j,j}} \left( A_{i,j} - \sum_{k=0}^{j-1} L_{i,k} L_{j,k} \right) $$

!!! tip "SIMD 하드웨어 가속 (SIMD Hardware Acceleration)"
    우리의 `cholesky_solver`에서 내적(dot product) $\sum L_{i,k} L_{j,k}$는 스칼라 루프를 우회하여 AVX2 FMA (Fused Multiply-Add)와 같은 SIMD 명령어를 사용하여 블록 단위로 계산됩니다.

### 2.2 LDLT 분해 (LDLT Decomposition) ($A = LDL^T$)

$A$가 특이점(singularity)에 접근하거나 음의 고윳값(negative eigenvalues)을 갖는 경우 (예: 충분한 정규화가 없어 조건이 나쁜(poorly conditioned) 헤시안), 숄레스키는 음수의 제곱근 때문에 실패합니다. LDLT 인수분해는 제곱근을 완전히 피합니다:
$$ A = L D L^T $$
여기서 $D$는 대각 행렬입니다. 재귀 공식은 다음과 같습니다:

$$ D_{j,j} = A_{j,j} - \sum_{k=0}^{j-1} L_{j,k}^2 D_{k,k} $$
$$ L_{i,j} = \frac{1}{D_{j,j}} \left( A_{i,j} - \sum_{k=0}^{j-1} L_{i,k} L_{j,k} D_{k,k} \right) $$

### 2.3 QR 분해 (하우스홀더 반사) (QR Decomposition / Householder Reflections)

정방 행렬이 아니거나 조건이 매우 나쁜(ill-conditioned) 시스템에 대해, QR 분해는 $A$를 직교 행렬(orthogonal matrix) $Q$ ($Q^T Q = I$)와 상삼각 행렬(upper triangular matrix) $R$로 인수분해합니다:
$$ A = Q R $$

우리는 대각선 아래의 요소들을 0으로 만들기 위해 **하우스홀더 반사(Householder Reflections)** 를 활용합니다. 대상 열 벡터 $x$에 대해 법선 벡터 $v$를 갖는 반사 초평면(reflection hyperplane)을 설계합니다:
$$ v = x + \text{sign}(x_1) \|x\|_2 e_1 $$
반사 행렬은 다음과 같습니다:
$$ H = I - 2 \frac{v v^T}{v^T v} $$

이 반사를 재귀적으로 반복 적용하면 $A$가 삼각화됩니다. $Ax = b$의 해는 노이즈에 대해 견고해집니다:
$$ Rx = Q^T b $$
