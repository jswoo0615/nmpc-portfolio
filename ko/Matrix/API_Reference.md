# 행렬 API 레퍼런스 (Matrix API Reference)

!!! info "API 문서 (API Documentation)"
    이 문서는 코어(Core), 자동 미분(Auto-Differentiation, AD) 및 선형 대수(Linear Algebra, Linalg) 클래스의 API 시그니처를 간략히 설명합니다.

---

## :material-math-integral: 1. 코어 모듈 (Core Module)

### :material-cube-outline: `StaticMatrix`
```cpp
template <typename T, std::size_t Rows, std::size_t Cols>
class alignas(64) StaticMatrix;
```
중심적인 메모리 컨테이너입니다. CPU 캐시 라인과 완벽하게 동기화되도록 64바이트로 완전히 정렬된(fully aligned) 데이터를 스택(stack)에 할당합니다. SIMD에 최적화된 생성자 및 할당 연산자를 통합하고 있습니다.

### :material-cube-outline: `StaticMatrixView`
```cpp
template <typename T, std::size_t Rows, std::size_t Cols>
class StaticMatrixView;
```
원시 데이터 포인터 위에 매핑되는 가벼운 제로 카피(zero-copy) 추상화로, `StaticMatrix`와 동일하게 작동합니다. 복사를 유발하지 않고 다른 구조체가 소유한 메모리 블록과 원활하게 인터페이스하는 데 필수적입니다.

---

## :material-math-integral: 2. 자동 미분 (Automatic Differentiation, AD)

### :material-cube-outline: `DualVec`
```cpp
template <typename T, std::size_t N> 
struct alignas(64) DualVec;
```
정확한 1차 도함수(first-order derivatives)를 전파하는 데 사용되는 다차원 쌍대 수(Dual Number) 구조체입니다.
`v`는 원시(primal) 값을 저장하는 반면, 배열 `g[N]`은 기울기 벡터(gradient vector)를 유지합니다. 하드웨어의 수평 연산(horizontal operations)을 활용하고 메모리 경계를 정렬하여 병목 없는 인스턴스화를 보장합니다.

---

## :material-math-integral: 3. 선형 대수 (Linear Algebra, Linalg)

`linalg` 네임스페이스는 추상적인 다형성(abstract polymorphism)을 피합니다. 대신, 중소규모 행렬에 최적화된 정적 인라인 솔버(static inline solvers)를 제공합니다. 이러한 루틴은 스택의 진동(oscillation)을 금지하기 위해 **제자리(In-place) 메모리 재사용**을 우선시합니다.

### :material-function: Cholesky 인수분해 (Cholesky Factorization)
```cpp
template <typename MatL, typename VecB, typename VecX>
inline void cholesky_solver(const MatL& L, const VecB& b, VecX& x) noexcept;
```
$A = L L^T$일 때 $A x = b$를 풉니다. 임시 할당을 완전히 우회합니다. 가장 안쪽의 루프는 표준 배열 덤프를 플랫폼별 네이티브 수평 덧셈(`_mm256_hadd_pd` / `vpaddq_f64`)으로 대체합니다.

### :material-function: LDLT 인수분해 (LDLT Factorization)
```cpp
template <class MatType, class VecB, class VecX>
inline void LDLT_solve(const MatType& mat, const VecB& b, VecX& x) noexcept;
```
주로 수치적 결함으로 인해 헤시안에 거의 특이점(near-singular)에 가까운 요소나 음의 고윳값(negative eigenvalues)이 포함되어 있어 정상적이라면 Cholesky 연산이 충돌할 때 사용됩니다. `NaN` 침투에 대한 보호 장치가 포함되어 있습니다.

### :material-function: LU 인수분해 (LU Factorization)
```cpp
template <typename MatLU, typename VecB, typename VecX>
inline void lu_solve(const MatLU& LU, const std::array<int, MatLU::NumRows>& p, const VecB& b, VecX& x) noexcept;
```
피벗 순열(pivot permutations)을 출력 벡터 `x`에 직접 적용하여 임시적인 $O(N)$ 할당을 완전히 차단합니다.

### :material-function: QR 인수분해 (QR Factorization)
```cpp
template <class MatType, class VecTau>
inline MathStatus QR_decompose_Householder(MatType& mat, VecTau& tau) noexcept;
```
공격적인 SIMD 내적 가속(dot-product acceleration)을 활용하여 Householder 반사(reflections)를 적용합니다. 견고한 과결정 시스템(overdetermined system) 솔루션에 사용됩니다.
