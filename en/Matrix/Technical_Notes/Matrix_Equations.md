# Matrix Mathematical Foundations

!!! abstract "Overview"
    This document outlines the core mathematical formulations driving the custom `Matrix` module. It focuses on the algebraic basis of Multidimensional Auto-Differentiation via Dual Numbers and the algorithms behind the optimized linear algebra solvers.

---

## 1. Automatic Differentiation via Dual Numbers

Traditional finite difference methods suffer from catastrophic cancellation and floating-point truncation errors. To compute exact Jacobians $J = \frac{\partial f}{\partial x}$ in a single evaluation pass, we employ **Dual Numbers**.

### 1.1 Dual Number Algebra

A dual number $z$ is defined as $z = v + g \epsilon$, where $v$ is the real (primal) part, $g$ is the dual (gradient) part, and $\epsilon$ is an infinitesimal unit satisfying:
$$ \epsilon \neq 0, \quad \epsilon^2 = 0 $$

By performing Taylor expansion on any smooth function $f(x)$:
$$ f(v + g \epsilon) = f(v) + f'(v)(g \epsilon) + \frac{f''(v)}{2}(g \epsilon)^2 + \dots $$
Since $\epsilon^2 = 0$, all higher-order terms vanish instantly, yielding exact analytical derivatives:
$$ f(v + g \epsilon) = f(v) + f'(v) g \epsilon $$

### 1.2 Multidimensional Propagation (`DualVec`)

In `DualVec<T, N>`, the gradient $g$ is extended to a vector $\mathbf{g} \in \mathbb{R}^N$. Applying a multivariable function $f: \mathbb{R} \to \mathbb{R}$:
$$ f(v + \mathbf{g} \epsilon) = f(v) + \nabla f(v) \cdot \mathbf{g} \epsilon $$

For fundamental operations, the algebraic rules map directly to SIMD vectorized execution:

- **Addition**: $(v_1 + \mathbf{g}_1 \epsilon) + (v_2 + \mathbf{g}_2 \epsilon) = (v_1 + v_2) + (\mathbf{g}_1 + \mathbf{g}_2) \epsilon$
- **Multiplication**: $(v_1 + \mathbf{g}_1 \epsilon)(v_2 + \mathbf{g}_2 \epsilon) = (v_1 v_2) + (v_1 \mathbf{g}_2 + v_2 \mathbf{g}_1) \epsilon$
- **Division**: $\frac{v_1 + \mathbf{g}_1 \epsilon}{v_2 + \mathbf{g}_2 \epsilon} = \frac{v_1}{v_2} + \frac{v_2 \mathbf{g}_1 - v_1 \mathbf{g}_2}{v_2^2} \epsilon$

---

## 2. Linear Algebra Factorizations

Solving the system of linear equations $Ax = b$ efficiently without heap allocations is the primary objective of the `Linalg` module.

### 2.1 Cholesky Decomposition ($A = LL^T$)

For a symmetric positive-definite matrix $A$, Cholesky factorization computes a lower triangular matrix $L$:
$$ A = L L^T $$

The elements of $L$ are derived sequentially. For the diagonal entries:
$$ L_{j,j} = \sqrt{A_{j,j} - \sum_{k=0}^{j-1} L_{j,k}^2} $$

For the off-diagonal entries ($i > j$):
$$ L_{i,j} = \frac{1}{L_{j,j}} \left( A_{i,j} - \sum_{k=0}^{j-1} L_{i,k} L_{j,k} \right) $$

!!! tip "SIMD Hardware Acceleration"
    In our `cholesky_solver`, the dot product $\sum L_{i,k} L_{j,k}$ is computed block-by-block using SIMD instructions like AVX2 FMA (Fused Multiply-Add), bypassing scalar loops.

### 2.2 LDLT Decomposition ($A = LDL^T$)

When $A$ approaches singularity or has negative eigenvalues (e.g., poorly conditioned Hessians without enough regularization), Cholesky fails due to the square root of a negative number. The LDLT factorization avoids square roots entirely:
$$ A = L D L^T $$
Where $D$ is a diagonal matrix. The recursive formulas are:

$$ D_{j,j} = A_{j,j} - \sum_{k=0}^{j-1} L_{j,k}^2 D_{k,k} $$
$$ L_{i,j} = \frac{1}{D_{j,j}} \left( A_{i,j} - \sum_{k=0}^{j-1} L_{i,k} L_{j,k} D_{k,k} \right) $$

### 2.3 QR Decomposition (Householder Reflections)

For non-square or severely ill-conditioned systems, QR decomposition factorizes $A$ into an orthogonal matrix $Q$ ($Q^T Q = I$) and an upper triangular matrix $R$:
$$ A = Q R $$

We utilize **Householder Reflections** to zero out elements below the diagonal. For a target column vector $x$, we design a reflection hyperplane with normal vector $v$:
$$ v = x + \text{sign}(x_1) \|x\|_2 e_1 $$
The reflection matrix is:
$$ H = I - 2 \frac{v v^T}{v^T v} $$

Applying this reflection recursively iteratively triangularizes $A$. The solution to $Ax = b$ becomes robust against noise:
$$ Rx = Q^T b $$
