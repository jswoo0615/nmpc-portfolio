# 상태 추정기 API 레퍼런스 (Estimator API Reference)

!!! info "API 문서 (API Documentation)"
    이 문서는 고성능 상태 추정(State Estimation) 모듈을 위한 API 시그니처를 간략히 설명합니다.

## :material-cube-outline: `EKF`

```cpp
template <size_t Nx = 8, size_t Nu = 2>
class EKF;
```
하드웨어 제약 조건에 맞춰 최적화된 확장 칼만 필터(Extended Kalman Filter)입니다. 메모리에서 동적으로 전치 행렬(transposed matrices)을 생성하는 것을 피하고, 대신 Zero-Copy SIMD SAXPY 업데이트를 통해 공분산 예측(covariance prediction) 중에 연산을 언롤링(unrolling)합니다.

### 메서드: `predict`
```cpp
template <typename Model>
void predict(Model& model, const matrix::StaticVector<double, Nu>& u, double dt);
```
동적 시스템 모델과 제어 입력 `u`가 주어지면 다음 상태를 예측합니다. 자동 미분(Automatic Differentiation)을 사용하여 자체적으로 자코비안을 평가하고 열 우선(column-major) 메모리 레이아웃을 통해 최적으로 캐싱합니다.

### 메서드: `update`
```cpp
void update(const matrix::StaticVector<double, Nx>& z);
```
취약하고 단순한(naive) 행렬 역산 대신 견고한 LDLT 분해 솔버를 통합하여 측정 업데이트(measurement update)를 적용합니다.

---

## :material-cube-outline: `HistoryBuffer`

```cpp
template <size_t Dim, size_t Capacity>
class HistoryBuffer;
```
과거 추정치 및 센서 데이터를 캐시하도록 맞춤화된 순환 버퍼(circular buffer)입니다. ARM Cortex 유닛에서 심각한 파이프라인 지연(pipeline stalls)을 유발하는 것으로 알려진 모듈로(`%`) 나눗셈을 명시적으로 폐기하고, 이를 빠른 1사이클 조건부 포인터 시프트(conditional pointer shift)로 대체합니다.

### 메서드: `operator[]`
```cpp
inline const matrix::StaticVector<double, Dim>& operator[](size_t age) const;
```
`age`(`0`이 가장 최근 시점)에 따른 과거 요소를 반환합니다. 내부 인덱스 변환 메커니즘은 분기 없는(branchless) CPU 명령어(CSEL)로 처리됩니다.

---

## :material-cube-outline: `SparseMHE`

```cpp
template <size_t HE, typename PlantModel = Dynamics::RealTimeDynamicsModel, size_t Nx = 8, size_t Nu = 2, size_t Nz = 8>
class SparseMHE;
```
엄격하게 실시간 애플리케이션을 위해 설계된 희소 이동 구간 추정기(Sparse Moving Horizon Estimator)입니다. 다량의 센서 노이즈에도 불구하고 견고한 비선형 추적(tracking)을 보장하기 위해 차원 공간을 압축하고 정규화된 비용 함수(cost-functional)에 LDLT 분해기를 결합합니다.

### 메서드: `solve`
```cpp
bool solve();
```
전체 상태 히스토리 윈도우를 업데이트하기 위해 최적화 루틴을 실행합니다. 가우스-조던 소거법(Gauss-Jordan elimination)을 배제하여, 실행 시간을 최대 2회의 반복 내에서 타이트하게 제한된 $O(N^2)$ 실행 루프로 엄격하게 제한합니다.
