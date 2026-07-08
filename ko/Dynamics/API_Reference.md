# 차량 동역학 API 레퍼런스 (Dynamics API Reference)

!!! info "API 문서 (API Documentation)"
    이 문서는 차량 동역학(Vehicle Dynamics) 모듈에서 사용되는 구체적인 API 시그니처(signature)와 데이터 구조에 대해 심층적으로 다룹니다.

## :material-cube-outline: `RealTimeDynamicsModel`

```cpp
class RealTimeDynamicsModel;
```
내점법(Interior-Point) NMPC 솔버에 맞춤화된 최적화된 8-DoF Frenet 자전거 모델입니다. 중요한 비선형 타이어 특성은 보존하면서도 연산 시간을 극도로 단축하기 위해 서스펜션 기구학(kinematics)을 의도적으로 제외했습니다.

### 메서드: `operator()`
```cpp
template <typename T>
matrix::StaticVector<T, 8> operator()(const matrix::StaticVector<T, 8>& x, const matrix::StaticVector<T, 2>& u) const;
```
상태 미분 $\dot{x} = f(x, u)$를 계산합니다.
`T`가 `Dual<double>`로 인스턴스화될 때 자동 미분(Automatic Differentiation)과 호환됩니다.

---

## :material-cube-outline: `HighFidelityDynamicsModel`

```cpp
class HighFidelityDynamicsModel;
```
시뮬레이션 플랜트(Plant) 역할을 하도록 설계된 고정밀 차량 모델입니다.

### 메서드: `extractJacobians`
```cpp
void extractJacobians(const matrix::StaticVector<double, 8>& x0, 
                      const matrix::StaticVector<double, 2>& u0,
                      std::array<std::array<double, 8>, 8>& A,
                      std::array<std::array<double, 2>, 8>& B) const;
```
현재 동작점(operating point) `(x0, u0)`에서 자동 미분(Dual numbers)을 사용하여 정확한 해석적 자코비안($A$ 및 $B$ 행렬)을 연속적으로 추출합니다. 국소적 선형 안정성(local linear stability)을 확인하는 데 매우 유용합니다.

---

## :material-math-integral: `VehiclePhysicsCore.hpp` 함수

### `computeQuasiStaticLoadTransfer()`
```cpp
template <typename T>
FourWheelLoads<T> computeQuasiStaticLoadTransfer(
    const VehicleDynamicsParams<T>& params, const T v_x, const T yaw_rate, const T a_cmd);
```
강체 정적 근사(rigid-body static approximations)를 사용하여 타이어의 수직 하중을 계산합니다.

### `computeSuspensionLoadTransfer()`
```cpp
template <typename T>
FourWheelLoads<T> computeSuspensionLoadTransfer(
    const VehicleDynamicsParams<T>& veh_params, const SuspensionParams<T>& susp_params,
    const T v_x, const T yaw_rate, const T a_cmd, ChassisAttitude<T>& out_attitude);
```
서스펜션 스프링, 피치 각도, 롤 각도 및 롤 센터 기구학을 시뮬레이션하여 4륜 하중을 계산합니다.

### `computeBicycleLateralForces()`
```cpp
template <typename T>
BicycleLateralForces<T> computeBicycleLateralForces(
    const TireLoadParams<T>& tire_params, const FourWheelLoads<T>& loads, 
    const T alpha_f, const T alpha_r);
```
4개 타이어 모두에 대해 개별적으로 Pacejka Magic Formula를 평가하고 이를 합산하여 동등한 전륜 및 후륜 차축(axle) 횡력을 계산합니다.

---

## :material-flash: `FastMath.hpp`

### `FastTrig` 클래스
사전 계산된 1024개 요소의 룩업 테이블(LUT)을 사용하여 `sin` 및 `atan`을 계산하는 싱글톤 클래스 `FastTrig::getInstance()`입니다.

### 템플릿 라우팅 (Template Routing)
```cpp
template <typename T>
inline T math_sin(const T& x);
```
- `T`가 `double`인 경우, `FastTrig`로 리디렉션하여 엄청난 속도 향상을 얻습니다.
- `T`가 `Dual<double>`인 경우, `std::sin()` 또는 `ad::sin()`으로 리디렉션하여 자동 미분 체인이 LUT 불연속성 없이 정확한 해석적 미분을 평가하도록 보장합니다.
