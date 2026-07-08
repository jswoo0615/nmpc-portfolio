# 유틸리티 API 레퍼런스 (Utils API Reference)

!!! info "API 문서 (API Documentation)"
    이 문서는 NMPC 프레임워크에서 사용되는 보조 유틸리티에 대한 API를 간략히 설명합니다.

---

## :material-cube-outline: `CUDAMacros.hpp`

```cpp
#ifdef __CUDAACC__
    #define CUDA_CALLABLE __host__ __device__
#else
    #define CUDA_CALLABLE
#endif
```
크로스 플랫폼(cross-platform) 컴파일 매크로입니다. `nvcc`로 컴파일할 때 함수 시그니처에 `__host__ __device__`를 주입하여, 핵심 NMPC 구조체(`StaticMatrix` 등)가 CUDA 커널 내부에서 활용될 수 있도록 합니다.

---

## :material-cube-outline: `EnvironmentEvaluator`

```cpp
template <std::size_t Nx_active, std::size_t MaxObs = 10>
class EnvironmentEvaluator;
```
동적 장애물 회피를 위한 전용 평가기입니다. Frenet 좌표계에서 장애물을 추적하고 충돌 여유(collision margins)를 계산합니다.

### 구조체 (Structures)
```cpp
struct ObstacleFrenet {
    double s = 0.0;
    double d = 0.0;
    double r = 0.5;
    double vs = 0.0;
    double vd = 0.0;
};
```
종방향(`s`) 및 횡방향(`d`) 위치, 반경(`r`) 및 속도를 포함하는 동적 장애물을 정의합니다.

```cpp
template <std::size_t Nx_active>
struct ConstraintGradient {
    double c_val = 0.0;
    matrix::StaticVector<double, Nx_active> J_x;
    bool is_active = false;
};
```
솔버에 전달되는 데이터 구조체입니다. 제약 조건 위반 값 `c_val`과 활성 상태에 대한 자코비안 `J_x`를 포함합니다.

### 메서드: `evaluate_obstacles`
```cpp
std::array<ConstraintGradient<Nx_active>, MaxObs> evaluate_obstacles(double current_s, double current_d, double time_future) const;
```
장애물의 속도를 기반으로 `time_future`에서의 각 장애물의 위치를 예측합니다. 자아 차량(ego vehicle)에서 장애물까지의 유클리드 거리(Euclidean distance)를 계산합니다. 장애물이 관심 영역(ROI, Region of Interest) 내에 있으면 제약 조건 위반과 그 기울기(gradient)를 계산합니다. `is_active = false`로 설정하여 거리가 먼 장애물은 솔버의 계산 부담에서 제외시킵니다.
