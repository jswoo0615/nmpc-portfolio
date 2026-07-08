# 차량 동역학 방정식 (Vehicle Dynamics Equations)

!!! info "수학적 기반 (Mathematical Foundation)"
    이 기술 노트(Technical Note)는 `RealTimeDynamicsModel.hpp`, `HighFidelityDynamicsModel.hpp` 및 `VehiclePhysicsCore.hpp` 내에 구현된 정확한 수학적 공식을 자세히 설명합니다.

---

## 1. 상태 및 입력 정의 (State and Input Definitions)

차량 상태 $x \in \mathbb{R}^8$ 와 제어 입력 $u \in \mathbb{R}^2$ 은 다음과 같이 정의됩니다:

$$
x = \begin{bmatrix} s \\ d \\ \mu \\ v_x \\ v_y \\ r \\ \alpha_f \\ \alpha_r \end{bmatrix}, \quad
u = \begin{bmatrix} \delta \\ a_{cmd} \end{bmatrix}
$$

| 변수 | 설명 | 단위 |
| :---: | :--- | :---: |
| $s$ | 기준 경로를 따른 호의 길이 (Arc length) | m |
| $d$ | 기준 경로로부터의 횡방향 편차 (Lateral deviation) | m |
| $\mu$ | 기준 경로에 대한 헤딩 각도 오차 (Heading angle error) | rad |
| $v_x$ | 종방향 속도 (차체 좌표계) | m/s |
| $v_y$ | 횡방향 속도 (차체 좌표계) | m/s |
| $r$ | 요 레이트 (Yaw rate, 차체 좌표계) | rad/s |
| $\alpha_f$ | 전륜 타이어 슬립각 (Front tire slip angle) | rad |
| $\alpha_r$ | 후륜 타이어 슬립각 (Rear tire slip angle) | rad |
| $\delta$ | 전륜 조향각 (Front steering angle) | rad |
| $a_{cmd}$ | 종방향 가속도 명령 (Longitudinal acceleration cmd) | $m/s^2$ |

---

## 2. 하중 이동 물리 (Load Transfer Physics) (`VehiclePhysicsCore.hpp`)

횡력을 정확하게 모델링하려면 각 타이어의 수직 하중($F_z$)을 추정해야 합니다.

### 2.1 준정적 하중 이동 (Quasi-Static Load Transfer, 실시간 모델)
NMPC 루프의 계산 부담을 최소화하기 위해, `RealTimeDynamicsModel`은 독립적인 롤(roll) 및 피치(pitch) 적분기 없이 하중 이동을 계산하기 위해 횡방향 가속도의 정적 근사치($a_y \approx v_x \cdot r$)를 사용합니다.

**정적 무게 배분 (Static Weight Distribution):**
$$
F_{z,f}^{stat} = m g \frac{l_r}{L}, \quad F_{z,r}^{stat} = m g \frac{l_f}{L}
$$

**종방향 하중 이동 (Longitudinal Load Transfer):**
$$
\Delta F_z^{long} = m a_{cmd} \frac{h_{cg}}{L}
$$

**횡방향 하중 이동 (Lateral Load Transfer, 전/후륜):**
$$
\Delta F_{z,lat}^{front} = m (v_x r) \frac{h_{cg}}{t_f} K_{df}
$$
$$
\Delta F_{z,lat}^{rear} = m (v_x r) \frac{h_{cg}}{t_r} (1 - K_{df})
$$

여기서 $K_{df}$는 전륜 롤 강성 분배 비율(front roll stiffness distribution ratio)이며, $t_f, t_r$은 윤거(track widths)입니다.

### 2.2 서스펜션 기하학적 하중 이동 (Suspension Geometric Load Transfer, 고정밀 모델)
`HighFidelityDynamicsModel`은 롤 센터 높이와 서스펜션 스프링 강성을 모델링하여 섀시 롤($\phi$)과 피치($\theta$)를 능동적으로 계산합니다.

$$
\theta = \frac{m a_{cmd} h_{cg}}{K_{pitch}}
$$
$$
\phi = \frac{m (v_x r) h_{roll\_arm}}{K_{roll} - m g h_{roll\_arm}}
$$

이러한 각도는 정확한 동적 휠 하중 및 해당 캠버 추력(camber thrust) 효과를 계산하는 데 사용됩니다.

---

## 3. 타이어 물리 & 매직 포뮬러 (Tire Physics & Magic Formula)

### 3.1 정상 상태 슬립각 계산 (Steady-State Slip Angle calculation)
정상 상태 기구학적 슬립각은 전륜 및 후륜 차축의 속도를 사용하여 계산됩니다:

$$
\alpha_{ss,f} = \delta - \arctan\left(\frac{v_y + l_f r}{v_x}\right)
$$
$$
\alpha_{ss,r} = - \arctan\left(\frac{v_y - l_r r}{v_x}\right)
$$

### 3.2 이완 길이 / 1차 지연 (Relaxation Length / First-Order Delay)
타이어 힘은 즉각적으로 발생하지 않습니다. 수치적 불안정성을 방지하고 실제 고속 동역학을 모델링하기 위해 이완 길이($\sigma$)가 도입됩니다:

$$
\dot{\alpha}_f = \frac{v_x}{\sigma_f} (\alpha_{ss,f} - \alpha_f)
$$
$$
\dot{\alpha}_r = \frac{v_x}{\sigma_r} (\alpha_{ss,r} - \alpha_r)
$$

### 3.3 Pacejka Magic Formula
비선형 횡력 $F_y$는 하중에 종속적인 Pacejka Magic Formula를 사용하여 계산됩니다.

**마찰 계수 감소 (Friction Coefficient Decay):**
$$
\mu = \mu_0 - d\mu (F_z - F_{z,nom})
$$

**코너링 강성 (Cornering Stiffness):**
$$
C_\alpha = c_1 F_z - c_2 F_z^2
$$

**횡력 계산 (Lateral Force Calculation):**
$$
D = \mu F_z, \quad B = \frac{C_\alpha}{C_{shape} D}
$$
$$
F_y = D \sin(C_{shape} \arctan(B \alpha))
$$

*(C++ 구현에서는 배정밀도(double precision) 연산 시 `FastMath.hpp`를 통해 표준 `sin` 및 `atan`이 고속 LUT 연산으로 대체되어 실시간 적분을 가속화합니다.)*

---

## 4. Frenet 기구학 및 운동 방정식 (Frenet Kinematics and Equations of Motion)

마지막으로 힘을 결합하여 상태 미분을 구합니다.

$\kappa$를 차량 위치의 도로 곡률(road curvature)이라고 합시다.

### 4.1 기구학 (경로 추종, Kinematics)
$$
\dot{s} = \frac{v_x \cos(\mu) - v_y \sin(\mu)}{1 - d \cdot \kappa}
$$
$$
\dot{d} = v_x \sin(\mu) + v_y \cos(\mu)
$$
$$
\dot{\mu} = r - \kappa \cdot \dot{s}
$$

*(엄격하게 후진하거나 경로 위에 완벽하게 있을 때의 특이점(singularities)을 피하기 위해 분모 $(1 - d \cdot \kappa)$는 안전하게 제한됩니다.)*

### 4.2 동역학 (차체 가속도, Dynamics)
$$
\dot{v}_x = a_{cmd} - \frac{F_{drag}}{m} + r \cdot v_y
$$
$$
\dot{v}_y = \frac{F_{y,f} \cos(\delta) + F_{y,r}}{m} - r \cdot v_x
$$
$$
\dot{r} = \frac{l_f F_{y,f} \cos(\delta) - l_r F_{y,r}}{I_z}
$$

이 방정식들은 본질적으로 $\dot{x} = f(x, u)$ 벡터를 형성하며, 이는 `SparseNMPC_IPM` 솔버 내부의 Runge-Kutta 4 (RK4) 적분기로 공급됩니다.
