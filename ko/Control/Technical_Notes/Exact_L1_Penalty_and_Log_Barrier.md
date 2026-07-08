# 정확한 L1 페널티 및 로그 장벽 (Exact L1 Penalty and Log-Barrier)

!!! abstract "개요 (Overview)"
    이 문서는 NMPC 라인 서치(Line Search) 프로세스의 스칼라 `merit` 함수가 추적 성능, 제어 노력 및 물리적 제약 조건(예: 장애물 회피)의 균형을 어떻게 맞추는지, 그리고 특히 **물리적 제약 조건을 어떻게 엄격하게 강제하는지** 설명합니다.

## :material-scale-balance: `L1_WEIGHT`의 역할 (정확한 페널티, Exact Penalty)

### 흔한 오해: "위반했을 때만 페널티를 주나요?" (Common Misconception)

$c(x) \le 0$ (예: 안전 거리 위반)과 같은 제약 조건을 다룰 때, 일반적으로 값이 양수일 때(즉, 위반했을 때)만 페널티가 적용된다고 생각하기 쉽습니다. 그러나 장점(Merit) 함수에 사용되는 L1 페널티는 전혀 다르게 작동합니다.

### 엄격한 일관성 강제 (Enforcing Strict Consistency)

부등식 제약 조건을 풀기 위해 IPM 솔버는 가상의 양수 여유 변수(slack variable) $s \ge 0$를 도입하여 제약 조건을 인위적으로 등식 제약 조건으로 변환합니다.

!!! math "여유 변수가 포함된 등식 제약 조건 (Equality Constraint with Slack)"
    $$
    c(x) + s = 0
    $$

장점 함수에 포함된 L1 페널티 항은 다음과 같습니다:

!!! math "L1 페널티 항 (L1 Penalty Term)"
    $$
    w_{L1} |c + s + \alpha ds|
    $$

이 절댓값 $(|\cdot|)$ 항은 제약 조건 위반 여부와 관계없이 **"물리적 현실($c$)이 솔버의 내부 가상 변수($s$)와 완벽하게 일치하는가?"**를 검증합니다.

- 차량이 완전히 안전하여 장애물에서 5m 떨어져 있더라도 ($c = -5$), 솔버의 내부 여유 변수가 $s = 3$으로 계산되었다면 $|-5 + 3| = |-2| = 2$가 됩니다. 이로 인해 $2000 \ (w_{L1} \times 2)$이라는 막대한 페널티가 발생합니다.
- 즉, 이 항은 **물리적 상태와 수학적 상태 간의 단 1mm의 불일치도 용납하지 않는 엄격한 일관성 메커니즘**으로 작동합니다.

## :material-puzzle-outline: 완벽한 분업 (로그 장벽 vs L1 페널티) (Perfect Division of Labor)

이 시스템은 절대적인 견고성(robustness)을 달성하기 위해 두 가지 극단적인 메커니즘을 결합합니다.

<div class="grid cards" markdown>

- :material-chart-bell-curve:
    **로그 장벽 (Log-Barrier)** (장기적 방향 / 소프트 제약, Soft Constraint)
    
    $$-\mu \log(s + \alpha ds)$$
    
    - **메커니즘**: 내부 여유 변수 $s$가 0에 가까워지면 무한대($\infty$) 페널티를 부과합니다.
    - **목적**: "실행 가능한 영역(feasible region) 내에 머무르기"라는 장기적인 목표를 설정합니다. 경계에 닿지 않고 궤적이 내부($s > 0$)에 안전하게 머물도록 부드럽고도 단호하게 밀어냅니다.

- :material-hammer:
    **L1 페널티 (L1 Penalty)** (즉각적인 제어 / 하드 제약, Hard Constraint)
    
    $$w_{L1} |c + s + \alpha ds|$$ (여기서 $w_{L1} = 1000.0$)
    
    - **메커니즘**: 물리적 현실($c$)과 내부 변수($s$)의 합이 0에서 벗어나는 순간 엄청난 선형 페널티를 즉시 적용합니다.
    - **목적**: 솔버가 성능 향상(비용 함수 감소)에 눈이 멀어 라인 서치 중에 제약 조건을 무시하는 스텝($\alpha$)을 밟으려 할 때, 이 항이 무자비한 가중치로 `merit` 값을 폭발시켜 **해당 스텝을 즉각적으로 거부하는 물리적 브레이크** 역할을 합니다.

</div>

## :material-check-decagram: 왜 "정확한(Exact)" 가중치인가?

!!! success "부정확한 페널티 vs 정확한 페널티 (Inexact Penalty vs Exact Penalty)"
    **표준 이차 페널티(quadratic penalty)** ($c^2$)를 사용하는 경우, 가중치가 아무리 크더라도 최적화 솔버는 비용 함수를 더 줄이기 위해 필연적으로 제약 조건을 "약간 위반"하는 트레이드오프(trade-off)를 수행하게 됩니다. (부정확한 페널티)

    그러나 **L1 페널티** ($|c|$)를 사용하고 그 가중치($w_{L1}$)를 시스템의 최대 쌍대 변수($\lambda_{\max}$, 예: $1000.0$)보다 충분히 크게 설정하면, 비용 함수가 얼마나 희생되든 상관없이 제약 조건이 단 1mm도 위반되지 않는 **"정확한 최적해(Exact Optimal Solution)"** 를 산출한다는 것이 수학적으로 증명되어 있습니다.
