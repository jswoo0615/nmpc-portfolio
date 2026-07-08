# 기술 노트: 여유 변수와 L1 페널티 (Technical Note: Slack Variables and L1 Penalty)

!!! info "배경 (Background)"
    이 문서는 희소 NMPC를 위한 원시-쌍대 내점법(Primal-Dual Interior Point Method)을 구현할 때 흔히 마주치는 오해들을 요약합니다.
    
    엄밀한 수학적 증명보다는 수치적 동작 이면에 있는 직관을 설명하기 위한 것입니다.

## :material-help-circle: 질문 1: $c = -5$ 일 때 어떻게 $s = 3$ 이 될 수 있나요? (How can $s = 3$ when $c = -5$?)

**질문:**
> 차량이 매우 안전하여 장애물에서 5m 떨어져 있더라도 ($c = -5$), 솔버의 내부 여유 변수(slack variable)가 $s = 3$으로 계산된다면 $|-5 + 3| = |-2| = 2$가 되어 2000 ($w_{L1} \times 2$)이라는 막대한 페널티가 발생합니다.
> 만약 $s = 3$이라면, 여유 변수는 $g(x) + s \ge 0$ 형태로 도입되므로 제약 조건 경계가 유연하게 $g(x) \ge -3$으로 이동했다는 뜻인가요?

### 1. 수학적 정정: 그것은 부등식이 아니라 등식입니다 (Mathematical Correction: It's an Equality, Not an Inequality)

$g(x) + s \ge 0$에 여유 변수를 도입하면 $s = 3$일 때 경계가 $g(x) \ge -3$으로 이동한다고 생각하는 것은 직관적입니다. 그러나 IPM의 실제 수학적 변환은 다르게 작동합니다.

물리적 제약 조건(예: 안전 여유 위반)이 $c(x) \le 0$일 때, IPM 솔버는 가상의 양수 변수 $s$ ($s > 0$)를 도입하여 이를 **완벽한 등식(Equality)** 으로 강제 변환합니다.

!!! math "등식 제약 조건 (Equality Constraint)"
    $$
    c(x) + s = 0 \quad \text{단, } s > 0
    $$

- **일관된 상태 (Consistent State)**: 차량이 장애물에서 5m 떨어져 있다면 ($c(x) = -5$), 이 방정식이 $0$이 되기 위해서는 ($-5 + 5 = 0$) 여유 변수 $s$가 정확히 $5$여야만 합니다.

### 2. 그렇다면 왜 $c = -5$일 때 $s = 3$이 나타날까요? (Then why does $s = 3$ emerge when $c = -5$?)

여기에 원시-쌍대 내점법(Primal-Dual IPM)의 가장 결정적인 특징이 있습니다.
솔버의 "두뇌" 안에서는 물리적 상태 $x$ (차량 위치 등)와 가상 변수 $s$ (여유 변수)가 **완전히 독립적인 개체**로 취급되며 개별적으로 계산되고 업데이트됩니다.

<div class="grid cards" markdown>

- :material-calculator:
    **1. 뉴턴 스텝 (방향 계산, Newton Step)**
    
    솔버는 연립 방정식([RiccatiSolver](../../Solver/Technical_Notes/Riccati_Solver.md))을 풀어서 $x$를 얼마나 변경할지($\Delta x$)와 $s$를 얼마나 변경할지($\Delta s$)를 동시에 계산합니다.

- :material-arrow-decision:
    **2. 라인 서치 (스텝 크기 $\alpha$ 적용, Line Search)**
    
    두 변수 모두 $\alpha$ 크기의 스텝을 밟습니다:
    
    - 새로운 물리적 상태: $x_{\text{new}} = x + \alpha \Delta x \implies c(x_{\text{new}}) = -5$ 의 결과를 낳음
    - 새로운 여유 변수: $s_{\text{new}} = s + \alpha \Delta s \implies s_{\text{new}} = 3$ 의 결과를 낳음

</div>

**왜 일치하지 않을까요?**
왜냐하면 **$c(x)$는 비선형 함수**인 반면, 솔버가 계산한 $\Delta x$와 $\Delta s$는 **선형화된 근사치(linearized approximations)** (자코비안)를 기반으로 하기 때문입니다. 세상이 선형적이라고 가정하고 큰 스텝($\alpha$)을 밟음으로써, 물리적 현실($c = -5$)과 수학적 예상($s = 3$) 사이에 $2$라는 불일치(Inconsistency)가 발생하게 됩니다.

### 3. L1 페널티의 개입: "거짓말하지 마라" (The L1 Penalty Intervention: "Do Not Lie")

이제 $|-5 + 3| = |-2| = 2$의 진정한 의미가 명확해집니다.
차는 5m 떨어져 있으므로($c = -5$), 물리적으로 충돌 없이 매우 안전합니다.
그러나 수학적으로는 $c(x) + s = 0$ 이라는 절대 규칙이 깨졌습니다: $-5 + 3 = -2 \neq 0$. 솔버의 물리적 세계와 수학적 세계가 동기화를 잃고 서로에게 "거짓말"을 하고 있는 것입니다.

!!! warning "L1 페널티의 응답 (L1 Penalty Response)"
    **[evaluate_merit](Merit_Function.md)**: "차가 물리적으로 안전하다는 것은 인정하지만, 네가 계산한 물리적 상태($c$)와 가상 상태($s$)의 차이가 2만큼 어긋나 있어. 이는 수치 해석이 붕괴하고 있다는 뜻이야. 페널티: 2000! 이 스텝 크기($\alpha$)를 버리고 다시 계산해!"

### 결론 (Conclusion)

제약 조건 경계는 유연하게 $g(x) \ge -3$으로 이동하지 *않습니다*.
반대로, 물리 법칙 $c(x)$와 수학적 가상 변수 $s$는 $c(x) + s = 0$에 의해 완벽하게 묶여 있어야 합니다. 이들이 아주 조금이라도 벌어지는 순간 (예: $c = -5$, $s = 3$), 선형화 오차로 인한 궤적의 탈선을 막기 위해 엄격한 L1 페널티가 부과됩니다.

---

## :material-help-circle-outline: 질문 2: 여유 변수는 제약 조건과 묶여 있는 것 아닌가요? (Aren't Slack Variables Tied to Constraints?)

**질문:**
> 여유 변수는 본질적으로 제약 조건과 관련되어 있지 않나요? 왜 분리되어 있는 건가요?

### 답변: 이론적 통합 vs. 수치적 분리 (Theoretical Unity vs. Numerical Separation)

이론적으로는 질문이 100% 맞습니다. 그러나 종이 위의 수학이 **C++ 수치 솔버(Numerical Solver)** 로 변환되는 순간, 이들은 완전히 독립적인 변수들로 이혼(?)하게 됩니다.

=== "1. 솔버 구조: 차원 격상 (Solver Structure: Dimensional Lifting)"

    만약 여유 변수가 제약 조건에 완전히 묶여 있다면, 솔버는 단순히 $s$ 대신 $-c(x)$를 대입하여 다음을 풀면 됩니다:
    $$-\mu \log(-c(x))$$
    
    그러나 $c(x)$가 비선형 함수라면, 비선형 함수를 로그 함수 안에 가두게 되어 자코비안과 헤시안 행렬이 엄청나게 꼬이게 됩니다. $25\text{ms}$ 안에는 도저히 풀 수 없는 끔찍한 연산이 되어버립니다.
    
    대신 IPM 설계자들은 다르게 접근합니다: **"여유 변수 $s$를 $x$의 종속 변수에서 $x$와 동급인 '완전히 독립적인 최적화 변수'로 격상시키자!"**
    
    - **원래 변수**: $x$ (차량 상태, 조향 등)
    - **IPM 변수**: $Z = [x, s, \lambda]^\top$
    
    이제 솔버의 눈에는 $x$와 $s$가 단지 배열 인덱스 `Z[0]`과 `Z[1]`일 뿐입니다. 그들은 완전히 독립적으로 움직일 수 있는 권리를 얻었습니다.

=== "2. 선형화의 배신 (선형화 오차) (The Betrayal of Linearization / Linearization Error)"

    이 두 개의 독립 변수 $x$와 $s$를 가지고, 솔버는 다음 스텝을 위해 뉴턴 방법(Newton's Method)을 사용합니다. 뉴턴 방법은 복잡한 곡선의 세계를 "직선"(접선)으로 무식하게 단순화하여 예측합니다.
    
    솔버는 연립 방정식을 풀어 독립적인 스텝($\Delta x$, $\Delta s$)을 계산합니다:
    
    - $x_{\text{new}} = x + \alpha \Delta x$
    - $s_{\text{new}} = s + \alpha \Delta s$
    
    여기서 비극이 발생합니다:
    
    - $s$는 원래 가상 변수이므로, 예측한 대로 완벽하게 **직선**으로 움직입니다.
    - 그러나 $x$의 움직임에 의해 생성된 물리적 현실 $c(x_{\text{new}})$는 **비선형 곡선**입니다.
    
    직선 경로를 가정하고 솔버는 큰 스텝($\alpha$)을 밟습니다. 하지만 실제 물리적 세계 $c(x)$는 곡선으로 꺾여 나가면서, 두 독립 변수 사이의 $c(x_{\text{new}}) + s_{\text{new}} = 0$ 이라는 약속을 깨버립니다.

=== "3. L1 페널티의 진정한 역할: 수학적 고무줄 (The True Role of L1 Penalty: A Mathematical Rubber Band)"

    요약하자면, **여유 변수 $s$는 본질적으로 제약 조건 $c(x)$에서 태어났지만, 최적화 속도를 높이기 위해 솔버 내부에서는 '독립적인 자유 변수'로 풀려납니다.**
    
    이렇게 풀려난 두 변수($c(x)$와 $s$)가 선형화 오차로 인해 서로 찢어지려 할 때, 이들을 강제로 묶어주는 **무자비한 수학적 고무줄**이 바로 L1 페널티 항입니다:
    
    $$w_{L1} |c(x_{\text{new}}) + s_{\text{new}}|$$
