# Hamilton–Jacobi Reachability Problem

*2026-10-27*

## System Dynamics and State Constraints

We consider the following nonlinear disturbed system dynamics defined over an infinite horizon:

$$
\mathrm{x}_{x}^{\mathrm{u},\mathrm{d}}(0)=x,\quad
\dot{\mathrm{x}}_{x}^{\mathrm{u},\mathrm{d}}(t)=f\bigl(\mathrm{x}_{x}^{\mathrm{u},\mathrm{d}}(t),\mathrm{u}(t),\mathrm{d}(t)\bigr),
\quad t\in(0,\infty),
$$

where $\mathrm{x}(t)\in\mathbb{R}^{n}$ is the state at time $t$ and $x$ is the initial state. The control input is $\mathrm{u}(t)\in U$, and the disturbance is $\mathrm{d}(t)\in D$, where $U$ and $D$ are compact sets. The control $\mathrm{u}\in\mathcal{U}$ and disturbance signal $\mathrm{d}\in\mathcal{D}$ are measurable functions from $[0,\infty)$ to $U$ and $D$, respectively. We assume that $f$ is locally Lipschitz in $x$, so $\mathrm{x}_{x}^{\mathrm{u},\mathrm{d}}(\cdot)$ uniquely exists on its maximal interval of existence. The disturbance $\mathrm{d}$ captures, e.g., modeling error, external perturbations, or uncertain controls of other agents.

We consider Stackelberg zero-sum games to model the interaction between control and disturbance signals, where the disturbance signal $\mathrm{d}$ is reactive to the control signal $\mathrm{u}$ in the sense of a non-anticipative strategy, i.e., $\mathrm{d}=\delta[\mathrm{u}]$, where $\delta\in\Delta:\mathcal{U}\to\mathcal{D}$ denotes a non-anticipative disturbance strategy. This non-anticipative strategy prevents the disturbance signal from exploiting future information about the control input.

In safety-critical systems, the state is subject to hard constraints, such as collision avoidance. Let $g:\mathbb{R}^{n}\to\mathbb{R}$ be a bounded Lipschitz continuous function, and define the safety constraint set

$$
\mathcal{X}:=\{x\in\mathbb{R}^{n}\mid g(x)\le 0\}.
$$

A trajectory is called *safe* if it remains in $\mathcal{X}$ for all time.

To analyze trajectory safety, we start by characterizing a robust control invariant set (RCIS) within $\mathcal{X}$. Formally, a set $\mathcal{S}\subseteq\mathcal{X}$ is called an RCIS if, for every $x\in\mathcal{S}$ and all admissible non-anticipative disturbance strategies, there exists a control signal $\mathrm{u}\in\mathcal{U}$ such that the corresponding trajectory remains in $\mathcal{S}$, i.e.,

$$
\forall\,\delta\in\Delta,\,\exists\,\mathrm{u}\in\mathcal{U}\,\text{ s.t. }\,
\mathrm{x}^{\mathrm{u},\delta[\mathrm{u}]}_{x}(t)\in \mathcal{S},\,
\forall\,t\ge 0.
$$

The maximal RCIS contained in $\mathcal{X}$ is the viability kernel (VK), denoted by

$$
\mathcal{V} := \bigl\{
    x
    \,\big|\,
    \forall\,\delta\in\Delta,\,\exists\,\mathrm{u}\in\mathcal{U}\,\text{ s.t. }\,
    \mathrm{x}^{\mathrm{u},\delta[\mathrm{u}]}_{x}(t)\in\mathcal{X},\,
    \forall\,t\ge 0
\bigr\}.
$$

## HJ Reachability Problem

HJ reachability analysis provides a principled framework for characterizing the VK and devising robust safety filters by formulating the problem as a Stackelberg zero-sum game:

$$
V_{\gamma}(x):=\sup_{\delta\in\Delta}\inf_{\mathrm{u}\in\mathcal{U}}
J_{\gamma}\bigl(x,\mathrm{u},\delta[\mathrm{u}]\bigr),
$$

where the cost $J_{\gamma}$ is defined as

$$
J_{\gamma}(x,\mathrm{u},\mathrm{d})
:= \sup_{t\in[0,\infty)} e^{-\gamma t}\,
    g\bigl(\mathrm{x}^{\mathrm{u},\mathrm{d}}_{x}(t)\bigr),
$$

with a discount factor $\gamma > 0$. Under this formulation, the safe set is defined as the zero-level set of the value function, i.e., $\mathcal{V} = \{x \mid V_{\gamma}(x)=0 \}$.

By the principle of dynamic programming, $V_{\gamma}$ is the unique bounded continuous viscosity solution of the following HJI-VI:

$$
0=
\max\Bigl\{
    g(x)-V_{\gamma}(x),\;
    \min_{u\in U}\max_{d\in D}
    \nabla V_{\gamma}(x)\cdot f(x,u,d)-\gamma V_{\gamma}(x)
\Bigr\}.
$$

Therefore, solving the HJ reachability problem amounts to solving for the value function $V_{\gamma}(x)$ in the above HJI-VI. This HJI-VI is typically solved by grid-based level-set methods based on dynamic programming. Since the associated Bellman operator is provably a contraction mapping, one can start with any initialization, not necessarily $g(x)$.
