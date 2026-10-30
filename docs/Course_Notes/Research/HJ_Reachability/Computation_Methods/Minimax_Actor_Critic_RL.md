# Learning the Reachability Value Function via Minimax Actor–Critic Reinforcement Learning

## Problem Formulation

Consider a discrete-time dynamic system

$$
x_{k+1}=f(x_k,u_k,d_k),
$$

where $x_k\in\mathcal X$ is the system state, $u_k\in\mathcal U$ is the control input, and $d_k\in\mathcal D$ is the adversarial disturbance. Let $g:\mathcal X\rightarrow\mathbb R$ be a safety function, with the safe set defined as

$$
\mathcal K=\{x\in\mathcal X:g(x)\geq 0\}.
$$

The objective of the controller is to maximize the worst-case safety margin over an infinite horizon, while the disturbance attempts to minimize it. Therefore, the optimal safety value function is defined as

$$
\boxed{
V^*(x) = \sup_{\pi}\inf_{\phi}\inf_{k\geq 0} g(x_k^{\pi,\phi}),
}
$$

where $\pi$ and $\phi$ denote the control and disturbance strategies, respectively.

Under suitable assumptions on the dynamics and admissible strategies, the robust viability kernel is

$$
\mathcal K^* = \{x:V^*(x)\geq 0\}.
$$

By the dynamic programming principle, the value function satisfies the Bellman equation

$$
\boxed{
V^*(x)
=
\min\left\{
    g(x),\;
    \max_{u\in\mathcal U}\min_{d\in\mathcal D} V^*(f(x,u,d))
\right\}.
}
$$

Our goal is to learn an approximation of $V^*$ using reinforcement learning.

## Action-Value Function and Neural Parameterization

### Why Introduce a Q Function?

The Bellman equation involves the optimization

$$
\max_u\min_d V^*(f(x,u,d)).
$$

Instead of learning the value function alone, we introduce an action-value function

$$
\boxed{
Q^*(x,u,d) = \min\left\{ g(x),\; V^*(f(x,u,d)) \right\}.
}
$$

To understand this definition, suppose that the initial state is $x$ and the first control and disturbance actions are fixed at $(u,d)$. Then, the next state is

$$
x^+=f(x,u,d).
$$

The worst safety margin over the entire trajectory is the minimum between the current safety margin $g(x)$ and the optimal future safety margin $V^*(x^+)$, which gives precisely the definition of $Q^*$. Consequently,

$$
\boxed{
V^*(x)=\max_u\min_d Q^*(x,u,d).
}
$$

**Remark:** The Q function evaluates the long-term safety associated with a prescribed first control-disturbance pair, whereas the value function also optimizes that first pair.

### Three Neural Networks

We introduce three neural networks:

1. A control Actor:

    $$
    u=\pi_{\theta_u}(x).
    $$

2. A disturbance Actor:

    $$
    d=\phi_{\theta_d}(x).
    $$

3. A Critic:

    $$
    Q_{\theta_q}(x,u,d).
    $$

The control Actor approximately maximizes the Q value, while the disturbance Actor approximately minimizes it. Therefore, the learned value function is obtained from

$$
\boxed{
V_\theta(x)
=
Q_{\theta_q}\left(x,\; \pi_{\theta_u}(x),\; \phi_{\theta_d}(x)\right),
}
$$

where $\theta=(\theta_q,\theta_u,\theta_d)$. Thus, instead of solving the minimax optimization explicitly at every state, the two Actor networks learn approximate optimizing actions.

## Critic Learning via Bellman Bootstrapping

The Critic is trained using sampled transitions and the Bellman equation.

### Step 1: Collect Transitions

Using the current control and disturbance policies, we generate trajectories of the system. For a sampled state $x_i$, the policies generate

$$
u_i=\pi_{\theta_u}(x_i),
\qquad
d_i=\phi_{\theta_d}(x_i),
$$

possibly with additional exploration noise. The environment produces the next state

$$
x_{i+1}=f(x_i,u_i,d_i).
$$

We obtain a transition

$$
\tau_i = (x_i,\, u_i,\, d_i,\, x_{i+1},\, g(x_i)).
$$

**Remark:** Model free?

Note that the training mini-batch can contain transitions sampled from different trajectories.

### Step 2: Evaluate the Current Critic

For a sampled transition, the current Critic predicts

$$
\boxed{
q_i=Q_{\theta_q}(x_i,u_i,d_i).
}
$$

Recall that the optimal Q function is defined as

$$
Q^*(x_i,u_i,d_i) = \min\left\{ g(x_i),\; V^*(x_{i+1}) \right\}.
$$

Therefore, the training goal for the Critic network is to satisfy the above definition.

### Step 3: Construct the Bellman Target

We estimate the value at the next state using separate copies of the learned policies and Critic (target networks) for numerical stability. Define

$$
\widehat V(x_{i+1})
=
Q_{\bar\theta_q}\left(x_{i+1},\; \pi_{\bar\theta_u}(x_{i+1}),\; \phi_{\bar\theta_d}(x_{i+1})\right).
$$

The Bellman target is then

$$
\boxed{
y_i = \min\left\{ g(x_i),\; \widehat V(x_{i+1}) \right\}.
}
$$

### Step 4: Compute the Critic Loss

For a mini-batch containing $N$ transitions, define the temporal-difference loss

$$
\boxed{
\mathcal L_Q(\theta_q)
=
\frac{1}{N}\sum_{i=1}^{N}
\left( Q_{\theta_q}(x_i,u_i,d_i)-y_i \right)^2.
}
$$

The gradient is

$$
\nabla_{\theta_q}\mathcal L_Q
=
\frac{2}{N}\sum_{i=1}^{N}
\left( Q_{\theta_q}(x_i,u_i,d_i)-y_i \right)
\nabla_{\theta_q}Q_{\theta_q}(x_i,u_i,d_i).
$$

We then update the Critic parameters using Adam or another gradient-based optimizer:

$$
\theta_q^{k+1} = \operatorname{OptimizerStep}\left( \theta_q^k,\; \nabla_{\theta_q}\mathcal L_Q \right).
$$

## Control Actor Learning

After updating the Critic, we improve the control policy using the current Q-function approximation. The objective of the controller is to maximize the predicted long-term safety value:

$$
\max_{\theta_u}\;
\mathbb E_{x\sim\mathcal D}
\left[ Q_{\theta_q}\left( x,\pi_{\theta_u}(x),\phi_{\theta_d}(x) \right) \right],
$$

where $\mathcal D$ denotes the sampled state distribution. Equivalently, define the control Actor loss

$$
\boxed{
\mathcal L_u(\theta_u)
=
-\frac{1}{N}\sum_{i=1}^{N}
Q_{\theta_q}\left( x_i,\; \pi_{\theta_u}(x_i),\; \phi_{\theta_d}(x_i) \right).
}
$$

During this update, the Critic and disturbance Actor parameters are held fixed. By the chain rule,

$$
\boxed{
\nabla_{\theta_u}\mathcal L_u
=
-\frac{1}{N}\sum_{i=1}^{N}
\frac{\partial Q_{\theta_q}}{\partial u}
\frac{\partial\pi_{\theta_u}(x_i)}{\partial\theta_u}.
}
$$

## Disturbance Actor Learning

The disturbance Actor is trained to minimize the predicted safety value. Its objective is

$$
\min_{\theta_d}\;
\mathbb E_{x\sim\mathcal D}
\left[ Q_{\theta_q}\left( x,\pi_{\theta_u}(x),\phi_{\theta_d}(x) \right) \right].
$$

Define

$$
\boxed{
\mathcal L_d(\theta_d)
=
\frac{1}{N}\sum_{i=1}^{N}
Q_{\theta_q}\left( x_i,\; \pi_{\theta_u}(x_i),\; \phi_{\theta_d}(x_i) \right).
}
$$

During this update, the Critic and control Actor parameters are held fixed. The corresponding gradient is

$$
\boxed{
\nabla_{\theta_d}\mathcal L_d
=
\frac{1}{N}\sum_{i=1}^{N}
\frac{\partial Q_{\theta_q}}{\partial d}
\frac{\partial\phi_{\theta_d}(x_i)}{\partial\theta_d}.
}
$$

## Alternating Optimization

The Critic and Actors are not trained in completely independent stages. Instead, their parameters are updated alternately. At each training iteration:

1. Collect transitions using the current policies and store them in the data buffer.
2. Sample a mini-batch of transitions from the data buffer.
3. Construct Bellman targets using the target Critic and target Actors.
4. Update $\theta_q$ by minimizing $\mathcal L_Q$, while holding the Actor parameters fixed.
5. Update $\theta_u$ by minimizing $\mathcal L_u$, while holding the Critic and disturbance Actor parameters fixed.
6. Update $\theta_d$ by minimizing $\mathcal L_d$, while holding the Critic and control Actor parameters fixed.
7. Slowly update the target networks, for example using

    $$
    \bar\theta \leftarrow (1-\tau)\bar\theta+\tau\theta,
    \qquad 0<\tau\ll1.
    $$

8. Repeat the procedure.
