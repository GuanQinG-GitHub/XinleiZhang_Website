# DeepReach: A Neural PDE Solver for HJ PDE

## Neural Network Parameterization

DeepReach approximates the value function of a Hamilton–Jacobi–Isaacs variational inequality (HJI-VI) using a neural network:

$$
V(t,x) \approx V_\theta(t,x),
$$

where $x \in \mathbb{R}^n$ is the system state, $t$ is the time variable, and $\theta$ denotes the neural network parameters.

The key idea is to train $V_\theta$ by minimizing the HJI-VI residual, without requiring ground-truth values of $V(t,x)$ as supervised labels.

## Constructing the PDE Residual Loss

Consider the HJI-VI for a backward reachable tube:

$$
\max\left\{
    \frac{\partial V}{\partial t} - H(x,\nabla_x V),\;
    V(t,x)-l(x)
\right\}=0,
$$

subject to the initial condition

$$
V(0,x)=l(x),
$$

where $H$ is the Hamiltonian associated with the system dynamics and $l(x)$ is the implicit surface function defining the target set.

At each training iteration, we sample a batch of state-time pairs

$$
\{(t_i,x_i)\}_{i=1}^{N}.
$$

For each sampled pair, the neural network provides $V_\theta(t_i,x_i)$. We then compute its derivatives with respect to the inputs:

$$
\frac{\partial V_\theta}{\partial t},
\qquad
\nabla_x V_\theta.
$$

**Remark 1:** Both $D_{t}V_{\theta}$ and $D_{x}V_{\theta}$ are still functions of $\theta$, so we can simply forward-evaluate these derivatives with the current network parameters $\theta$ at the sample points.

Substituting these quantities into the HJI-VI yields the residual

$$
r_\theta(t_i,x_i)
=
\max\left\{
    \frac{\partial V_\theta}{\partial t}(t_i,x_i) - H\big(x_i,\nabla_x V_\theta(t_i,x_i)\big),\;
    V_\theta(t_i,x_i)-l(x_i)
\right\}.
$$

The empirical PDE residual loss can be defined as

$$
\mathcal{L}_{\mathrm{PDE}}(\theta)
=
\frac{1}{N}\sum_{i=1}^{N} |r_\theta(t_i,x_i)|.
$$

The initial condition must also be satisfied. This can be achieved either by adding an initial-condition penalty,

$$
\mathcal{L}_{\mathrm{IC}}(\theta)
=
\frac{1}{N_0}\sum_{j=1}^{N_0} |V_\theta(0,x_j)-l(x_j)|,
$$

or by adopting a parameterization that satisfies it exactly, such as

$$
V_\theta(t,x)=l(x)+t\,N_\theta(t,x),
$$

where $N_\theta$ denotes the neural network output, possibly including appropriate normalization factors. In the exact-initial-condition formulation, the initial condition holds for every $\theta$, so no separate initial-condition penalty is necessary.

## Computing the Gradient with Respect to Network Parameters

The central optimization objective is

$$
\min_\theta \mathcal{L}(\theta).
$$

Although the loss is ultimately a function of $\theta$, it depends explicitly on the value function and its derivatives. For a fixed sampled state-time pair, we can write

$$
\mathcal{L}(\theta)
=
F\left(
    V_\theta,\;
    \frac{\partial V_\theta}{\partial t},\;
    \nabla_x V_\theta
\right),
$$

where $F$ represents the loss constructed from the HJI-VI residual.

To optimize $\theta$, we need the gradient $\nabla_\theta \mathcal{L}$. By the chain rule, wherever the relevant derivatives exist,

$$
\nabla_\theta \mathcal{L}
=
\frac{\partial F}{\partial V_\theta}\,\nabla_\theta V_\theta
+
\frac{\partial F}{\partial(\partial_t V_\theta)}\,\nabla_\theta(\partial_t V_\theta)
+
\sum_{j=1}^{n}
\frac{\partial F}{\partial(\partial_{x_j}V_\theta)}\,\nabla_\theta(\partial_{x_j}V_\theta).
$$

This expression shows that the gradient of the loss with respect to $\theta$ depends on three types of quantities:

1. The sensitivity of the value function to the parameters:

    $$
    \nabla_\theta V_\theta.
    $$

2. The sensitivity of its time derivative to the parameters:

    $$
    \nabla_\theta(\partial_t V_\theta) = \frac{\partial^2 V_\theta}{\partial\theta\,\partial t}.
    $$

3. The sensitivity of its spatial derivatives to the parameters:

    $$
    \nabla_\theta(\partial_{x_j}V_\theta) = \frac{\partial^2 V_\theta}{\partial\theta\,\partial x_j}.
    $$

The latter two quantities are mixed second-order derivatives of the neural value function with respect to its inputs and parameters. Because $V_\theta$ is represented by a differentiable neural network, these derivatives can be evaluated through repeated applications of the chain rule.

## Updating the Neural Network Parameters

Once $\nabla_\theta \mathcal{L}$ is obtained, the network parameters are updated using a gradient-based optimizer such as Adam. For illustration, a standard gradient descent update is

$$
\theta_{k+1} = \theta_k - \eta\,\nabla_\theta \mathcal{L}(\theta_k),
$$

where $\eta$ denotes the learning rate.

Adam follows the same gradient-based optimization principle, but uses adaptive parameter updates based on estimates of the first and second moments of the gradients.

The training procedure repeatedly samples state-time pairs, evaluates the current neural value function and its derivatives, constructs the HJI-VI residual loss, and updates the parameters.

## Why DeepReach Uses SIREN Networks

DeepReach adopts sinusoidal representation networks (SIRENs), which use sinusoidal activation functions:

$$
\sigma(z)=\sin(\omega_0 z).
$$

The motivation is not merely to make the neural network differentiable, since other smooth activation functions also provide differentiability.

Instead, SIRENs offer useful representational properties for approximating complex functions *and their derivatives*. This is particularly relevant to DeepReach because the HJI-VI residual explicitly depends on the temporal and spatial derivatives of the neural value function (Remark 1).

## Summary of the DeepReach Algorithm

The high-level training procedure can be summarized as follows:

1. Initialize the neural network parameters $\theta$.
2. Sample a batch of state-time pairs $(t_i,x_i)$.
3. Evaluate the current neural value function $V_\theta(t_i,x_i)$.
4. Compute its temporal and spatial derivatives: $\partial_t V_\theta$ and $\nabla_x V_\theta$.
5. Substitute these quantities into the HJI-VI to construct the PDE residual and training loss.
6. Compute the parameter gradient $\nabla_\theta\mathcal{L}$ using the chain rule, including mixed derivatives where required.
7. Update $\theta$ using Adam.
8. Repeat until the chosen training or convergence criterion is reached.

## Key Insight

DeepReach converts the problem of solving a high-dimensional HJI-VI into an optimization problem over neural network parameters:

$$
\boxed{
\begin{gathered}
    \theta
    \longrightarrow
    \left(V_\theta,\, \partial_t V_\theta,\, \nabla_x V_\theta\right)
    \longrightarrow
    \mathcal{L}(\theta)
    \\[6pt]
    \longrightarrow
    \nabla_\theta\mathcal{L}
    \xrightarrow{\mathrm{Adam}}
    \theta_{\mathrm{new}}.
\end{gathered}
}
$$
