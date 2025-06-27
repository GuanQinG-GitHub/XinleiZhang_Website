High-level idea:
- Roughly speaking, controllability studies the possibility of steering the state from the input; observability studies the possibility of estimating the state from the output.
- These two concepts are defined under the assumption that the state-space equation or, equivalently, the system model is fully known. Thus, the problems of controllability or observability is different from the problem of realization or identification, which is to determine or estimate the system model from the information collected at the input and output terminals.


1. Definition of controllability
    >
    >Consider the $n$-dimensional $p$-input state equation
        >$$
        >\dot{\bold{x}}(t)=\bold{A}\bold{x}(t) + \bold{B}\bold{u}(t)
        >$$ 
    >where $\bold{A}$ and $\bold{B}$ are, respectively, $n \times n$ and $n \times p$ real cosntant matrices. The pair $(\bold{A},\bold{B})$ is said to be *controllable* if for any initial state $\bold{x}(0)=\bold{x}_{0}$ and any final state $\bold{x}_{1}$, there exists an input that transfers $\bold{x}_{0}$ to $\bold{x}_{0}$ in a finite time. Otherwise, the pair $(\bold{A},\bold{B})$ is said to be *uncontrollable*.
    
    Comments:
        a) what trajectory the state take is not specified, as long as the final state reach the goal within a finite time;
        b) no constraint imposed on the input, i.e., its magnitude can be as large as desired. 

2. Theorems to determine the controllability
3. Examples
4. Minimal energy control
5. Redundant control
6. Invariance of controllability
7. Definition of observability
    >
    >Consider the $n$-dimensional $p$-input $q$-output state equation
        >$$
        >\dot{\bold{x}}(t)=\bold{A}\bold{x}(t) + \bold{B}\bold{u}(t)\\
        >\bold{y}(t) = \bold{C}\bold{x}(t) + \bold{D}\bold{u}(t)
        >$$ 
    >The system described by the state-space equation above is said to be *observable* if for any unknown initial state $\bold{x}(0)$, there exists a finite time $t_{1}$ such that the knowledge of the input $\bold{u}$ and the output $\bold{y}$ over $[0,t_{1}]$ suffices to determine uniquely the initial state $\bold{x}(0)$. Otherwise, the system is said to be *unobservable*.

8. Theorems to determine the observability
9.  Exampels