# Review Outline for System Dynamics

### Chapter 1 Complex Number

1. Rectangular form and polar form
    - $z=a+bj$ (a,b) are the coordinate in complex plane
    - $z=|z|e^{j\theta},|z|=\sqrt{a^2+b^2},\theta=arctan(\frac{b}{a})$
2. Euler’s Formula
    - $e^{j\theta}=cos(\theta)+jsin(\theta)$
3. Complex operations
    - Complex conjugate: $z^*=a-bj=|z|e^{-j\theta}$
    - Addition and Subtraction
    - Multiplication: $z_1z_2=|z_1||z_2|e^{j(\theta_1+\theta_2)}$
    - Division: $\frac{z_1}{z_2}=\frac{|z_1|}{|z_2|}e^{j(\theta_1-\theta_2)}$
    - Inverse: $z^{-1}=|z|^{-1}e^{-j\theta}$
4. Complex variable and complex function
    - $s=\sigma+j\omega,~\sigma~and~\omega\in R$
    - $F(s)=F_x+jF_y$, $F_x$ and $F_y$ are real functions
5. zeros and poles of a complex function
    - $F(s)=\frac{k(s+z_1)(s+z_2)...(s+z_m)}{(s+p_1)(s+p_2)...(s+p_n)}$
    - zeros: $-z_1,-z_2,...,-z_m$
    - poles: $-p_1,-p_2,...,-p_n$
    - **In practice, the number of zeros is always less than or equal to the number of poles**, $m\le n$

### Chapter 2 Laplace Transform

1. Definition of Laplace Transform (L.T.)
2. Existence criteria
3. Unit step function
    - Used to transform the signal into L.T. applicable form
4. Laplace Transform table
5. Time delay and damping effect
6. Superposition of Laplace Transform
    - Used to decompose the general input signal into typical forms easier for L.T.
7. Solving Differential Equation
    - From time domain to Laplace domain, back to time domain
8. Final Value Theorem
    - Used to compute the steady-state value of the output

### Chapter 3 Partial Fraction Expansion (Work for L.T.)

$$
X(s)=\frac{b_0+b_1s+...+b_ms^m}{a_0+a_1s+...+a_ns^n}=\frac{B(s)}{A(s)}
$$

1. Work by hand
    - $m\lt n$
        - Distinct poles
        - Repeated poles
    - $m\ge n$
        - Distinct poles
        - Repeated poles
2. Matlab
    - **Matlab Convention**
        - The power of polynomials are in descending form and the power of poles are in ascending form

### Chapter 4 Mechanical system   All models are wrong but some are useful

1. Spring in series and paraller
2. Spring-mass system
    - EOM
    - L.T.
3. Spring-mass-damper system
    - EOM
    - L.T.
4. Characteristic equation
    - Solution in three situations
        - Complex Conjugates
        - One real root
        - Two real roots
    - Damping Ration
        - Underdamped (Exponentially decaying sinusoid)
        - Critically damped (Exponential decaying)
        - Overdamped (Exponential decaying)
    - Natural frequency and damped natural frequency

### Chapter 5 Transfer Function

1. Transfer function (TF)
    - Definition
        - $TF(s)=G(s)=\frac{Y(s)}{U(s)}=\frac{output}{input}$
        - Suppose zero initial condition
        - Only applicable to linear, constant coefficient, differential equation
    - Block Diagram (a graphical representation of TF)
        - elements: block, constant, summing point…

### Chapter 6 Transient and Steady-State Response

1. Two aspects of the construction of response
    - Transient repsonse (due to initial condition and input) + steady-state response (due to input)
    - Free vibration (dut to initial condition) + Forced vibration (due to input)
2. Find the steady-state response using Final Value Theorem

### Chapter 7 State-Space Model

1. Definition of state-space model
    - $\dot{x}=Ax+Bu,~y=Cx+Du$
    - Applicable to nonlinear, time-varying differential equation
2. Example of a 2 DOF mechanical system
3. Block diagram of state-space model
4. Transfer function of state-space model
    - $G(s)=C(sI-A)^{-1}B+D=\frac{Y(s)}{U(s)}$

### Chapter 8 Electrical Systems

1. Basic elements
    - Capacitor
        - $C=\frac{q}{e_c}$
        - $i=C\frac{de_c}{dt}$
    - Inductor
        - $L=\frac{e_L}{di/dt},~e_L=L\frac{di}{dt}$
2. Example: RL circuit with step input voltage

### Chapter 9 DC Motor

- Modelling Process
    - Electrical System
        
        $L_a\dot{i_a}+R_ai_a+e_b=e_a$ (KVL)
        
        $e_b=k_b\dot{\theta}$ (back emf)
        
    - Motor Mechanical System
        
        $J\ddot{\theta}+b\dot{\theta}=T-T_L$ (EOM)
        
        $T=ki_a$ (Physical Law)
        
    - Load
        
        $J_L\ddot{\theta}+b_L\dot{\theta}=T_L$ (EOM)
        
    - Gear (optional)
        
        Ideal Gear: $T_2=\frac{r_2}{r_1}T_1$ (Conservation of Power)
        
        General: $J_1\frac{r_2}{r_1}\ddot{\theta_1}+b_1\frac{r_2}{r_1}\dot{\theta_1}+J_2\ddot{\theta_2}+b_2\dot{\theta_2}=\frac{r_2}{r_1}T_1-T_2$ (EOM)
        
    - Coupler (optional)
        
        $T_C=k_{tc}(\theta-\theta_L)$
        

### Chapter 10 Fluid System

1. Reynold Number
    - Inertial force vs Viscous force
    - Turbulent vs Laminar
2. Valve equation (Relationship between head H and flow rate Q)
    - Laminar
    - Turbulent
3. Capacitance
    - Conservation of Volume
4. Example: Tank system
    - Equation 1: conservation of volume
    - Equation 2: valve equation

### Chapter 11 Linearization

1. Method: First order Taylor Series Expansion
2. Example: Used in single tank system with turbulent in vavle

### Chapter 12 Time Domain Analysis

1. First order ODE system $a\ddot{x}+b\dot{x}=0$
    - Time constant T: $T=\frac{a}{b}$
    - Typical values
        - 0.37 at T, 0.83 at 2T, 0.95 at 3T, 0.98 at 4T
2. Second order ODE system
    - only for underdamped system
    - Time constant for the Exponential Decay Envelop
        - $T=\frac{1}{\xi \omega_n}$
    - Experimentally determine the dample natural frequency and damping ration
        - $T=\frac{2\pi}{\omega_d},~ln(\frac{x_1}{x_2})=\frac{2\pi\xi}{\sqrt{1-\xi^2}}$
    - Some terminology
        - Settling time-converge to 2% or 5%
        - Overshoot $\%M_p\frac{x_{peak}-x_{ss}}{x_{ss}}$
        - Peak time-rise to peak
        - Delay time-reach 50% of the steady-state value
        - Rise time-from 10% to 90% of the steady-state value

### Chapter 13 Frequency Domain Analysis

1. Steady-state response of a system to a sinusoidal input
    - $u(t)=Psin(\omega t),~x_{ss}=Xsin(\omega t+\phi)$
    - $X$ and $\phi$ are determined by $\omega$
2. Forced vibration without damping
3. Forced vibration with damping
4. Sinusoidal transfer function (Frequency response function)
    - step 1: obtain transfer function $G(s)$
    - step 2: let $s$ equal to $jw$
    - step 3: $\frac{x_0}{P}=|G(jw)|,~\phi=\angle G(jw)$

### Chapter 14 Bode Plot

1. Decibel (dB)
    
    $AR_{dB}=20log(|G(jw)|)=20log(AR)$
    
2. Example

### Chapter 15 Vibration Isolation

1. Imbalance in rotatin mechanical systems
2. Vibration isolation
    
    $G(jw)=\frac{1+j(2\xi\Omega)}{(1-\Omega^2)+j(2\xi\Omega)},~\Omega=\frac{\omega}{\omega_n}$
    
    - Force excitation
    - Motion excitation
    - Transimissibility TR
        - $TR\lt1~when~\Omega\gt\sqrt{2}$

### Chapter 16 Vibration Absorber

- Condition for zero vibration in main system
    
    $\sqrt{\frac{k_a}{m_a}}=\omega_n$
    

### Chapter 17 Mode Shape

- Example for free vibration of 2 DOF system

### Chapter 18 PID

1. Block diagram of feedback control system
2. PID controller
3. General conclusion
4. Criterion for stability of PID controller

### Chapter 19 Performance of second order system

1. Peak Time
2. Percent Overshoot
3. Settling Time

### Chapter 20 Stability

1. Stable
2. Marginally stable
3. Unstable
