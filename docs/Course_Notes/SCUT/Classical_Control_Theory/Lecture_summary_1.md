# Lecture Review 1

## What’s control system and why control system?

## History of control problem

## How to design a control system

### Step 1: Modeling a system/process/plant

> What is a system/process/plant?

#### Modeling method 1: Modeling Method in Mathematics

For a real world object, process or anything we want to control, we call it a system. For a system, we can describe it mathematically, i.e. using equations to represent the system. Three examples are given as follows.

- Example 1: for a classical physical system, we can model it using Newton’s laws, e.g., $m\ddot{x}+c\dot{x}+kx=u.$
- Example 2: for an electrical system, we can model it using Ohm’s law and Kirchhoff’s laws,
    
- Example 3: for a fluid system,

More specifically, the mathematical equation in example 1 is an **O**rdinary **D**ifferential **E**quation (ODE). In example 2, one ODE and one **A**lgebraic **E**quation (AE) compose the equation system. In example 3, a **P**artial **D**ifferential **E**quation (PDE) is used to model the system.

> Trade-off in modeling mathematically: Accuracy vs Simplicity


With the language of system, there are always three important aspects, i.e., **input, output** and **system parameters**. As for the example 1 above, we impose a force $u$ and concern the displacement $x$. Then, the displacement $x$ could be regarded as the system output while the force $u$ could be regarded as the system input. Quantities as $m,c,k$ are the system intrinsic property, defined as the system parameters. The concept of system is usually depicted with the block diagram.

#### Categorization of systems

##### Linear system vs Nonlinear system

A system is called a linear system if the **principle of superposition** applies. If not, the system is a nonlinear system. The principle of superposition states that the output of the system produced by the simultaneous application of two input is the sum of the two individual output. 

##### Time varying system vs Time invariant system

A time invariant system is described by an equation whose coefficients are all constants, not the function of time. In the opposite, at least one coefficient is the function of time in the equation describing a time varying system. The rising of a rocket is a typical example of time varying system (varying mass due to ejection and combustion).

##### Continuous system vs Discrete system

##### Single input single output system vs Multiple input multiple output system

If we concern several quantities in a system, we may want the system to have multiple output, which means that a vector but not a scalar will be used to represent the system output. 

#### Modeling methods in control theory

##### Modeling method 1: Transfer Function

##### Modeling method 2: System Identification

##### Modeling method 3: Computer-aided

### Step 2: Evaluate the modeled system

#### Evaluation aspect 1: Response

#### Evaluation aspect 2: Stability

#### Evaluation aspect 3: Controllability

#### Evaluation aspect 4: Observability

## References
[1] 