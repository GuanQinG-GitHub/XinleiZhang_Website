## References

## Motivation
- Solve the multi-stage optimization problem

## DP in finite-horizon discrete-time problem

### Principle of Optimality

### DP algorithm
- Backward and forward computation
- Proof of the algorithm

***Remarks***
- Value function

### Example: double integrator

#### Results

<video controls>
<source src="../Animation_of_double_integrator(201*201).mp4" type="video/mp4">
</video>

***Example Code***
```matlab
% Copied from DP_example_code.m

% Backward computation: value function and Q function
% step 1. V(x,K) = g(x)
V(:,:,K) = g(X1_grid, X2_grid);
% step 2. V(x,k) = min_{a \in A} L(x,a) + V(f(x,a),k+1) 
for k = K-1:-1:1
    % define an interpolation function V(:,:,k+1)
    VNext_interpolation_Func = griddedInterpolant(X1_grid, X2_grid, V(:,:,k+1));
    % Note: if you want to find V(0.5,0.4,k+1) = VNext_interpolation_Func(0.5,0.4)
    % Q-value: Q(x1,x2,a,k) = L(x,a) + V(f(x,a),K+1) 
    for aa = 1:length(A) % Q(x1,x2,k){aa}
        % compute the next state 
        NextState1 = next_x1(X1_grid, X2_grid, A(aa));
        NextState2 = next_x2(X1_grid, X2_grid, A(aa));

        % Q(x1,x2,a,k) = L(x1, x2, a) + V(f(x,a),K+1) 
        Q(:,:,aa,k) = L(X1_grid, X2_grid, A(aa)).*dt + ...
            VNext_interpolation_Func( NextState1, NextState2 );
    end
    % V(:,:,k) = % min_{a \in A} L(x,a) + V(f(x,a),K+1) = min_a Q(x1,x2,a,k)
    % compute V value from Q functions
    V(:,:,k) = min( Q(:,:,:,k) ,[], 3);
end

% Forward computation: optimal control + trajectory
XTraj(:,1) = [2;0.4];
for k = 1:K-1
    % find optimal control from Q-functions
    % V(x1,x2,k) = min_a Q(x1,x2,a,k): a is a minimizer of the Q functions
    % approximate Q(x1,x2, 1, k)
    tmp_Q_value = [];
    for aa = 1:length(A)
        Q_Func{aa} = griddedInterpolant(X1_grid, X2_grid, Q(:,:,aa,k));
        tmp_Q_value = [tmp_Q_value, Q_Func{aa}(XTraj(1,k),XTraj(2,k))];
    end
    [min_value, min_idx] = min(tmp_Q_value);

    OptCtrl(k) = A(min_idx);
    % find optimal trajectory
    XTraj(1, k+1) = next_x1( XTraj(1,k), XTraj(2,k), OptCtrl(k) );
    XTraj(2, k+1) = next_x2( XTraj(1,k), XTraj(2,k), OptCtrl(k) );
end
```

***Observations***

- Increase the grid size will get more closed results
- No matter how the grid size is changed, the optimal ctrl always chooses from -1,0,1, like a bang-bang control

### Example: double integrator in minimum-time problem

## DP in infinite-horizon discrete-time problem

### Bellman equation

## DP in continuous-time problem
placeholder

