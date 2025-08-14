# MATLAB GPU Programming Tutorial for Monte Carlo Simulations

## Table of Contents
1. [Basic Setup](#1-basic-setup)
2. [Key Functions](#2-key-functions)
3. [Example Implementation](#3-example-implementation)
4. [Best Practices](#4-best-practices)
5. [Common Pitfalls](#5-common-pitfalls)

## 1. Basic Setup

```matlab
% Initialize random number generators
seed = 1234;
rng(seed);         % CPU random number generator
gpurng(seed);      % GPU random number generator

% Check GPU availability
hasGPU = gpuDevice();  % Returns GPU device object if available
```

## 2. Key Functions

### Creating GPU Arrays
```matlab
% Convert CPU array to GPU array
gpu_array = gpuArray(cpu_array);

% Create arrays directly on GPU
zeros_gpu = zeros(100, 100, 'gpuArray');
rand_gpu = randn(100, 100, 'gpuArray');
```

### Data Transfer
```matlab
% GPU to CPU transfer
cpu_data = gather(gpu_data);

% CPU to GPU transfer
gpu_data = gpuArray(cpu_data);
```

### Parallel Execution
```matlab
% Apply function to each element of GPU array
result = arrayfun(@my_function, gpu_array);
```

## 3. Example Implementation

```matlab
% GPU_vs_CPU_Example.m
function GPU_vs_CPU_Example()
    % Parameters
    n_paths = 10000;    % Number of Monte Carlo paths
    n_steps = 1000;     % Time steps per path
    
    % Test both implementations
    disp('Testing CPU implementation...');
    tic;
    [cpu_results, cpu_time] = runCPU(n_paths, n_steps);
    cpu_elapsed = toc;
    
    disp('Testing GPU implementation...');
    tic;
    [gpu_results, gpu_time] = runGPU(n_paths, n_steps);
    gpu_elapsed = toc;
    
    % Compare results
    fprintf('\nResults Summary:\n');
    fprintf('CPU total time: %.4f seconds\n', cpu_elapsed);
    fprintf('GPU total time: %.4f seconds\n', gpu_elapsed);
    fprintf('Speedup factor: %.2fx\n', cpu_elapsed/gpu_elapsed);
    
    % Verify results are similar
    max_diff = max(abs(cpu_results - gather(gpu_results)));
    fprintf('Maximum difference between CPU and GPU results: %.2e\n', max_diff);
end

function [results, computation_time] = runCPU(n_paths, n_steps)
    % CPU implementation
    tic;
    results = zeros(1, n_paths);
    
    for i = 1:n_paths
        % Initialize path
        x = 0;
        
        % Simulate path
        for j = 1:n_steps
            eps = randn();
            x = x + 0.1*x*0.01 + 0.2*eps*sqrt(0.01);
        end
        
        % Store result
        results(i) = x;
    end
    computation_time = toc;
end

function [results, computation_time] = runGPU(n_paths, n_steps)
    % GPU implementation
    tic;
    
    % Initialize all paths on GPU
    paths = zeros(1, n_paths, 'gpuArray');
    
    % Simulate all paths in parallel
    for j = 1:n_steps
        eps = randn(1, n_paths, 'gpuArray');
        paths = paths + 0.1*paths*0.01 + 0.2*eps*sqrt(0.01);
    end
    
    results = paths;
    computation_time = toc;
end
```

## 4. Best Practices

### Efficient Data Transfer
```matlab
% Good Practice
gpu_data = gpuArray(cpu_data);
for i = 1:n_steps
    % Process on GPU
end
final_result = gather(gpu_result);

% Bad Practice
for i = 1:n_steps
    gpu_data = gpuArray(cpu_data);  % Avoid frequent transfers
    % Process
    cpu_result = gather(gpu_result);
end
```

### Memory Management
```matlab
% Clear GPU memory
reset(gpuDevice);

% Check GPU memory usage
gpu = gpuDevice();
fprintf('Available GPU memory: %g GB\n', gpu.AvailableMemory/1e9);
```

### When to Use GPU
- Large-scale parallel computations
- Computationally intensive operations
- Minimal data dependencies
- Large datasets

### When to Avoid GPU
- Small datasets
- Sequential operations
- Operations requiring frequent CPU-GPU transfers
- Memory-constrained situations

## 5. Common Pitfalls

1. **Excessive Data Transfer**
   - Minimize transfers between CPU and GPU
   - Keep computations on GPU as long as possible

2. **Poor Memory Management**
   - Monitor GPU memory usage
   - Clear GPU memory when needed
   - Consider memory limitations

3. **Inefficient Parallelization**
   - Use vectorized operations instead of loops
   - Utilize built-in GPU-enabled functions
   - Ensure proper batch sizing

4. **Error Handling**
```matlab
% Always check for GPU availability
if gpuDeviceCount > 0
    % GPU code
else
    % CPU fallback
end
```

This tutorial is based on the implementation shown in the provided Monte Carlo simulation code. For specific applications, you may need to adjust the approaches based on your computational requirements and hardware capabilities.