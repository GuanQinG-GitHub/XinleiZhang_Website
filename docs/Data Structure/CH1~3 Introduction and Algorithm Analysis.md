# CH1~3 Introduction and Algorithm Analysis

## 1 Philosophy

- Improve the **efficiency** of a program
    
    - Shorter running time
    - Less memory
- Definition: any **data representation** and its associated **operations**
    
    - e.g., Integer & Summation, String & Replace
- Every data structure has costs and benefits; no data structure is better than another in all situations.
- A data structure requires:
    
    - **Space** for each data item it stores
    - **Time** to perform each basic operation
    - **Programming Effort**
- Terminology
    
    - Type: **a collection of values**
        
        - e.g., Boolean type and Integer type
    - Data Type: **a type** and **a collection of operations** that manipulate the type
        
        - e.g., Integer type and {+,-,\*,/} operations
    - Data Item: **a piece of information** of a record drawn from a data type
        
        - e.g., true or false from a Boolean data type
    - Atomic Data Type: contain no subparts
        
        - e.g., Integer and Boolean
    - Structure Data Type: contain several pieces of information
        
        - e.g., Array and String
    - Abstract Data Type: a definition for a data type **solely** in terms of a set of values and a set of operations on that data type.
    - Data Structure: the **physical implementation** of an ADT
        
        [Abstract Data Types - GeeksforGeeks](https://www.geeksforgeeks.org/abstract-data-types/)
        
    - Forms of data items
        
        - Logical Form: the data defined within ADT
            
            - e.g., Integers in mathematiacl sense
        - Physical Form: Implementation of the data item within a data structure
            
            - e.g., 16/32 bit integers: overflow
    - Problem & Algorithm & program

## Algorithm Analysis

- Two criteria
    
    - Efficiency
        
        - Empirical Comparison
            
            - Machine dependent
            - Difficult to setup a baseline
        - Algorithm Analysis
            
            - Estimate the time and space needed for a program
                
                - Growth Rate
                - Wrost Case
            - Machine Independent
    - Easy to understand
- Three different measures:
    
    - Big-Oh
        
        - the **upper bound** of a growth rate; the **tightest** upper bound is preferred
        - e.g., for sequential search, T(O) is in O(n) and the worst case is n.
    - Big-Omega
        
        - the **lower bound** of a growth rate
    - Big-Theta
        
        - when Big-Oh and Big-Omega are the same, this situation is indicated using Big-Theta.
- Space/Time Tradeoff Principle
    
    - One can often reduce time if one is willing to sacrifice space, or vice versa.
        
        - e.g., Encoding or packing information (save space)
        - e.g., Table lookup (save time)