# CH 7 Internal Sorting

- Bubble sort
    
    - Smaller value bubble up from bottom
- Selection sort
    
    - Some swaps in bubble are not necessary
    - Find the correct value for the ith position
- Quick sort
    
    - [快速排序算法_哔哩哔哩_bilibili](https://www.bilibili.com/video/BV1at411T75o/?spm_id_from=333.337.search-card.all.click&vd_source=7b7c808fef5805bab182d3bbcf195227)
    - Recursion based
    - The Quick Sort may not be better than Bubble sort in the worst case
        
        - e.g., 4 3 0 2 1 causes the maximum number of swaps
    - Optimization: Better pivot, Better algorithm for small sublists, eliminate recursion

# CH 9 Searching

- Definition: a systematic method for locating the record with key value K
- Directly tell you the location **by calculation**, or the process of **mapping a key value to a position in table**
- Hash function h(k) and hash table (HT)
- Collision: two keys hash to the same slot
    
    - Usually, collision can not be avoidable, but the number of collisions can be minized.
    - Solution 1: add a new seat at the same position
        
        - Linked List
            
            - Ads: deletion is easy
            - Disads: memory allocation in linked list manipulation will slow down the program
        - Bucket Hashing
            
            - Search is easy with litter records and expensive if many records in overflow.
    - Solution 2: Find another seat according to a mechanism
        
        - Probing
            
            - the goal of collision resolution is to **find a free slot in the table**
            - Probe Sequence: the series of slots visited during insert/search by following a collision resolution policy
            - Probe Function: p(i), i is the number of trials
                
                - Linear Probing: p(i)=i
                    
                    - Simply goes to the next slot
                    - Drawback: Primary Clustering
                - Quadratic Probing: p(i)=i^2
                    
                    - Primary Clustering is eliminated
                    - Drawback: Secondary clustering, same probe patterns
                - Double Probing
                    
                    - Avoid secondary clustering, two hash functions are applied
                    - h(k) determine the initial position, and h2(k) determine the interval of each step
                        
                        - p(k,i)=i\*h2(k)

# CH 10 Indexing

- Definition: a process of associating a key with the location of a corresponding data record
- Primary Index and Secondary Index
- Linear Indexing
- Tree Indexing
    
    - Binary Search Tree is not a suitable structure
    - 2-3 Tree
        
        - Definition
            
            - A node contains one or two keys
            - Every internal node has either
                
                - two children, if it contains one key at left
                - three children, if it contains two keys
            - All leaves are at the same level in the tree (**height balanced**)
            - Functions
                
                - Find: examine left key, right key, left child, center child, and right child
                - Insert

# CH 11 Graph

- Terminology: Vertice, Edge, Undirected Graph, Directed Graph, Adjacent, Path, Cycle and Simple cycle, Acyclic, Directed Acyclic Graph (DAG), Subgraph, Connected, Strongly connected, Weakly connected, Complete graph
- Representation
    
    - Adjacency Matrix
        
        - Space: V^2
    - Adjacency List
        
        - Space: V + E
- Graph Traversal (Alphabetically)
    
    - Depth first search
    - Breadth first search
- Shortest Paths Problem
    
    - Find out the shortest path between two specified vertices
    - Single-source Shortest-Paths Problem (SS SP Problem)
        
        - Dijkstra’s Algorithm
            
            - 每次从全局路径最小的节点出发更新到身边连接节点的距离
    - Floyd’s Algorithm
        
        - All pairs Shortest-path Problem
        - K-path
- Minimum Cost Spanning Tree (MST)
    
    - Definition: has the minimum total cost as measured by summing the values of all the edges in the subset, keeping vertices connected.