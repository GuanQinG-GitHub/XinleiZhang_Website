# CH5 Binary Tree

## 5.1 Recursion

- Definition: A procedure which calls itself
- Two elements:
    
    - Recursive Call
    - Base Case
- Example: Factorial Function

## 5.2 Binary Tree

- Terminology
    
    - Root, Ancestor, Parent, Sibling, Children, Descendant
    - Internal Node, Edge, Leaf Node, Sub-tree
    - Depth, Level, Height
- Types:
    
    - **Full Binary Tree**: each node being either Leaf or Internal Node with exactly two non-empty Children;
    - **Complete Binary Tree**: if the height of the tree is **d** (level is d-1), then all levels except possibly level **d-1** are completely full and the bottom level has all nodes filled in from the **left** side.  
- Theorem 1: The number of **leaves** in a non-empty **full binary tree** is **one** more than the number of **internal nodes**. (root is also counted as an internal node)
- Theorem 2: The number of **null pointers** in a non-empty binary tree is one more than the number of **nodes** in the tree. (Derived from theorem 1)
    
    - number of null pointers: 2 \* number of leaves
    - number of nodes: number of leaves + number of internal nodes
    - number of leaves = number of internal nodes + 1 (theorem 1)
- Binary Node: Node ADT
- Traversals
    
    - Definition: a process for visiting the nodes in some order.
    - Enumeration: any traversal that lists every node in the tree exactly once
    - Types:
        
        - Preorder traversal
            
            - Definition: visit each node before visiting its children
            - Example:
        - Postorder traversal
            
            - Definitoin: visit each node after visting its children
            - Example:
        - Inorder traversal
            
            - Definition: visit the left subtree, then the node, then the right subtree
            - Example: C B A F E G D I J H
- Different Leafs & Internal Nodes
    
    - Type 1: **Each node** stores the same kind of data
    - Type 2: **Internal nodes and leaves** store different types of data
    - Three Implementations
        
        - Union Implementation
        - Inheritance Implementation
        - Inheritance with virtual function
- Space Requirements
    
    - Terminology
        
        - Overhead Space: amount of space **necessary** to **maintain the data structure** (in other words, it is any space not used to store data records, or simply the space used to store pointers)
        - Overhead Fraction: amount of overhead space divided by amount of total space used
        - Example: suppose a full tree with all nodes are of the same data structure, two pointers and one element, while number of nodes is n, the space required by a pointer is p and by an element is d. Overhead Space = 2pn and Over Fraction is 2pn/(2pn + dn)

## 5.3 Binary Search Tree

## 5.4 Array-based Complete Binary Tree

## 5.5 Heaps

## 5.6 Huffman Coding Tree