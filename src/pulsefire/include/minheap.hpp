// MODIFIED FROM https://www.geeksforgeeks.org/binary-heap/
#pragma once

#include <vector>

struct Node {
    float path_cost = 0;
    float goal_cost = 0;
    float cost = 0;
    Node* parent = nullptr;

    size_t x;
    size_t y;
    size_t i;

    bool is_open = true;
    bool is_unknown = false;
    bool is_inserted = false;
    bool is_visited = false;
};
  
// A class for Min Heap
class MinHeap
{ 
    std::vector<Node*> harr;
public:
  
    // to heapify a subtree with the root at given index 
    void MinHeapify(int i)
    { 
        int l = left(i); 
        int r = right(i); 
        int smallest = i; 
        if (l < harr.size() && harr[l]->cost < harr[i]->cost) 
            smallest = l; 
        if (r < harr.size() && harr[r]->cost < harr[smallest]->cost) 
            smallest = r; 
        if (smallest == i)
            return;
        swap(i, smallest); 
        return MinHeapify(smallest);
    }
  
    int parent(int i) { return (i-1)/2; } 
  
    // to get index of left child of node at index i 
    int left(int i) { return (2*i + 1); } 
  
    // to get index of right child of node at index i 
    int right(int i) { return (2*i + 2); } 
  
    // to extract the root which is the minimum element 
    Node* pop()
    {
        if (harr.size() == 0) 
            return NULL;
        Node* root = harr[0]; 
        if (harr.size() > 1)
            harr.front() = harr.back();
        harr.pop_back();
        MinHeapify(0);
        return root;
    }
  
    // Decreases key value of key at index i to new_val 
    void decreaseKey(int i)
    {
        while (i != 0 && harr[parent(i)]->cost > harr[i]->cost) 
        { 
           swap(i, parent(i)); 
           i = parent(i); 
        } 
    }
  
    // Inserts a new key 'k' 
    void push(Node* k)
    {
        // First insert the new key at the end
        k->i = harr.size();
        harr.push_back(k);
        decreaseKey(k->i);
    }

    void swap(int i, int j)
    {
        Node* tmp = harr[i];
        harr[i] = harr[j];
        harr[j] = tmp;
        harr[i]->i = i;
        harr[j]->i = j;
    }

    size_t size()
    {
        return harr.size();
    }
};