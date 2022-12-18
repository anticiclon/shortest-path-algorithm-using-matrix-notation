#include "functions.h"


// A utility function to print the constructed path array.
void printPath(int currentNode, int* parent, int src) {

    if (currentNode == src) {
        std::cout << src << ' ';
        return;
    }

    printPath(parent[currentNode], parent, src);
    std::cout << currentNode << ' ';
}

// A utility function to construct the path array.
void auxPath(int currentNode, int* parent, int src, std::vector<int>* path) {

    if (currentNode == src) {
        (*path).push_back(currentNode);
        return;
    }
    (*path).push_back(currentNode);
    auxPath(parent[currentNode], parent, src, path);

}

