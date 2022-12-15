#include <iostream>
#include <vector>

// A utility function to print the constructed path array.
void printPath(int currentNode, int* parent, int src) {

    if (currentNode == src) {
        std::cout << src << ' ';
        return;
    }

    printPath(parent[currentNode], parent, src);
    std::cout << currentNode << ' ';
}

// A utility function to print the constructed distance array.
void printSolution(int* dist, int* parent, int V, int src) {

    std::cout << "Node \t\t Distance from Source \t\tPath" << '\n';

    for (int i{ 0 }; i < V; i++) {
        std::cout << i << "\t\t\t" << dist[i] << "\t\t\t";
        printPath(i, parent, src);
        std::cout << '\n';
    }
    
}

// A utility function to find the vertex with minimun
// distance value, from the set of vertices not yet included
// in shortest path tree
int minDistance(int* dist, bool* sptSet, int V) {

    int min{ INT_MAX };
    int min_index{ 0 };

    for (int v{ 0 }; v < V; v++) {
        if (sptSet[v] == false && dist[v] <= min) {
            min = dist[v];
            min_index = v;
        }
    }

    return min_index;
}



// Function that implements Dijkstra's single source
// shortest path algorithm for a graph represented using
// adjacency matrix representation
// ----
// V is the number of vertices in the graph
// ----
void dijkstra(int** graph, int src, int V, int* dist, bool* sptSet, int* parent) {
    // -- The output array dist[i] will hold the shortest
    //    distance from src to i
    // -- sptSet[i] will be true if vertex i is included in shortest path
    //    tree or shortest distance from src to i is finalize
    for (int i{ 0 }; i < V; i++) {
        dist[i] = INT_MAX;
        sptSet[i] = false;
        parent[i] = i;
    }

    // Distance of source vertex from itself is always 0
    dist[src] = 0;

    // Find the shortest path of all vertices
    for (int count{ 0 }; count < V - 1; count++) {

        // Pick the minimum distance vertex from the set of vertices not
        // yet processed. u is always iqual to src in the first iteration.
        int u = minDistance(dist, sptSet, V);

        // Mark the picked vertex as processed
        sptSet[u] = true;

        // Update dist value of the adjacent vertices of the picked vertex.
        for (int v{ 0 }; v < V; v++) {

            // Update dist[v] only if is not in the set of vertices included,
            // there is an edge from u to v, and total 
            // weight of path from src to v through u 
            // is smaller than current value of dist[v]
            if (!sptSet[v]
                && graph[u][v]
                && dist[u] != INT_MAX
                && dist[u] + graph[u][v] < dist[v]) {
                dist[v] = dist[u] + graph[u][v];
                parent[v] = u;
            }

        }
    }
    // print the constructed distance array
    printSolution(dist, parent, V, src);
}


// Function that return the shortest distance between two nodes
int dijkstraDistance(int** graph, int src, int tgt, int V, int* dist, bool* sptSet, int* parent) {

    // -- The output array dist[i] will hold the shortest
    //    distance from src to i
    // -- sptSet[i] will be true if vertex i is included in shortest path
    //    tree or shortest distance from src to i is finalize 
    // -- parent[i] will stores parent vertex of v in shortest path tree.
    for (int i{ 0 }; i < V; i++) {
        dist[i] = INT_MAX;
        sptSet[i] = false;
        parent[i] = i;
    }

    // Distance of source vertex from itself is always 0
    dist[src] = 0;

    // Find the shortest path of all vertices
    for (int count{ 0 }; count < V - 1; count++) {

        // Pick the minimum distance vertex from the set of vertices not
        // yet processed. u is always iqual to src in the first iteration.
        int u = minDistance(dist, sptSet, V);

        // Mark the picked vertex as processed
        sptSet[u] = true;

        // Update dist value of the adjacent vertices of the picked vertex.
        for (int v{ 0 }; v < V; v++) {

            if (!sptSet[v] // Update dist[v] only if is not in the set of vertices included,
                && graph[u][v] // there is an edge from u to v, and total 
                && dist[u] != INT_MAX // weight of path from src to v through u
                && dist[u] + graph[u][v] < dist[v]) { // is smaller than current value of dist[v]
                dist[v] = dist[u] + graph[u][v];
                parent[v] = u;
            }

            if (u == tgt) {
                break;
            }

        }

        if (u == tgt) {
            break;
        }

    }
    return dist[tgt];
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

// Function that implements Dijkstra's single source
// shortest path algorithm for a graph represented using
// adjacency matrix representation
std::vector<int> dijkstraPath(int** graph, int src, int tgt, int V, int* dist, bool* sptSet, int* parent) {

    // -- The output array dist[i] will hold the shortest
    //    distance from src to i
    // -- sptSet[i] will be true if vertex i is included in shortest path
    //    tree or shortest distance from src to i is finalize 
    // -- parent[i] will stores parent vertex of v in shortest path tree.
    for (int i{ 0 }; i < V; i++) {
        dist[i] = INT_MAX;
        sptSet[i] = false;
        parent[i] = i;
    }

    // Distance of source vertex from itself is always 0
    dist[src] = 0;

    // Find the shortest path of all vertices
    for (int count{ 0 }; count < V - 1; count++) {

        // Pick the minimum distance vertex from the set of vertices not
        // yet processed. u is always iqual to src in the first iteration.
        int u = minDistance(dist, sptSet, V);

        // Mark the picked vertex as processed
        sptSet[u] = true;

        // Update dist value of the adjacent vertices of the picked vertex.
        for (int v{ 0 }; v < V; v++) {

            if (!sptSet[v] // Update dist[v] only if is not in the set of vertices included,
                && graph[u][v] // there is an edge from u to v, and total 
                && dist[u] != INT_MAX // weight of path from src to v through u
                && dist[u] + graph[u][v] < dist[v]) { // is smaller than current value of dist[v]
                dist[v] = dist[u] + graph[u][v];
                parent[v] = u;
            }


            if (u == tgt) {
                break;
            }

        }

        if (u == tgt) {
            break;
        }

    }
    std::vector<int> path{};
    std::vector<int>* ptr_path{&path};

    auxPath(tgt, parent, src, ptr_path);

    std::reverse(path.begin(), path.end());

    return path;

}