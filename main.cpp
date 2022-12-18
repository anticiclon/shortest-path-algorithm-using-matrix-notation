#include "functions.h"

int main() {
    const int V{ 9 };
    //Let us create the example graph discussed above 

    int graph[V][V]{ { 0, 4, 0, 0, 0, 0, 0, 8, 0 },
                      { 4, 0, 8, 0, 0, 0, 0, 11, 0 },
                      { 0, 8, 0, 7, 0, 4, 0, 0, 2 },
                      { 0, 0, 7, 0, 9, 14, 0, 0, 0 },
                      { 0, 0, 0, 9, 0, 10, 0, 0, 0 },
                      { 0, 0, 4, 14, 10, 0, 2, 0, 0 },
                      { 0, 0, 0, 0, 0, 2, 0, 1, 6 },
                      { 8, 11, 0, 0, 0, 0, 1, 0, 7 },
                      { 0, 0, 2, 0, 0, 0, 6, 7, 0 } };

    int** ptr_graph{ new int* [V] }; // allocate an array of V int pointers — these are our rows
    for (int count{ 0 }; count < V; ++count)
        ptr_graph[count] = &graph[count][0]; // these are our columns

    int distancia[V]{};
    int* ptr_distancia{ distancia };
    bool conjunto[V]{};
    bool* ptr_conjunto{ conjunto };
    int padres[V]{};
    int* ptr_padres{ padres };
    int nodo_partida{ 3 };
    int nodo_final{ 7 };


    dijkstra<int>(ptr_graph, nodo_partida, V, ptr_distancia, ptr_conjunto, ptr_padres);

    std::cout << '\n';
    int distance{ dijkstraDistance<int>(ptr_graph, nodo_partida, nodo_final, V, ptr_distancia, ptr_conjunto, ptr_padres) };
    std::cout << "La distancia minima entre el nodo " << nodo_partida << " y el nodo " << nodo_final << " es " << distance << '\n';


    std::vector<int> camino = dijkstraPath<int>(ptr_graph, nodo_partida, nodo_final, V, ptr_distancia, ptr_conjunto, ptr_padres);

    std::cout << '\n';

    std::cout << "El camino mas corto entre el nodo " << nodo_partida << " y el nodo " << nodo_final << " es ";

    for (int i : camino) {
        std::cout << i << ' ';
    }
    std::cout << '\n';

    return 0;
}
