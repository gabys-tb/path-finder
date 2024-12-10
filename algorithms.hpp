#ifndef ALG_H
#define ALG_H

#include <queue>
#include <unordered_map>
#include <vector>
#include <iostream>
#include <stack>
#include <limits>
#include <fstream>
#include <utility>
#include <string>
#include <algorithm>
#include <chrono>    // Para medir o tempo de execução


using namespace std::chrono;
using namespace std;

// Estrutura para representar um nó
struct Node {
    float weight;              // Peso do nó
    int row, col;            // Coordenadas originais no mapa
    vector<int> neighbors;   // Lista de vizinhos conectados
};

void printPath(pair<vector<int>, float> result, int cols);
std::pair<int, int> toCoordinates(int index, int cols);
void isValidPath(const unordered_map<int, Node>& graph, const vector<int>& path);
pair<vector<int>, float> bfs(const unordered_map<int, Node> &graph, int start, int end, int &expandedStates, double &executionTime);
pair<vector<int>, float> ids(const unordered_map<int, Node> &graph, int start, int goal, int &expandedStates, double &executionTime);
pair<vector<int>, float> ucs(const unordered_map<int, Node> &graph, int start, int goal, int &expandedStates, double &executionTime);
pair<vector<int>, float> greedySearch(const unordered_map<int, Node> &graph, int start, int goal, int &expandedStates, double &executionTime);
pair<vector<int>, float> aStar(const unordered_map<int, Node> &graph, int start, int goal, int &expandedStates, double &executionTime);

#endif