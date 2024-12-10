#include "algorithms.hpp"
#include <iomanip>

// Converte um índice 1D para coordenadas 2D (linha, coluna)
std::pair<int, int> toCoordinates(int index, int cols) {
    int row = index / cols;  // Linha é o quociente da divisão
    int col = index % cols;  // Coluna é o resto da divisão
    return {row, col};       // Retorna as coordenadas como um par
}

// Função para executar BFS e encontrar o menor caminho com custo total
pair<vector<int>, float> bfs(
    const unordered_map<int, Node> &graph,
    int start,
    int end,
    int &expandedStates,     // Número de estados expandidos
    double &executionTime    // Tempo de execução em milissegundos
) {
    // Início da medição do tempo
    auto startTime = high_resolution_clock::now();

    unordered_map<int, int> parent;    // Para rastrear o caminho
    unordered_map<int, bool> visited; // Para marcar nós visitados
    unordered_map<int, float> cost;   // Para rastrear o custo acumulado
    queue<int> q;

    expandedStates = 0; // Inicializar contador de estados expandidos

    // Início do BFS
    q.push(start);
    visited[start] = true;
    parent[start] = -1;
    cost[start] = 0.0f;

    while (!q.empty()) {
        int current = q.front();
        q.pop();
        expandedStates++; // Incrementar estados expandidos

        // Se encontramos o destino, reconstruir o caminho
        if (current == end) {
            vector<int> path;
            float totalCost = cost[current];
            for (int at = end; at != -1; at = parent[at]) {
                path.push_back(at);
            }
            reverse(path.begin(), path.end());

            // Fim da medição do tempo
            auto endTime = high_resolution_clock::now();
            executionTime = duration<double, milli>(endTime - startTime).count();

            return {path, totalCost};
        }

        // Explorar os vizinhos
        for (int neighbor : graph.at(current).neighbors) {
            if (!visited[neighbor]) {
                visited[neighbor] = true;
                parent[neighbor] = current;
                cost[neighbor] = cost[current] + graph.at(neighbor).weight; // Acumula o custo do nó
                q.push(neighbor);
            }
        }
    }

    // Fim da medição do tempo, se não encontrar o objetivo
    auto endTime = high_resolution_clock::now();
    executionTime = duration<double, milli>(endTime - startTime).count();

    // Se não houver caminho, retornar vazio e custo -1
    return {{}, -1};
}


// Função para exibir o caminho em termos de coordenadas
void printPath(pair<vector<int>, float> result, int cols) {
    if (result.first.empty()) {
        cout << "Nenhum caminho encontrado." << endl;
        return;
    }

    cout << fixed << setprecision(1) << result.second << " ";
    
    for (int index : result.first) {
        auto [row, col] = toCoordinates(index, cols);
        cout << "(" << col << "," << row << ") ";
    }
    cout << endl;
    
}

// Função para verificar se o caminho retornado é válido
void isValidPath(const unordered_map<int, Node>& graph, const vector<int>& path) {
    if (path.empty()) {
        cout << "Caminho está vazio." << endl;
        return;
    }

    for (size_t i = 0; i < path.size() - 1; ++i) {
        int current = path[i];
        int next = path[i + 1];

        // Verificar se o nó atual existe no grafo
        if (graph.find(current) == graph.end()) {
            cout << "Nó " << current << " não existe no grafo." << endl;
            return;
        }

        // Verificar se o próximo nó está na lista de vizinhos do nó atual
        const vector<int>& neighbors = graph.at(current).neighbors;
        if (find(neighbors.begin(), neighbors.end(), next) == neighbors.end()) {
            cout << "Nó " << next << " não é vizinho de " << current << "." << endl;
            return;
        }
    }

    cout << "O caminho é válido." << endl;
    return;
}

bool depthLimitedSearch(
    const unordered_map<int, Node> &graph,
    int current,
    int goal,
    int depth,
    vector<int> &path,
    vector<bool> &visited,
    float &cost,
    int &expandedStates
) {
    // Se a profundidade máxima for atingida, retorna falso
    if (depth < 0) return false;

    visited[current] = true; // Marca como visitado
    path.push_back(current); // Adiciona ao caminho
    expandedStates++;        // Incrementa os estados expandidos

    // Verifica se atingiu o objetivo
    if (current == goal) {
        return true;
    }

    // Explora os vizinhos
    for (const auto &neighbor : graph.at(current).neighbors) {
        if (!visited[neighbor]) {
            float edgeCost = graph.at(current).weight; // Custo da aresta
            cost += edgeCost;

            if (depthLimitedSearch(graph, neighbor, goal, depth - 1, path, visited, cost, expandedStates)) {
                return true;
            }

            cost -= edgeCost; // Reverte o custo acumulado
        }
    }

    // Se não encontrar, remove o nó do caminho e marca como não visitado
    path.pop_back();
    visited[current] = false;

    return false;
}

pair<vector<int>, float> ids(
    const unordered_map<int, Node> &graph,
    int start,
    int goal,
    int &expandedStates,
    double &executionTime
) {
    // Início da medição do tempo
    auto startTime = high_resolution_clock::now();

    int maxDepth = numeric_limits<int>::max();
    expandedStates = 0;

    vector<int> path;                    // Pré-aloca o vetor para o caminho
    vector<bool> visited(graph.size());  // Usa vector<bool> para maior eficiência em memória

    for (int depth = 0; depth < maxDepth; ++depth) {
        path.clear();                    // Limpa o vetor antes de cada chamada
        fill(visited.begin(), visited.end(), false); // Reseta o vetor visited

        // Chamada da busca em profundidade limitada
        float cost = 0.0f;
        if (depthLimitedSearch(graph, start, goal, maxDepth, path, visited, cost, expandedStates)) {
            // Fim da medição do tempo
            auto endTime = high_resolution_clock::now();
            executionTime = duration<double, milli>(endTime - startTime).count();

            return {path, cost};
        }
    }
    
    // Fim da medição do tempo, se não encontrar o objetivo
    auto endTime = high_resolution_clock::now();
    executionTime = duration<double, milli>(endTime - startTime).count();

    return {{}, -1}; // Nenhum caminho encontrado
}

// Definição para a fila de prioridade (custo, nó, caminho)
using QueueNode = pair<float, pair<int, vector<int>>>;

// Função para executar UCS e encontrar o menor caminho com custo total
pair<vector<int>, float> ucs(
    const unordered_map<int, Node> &graph,
    int start,
    int goal,
    int &expandedStates,  // Número de estados expandidos
    double &executionTime // Tempo de execução em milissegundos
) {
    // Início da medição do tempo
    auto startTime = high_resolution_clock::now();

    priority_queue<QueueNode, vector<QueueNode>, greater<QueueNode>> pq; // Fila de prioridade (menor custo primeiro)
    unordered_map<int, bool> visited; // Controle de nós visitados

    // Inicializar contadores
    expandedStates = 0;

    // Inserir o nó inicial na fila: custo, nó atual, caminho
    pq.push({0.0, {start, {start}}});

    while (!pq.empty()) {
        // Extrair o elemento com menor custo
        auto [currentCost, currentData] = pq.top();
        pq.pop();

        int currentNode = currentData.first;
        vector<int> currentPath = currentData.second;

        // Incrementar o número de estados expandidos
        expandedStates++;

        // Se o nó já foi visitado, ignorar
        if (visited[currentNode]) continue;
        visited[currentNode] = true;

        // Verificar se chegamos ao objetivo
        if (currentNode == goal) {
            // Fim da medição do tempo
            auto endTime = high_resolution_clock::now();
            executionTime = duration<double, milli>(endTime - startTime).count();

            return {currentPath, currentCost}; // Retorna o caminho e o custo total
        }

        // Explorar os vizinhos do nó atual
        for (int neighbor : graph.at(currentNode).neighbors) {
            if (!visited[neighbor]) {
                float neighborCost = currentCost + graph.at(neighbor).weight;
                vector<int> newPath = currentPath;
                newPath.push_back(neighbor);
                pq.push({neighborCost, {neighbor, newPath}});
            }
        }
    }

    // Se não houver caminho para o objetivo, registrar o tempo e retornar
    auto endTime = high_resolution_clock::now();
    executionTime = duration<double, milli>(endTime - startTime).count();

    return {{}, -1.0};
}


// Função heurística: Distância de Manhattan
float manhattanDistance(int x1, int y1, int x2, int y2) {
    return abs(x1 - x2) + abs(y1 - y2);
}

// Função para executar o Algoritmo de Busca Gulosa
pair<vector<int>, float> greedySearch(
    const unordered_map<int, Node>& graph,
    int start,
    int goal,
    int& expandedStates,     // Número de estados expandidos
    double& executionTime    // Tempo de execução em milissegundos
) {
    // Início da medição do tempo
    auto startTime = high_resolution_clock::now();

    priority_queue<pair<float, int>, vector<pair<float, int>>, greater<>> pq; // Fila de prioridade (menor heurística primeiro)
    unordered_map<int, bool> visited; // Controle de nós visitados
    unordered_map<int, int> parent;   // Para reconstruir o caminho
    unordered_map<int, float> cost;  // Para armazenar o custo acumulado
    expandedStates = 0;               // Inicializar estados expandidos

    // Inserir o nó inicial na fila de prioridade
    pq.push({manhattanDistance(graph.at(start).row, graph.at(start).col, graph.at(goal).row, graph.at(goal).col), start});
    parent[start] = -1;
    cost[start] = 0.0f; // Custo inicial é 0

    while (!pq.empty()) {
        // Extrair o nó com menor heurística
        auto [heuristic, current] = pq.top();
        pq.pop();

        expandedStates++; // Incrementar estados expandidos

        // Se o nó já foi visitado, ignorar
        if (visited[current]) continue;
        visited[current] = true;

        // Verificar se chegamos ao objetivo
        if (current == goal) {
            vector<int> path;
            float totalCost = cost[current];
            for (int at = goal; at != -1; at = parent[at]) {
                path.push_back(at);
            }
            reverse(path.begin(), path.end());

            // Fim da medição do tempo
            auto endTime = high_resolution_clock::now();
            executionTime = duration<double, milli>(endTime - startTime).count();

            return {path, totalCost};
        }

        // Explorar os vizinhos do nó atual
        for (int neighbor : graph.at(current).neighbors) {
            if (!visited[neighbor]) {
                parent[neighbor] = current;
                cost[neighbor] = cost[current] + graph.at(neighbor).weight; // Acumular custo
                float heuristic = manhattanDistance(
                    graph.at(neighbor).row, graph.at(neighbor).col,
                    graph.at(goal).row, graph.at(goal).col
                );
                pq.push({heuristic, neighbor});
            }
        }
    }

    // Fim da medição do tempo, se não encontrar o objetivo
    auto endTime = high_resolution_clock::now();
    executionTime = duration<double, milli>(endTime - startTime).count();

    // Se não houver caminho, retornar vazio e custo -1
    return {{}, -1};
}

// Função para executar o Algoritmo A* (A-Estrela)
pair<vector<int>, float> aStar(
    const unordered_map<int, Node>& graph,
    int start,
    int goal,
    int& expandedStates,     // Número de estados expandidos
    double& executionTime    // Tempo de execução em milissegundos
) {
    // Início da medição do tempo
    auto startTime = high_resolution_clock::now();

    // Fila de prioridade para A* (f(n), nó)
    priority_queue<pair<float, int>, vector<pair<float, int>>, greater<>> pq;
    unordered_map<int, float> gCost;  // Custo acumulado para chegar a cada nó
    unordered_map<int, float> fCost;  // f(n) = g(n) + h(n)
    unordered_map<int, bool> visited; // Controle de nós visitados
    unordered_map<int, int> parent;   // Para reconstruir o caminho
    expandedStates = 0;               // Inicializar estados expandidos

    // Inicializar custos do nó inicial
    gCost[start] = 0.0f;
    fCost[start] = manhattanDistance(graph.at(start).row, graph.at(start).col, graph.at(goal).row, graph.at(goal).col);
    pq.push({fCost[start], start});
    parent[start] = -1;

    while (!pq.empty()) {
        // Extrair o nó com menor f(n)
        auto [currentFCost, current] = pq.top();
        pq.pop();

        expandedStates++; // Incrementar estados expandidos

        // Se o nó já foi visitado, ignorar
        if (visited[current]) continue;
        visited[current] = true;

        // Verificar se chegamos ao objetivo
        if (current == goal) {
            vector<int> path;
            float totalCost = gCost[current];
            for (int at = goal; at != -1; at = parent[at]) {
                path.push_back(at);
            }
            reverse(path.begin(), path.end());

            // Fim da medição do tempo
            auto endTime = high_resolution_clock::now();
            executionTime = duration<double, milli>(endTime - startTime).count();

            return {path, totalCost};
        }

        // Explorar os vizinhos do nó atual
        for (int neighbor : graph.at(current).neighbors) {
            float tentativeGCost = gCost[current] + graph.at(neighbor).weight;

            // Se o custo acumulado for menor ou o vizinho não foi visitado
            if (!gCost.count(neighbor) || tentativeGCost < gCost[neighbor]) {
                parent[neighbor] = current;
                gCost[neighbor] = tentativeGCost;
                fCost[neighbor] = gCost[neighbor] + manhattanDistance(
                    graph.at(neighbor).row, graph.at(neighbor).col,
                    graph.at(goal).row, graph.at(goal).col
                );

                pq.push({fCost[neighbor], neighbor});
            }
        }
    }

    // Fim da medição do tempo, se não encontrar o objetivo
    auto endTime = high_resolution_clock::now();
    executionTime = duration<double, milli>(endTime - startTime).count();

    // Se não houver caminho, retornar vazio e custo -1
    return {{}, -1};
}
