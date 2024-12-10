#include "algorithms.hpp"
#include <utility> 
#include <functional>


// Conversão de coordenadas 2D para um índice 1D único
int toIndex(int row, int col, int cols) {
    return row * cols + col;
}

// Função para verificar se a célula é válida para movimento
bool isValidCell(char cell) {
    return cell == '.' || cell == ';' || cell == 'x' || cell == '+' || cell == '@';
}

// Função para verificar se a célula é válida para movimento
bool isArroba(char cell) {
    return cell == '@';
}

// Função para atribuir peso a cada tipo de caractere
float getNodeWeight(char cell) {
    switch (cell) {
        case '.': return 1;  // Grama baixa
        case ';': return 1.5;  // Grama alta
        case 'x': return 6; // Fogo
        case '+': return 2.5;  // Água
        default: return numeric_limits<float>::max();  // Qualquer outro caractere é intransponível
    }
}



// Função para construir o grafo a partir do mapa
unordered_map<int, Node> mapToGraph(const vector<string> &map, int rows, int cols) {
    unordered_map<int, Node> graph;

    // Percorrer o mapa e criar nós
    for (int row = 0; row < rows; ++row) {
        for (int col = 0; col < cols; ++col) {
            char cell = map[row][col];
            int current = toIndex(row, col, cols);

            // Ignorar nós intransponíveis
            if (isArroba(cell)) continue;

            // Criar nó com peso e coordenadas
            Node node;
            node.weight = getNodeWeight(cell);
            node.row = row;
            node.col = col;

            // Verificar as células vizinhas e adicionar conexões
            vector<pair<int, int>> directions = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
            for (auto [dr, dc] : directions) {
                int nr = row + dr, nc = col + dc;
                if (nr >= 0 && nr < rows && nc >= 0 && nc < cols) {
                    int neighbor = toIndex(nr, nc, cols);
                    if (!isArroba(map[nr][nc])) {
                        node.neighbors.push_back(neighbor);
                    }
                }
            }

            // Adicionar nó ao grafo
            graph[current] = node;
        }
    }

    return graph;
}

// Função para remover tabulações de uma string
string removeTabs(const string &line) {
    string result;
    for (char c : line) {
        if (isValidCell(c)) {
            result += c;
        }
    }
    return result;
}

// Função para ler o mapa de um arquivo
bool readMapFromFile(const string &filename, vector<string> &map, int &rows, int &cols) {
    ifstream file(filename);

    if (!file.is_open()) {
        cerr << "Erro ao abrir o arquivo: " << filename << endl;
        return false;
    }

    // Ler a primeira linha com as dimensões
    file >> cols >> rows;
    file.ignore(); // Ignorar o caractere de nova linha após as dimensões

    // Ler o restante do mapa
    string line;
    while (getline(file, line)) {
        if (!line.empty()) {
            map.push_back(removeTabs(line)); // Remove tabulações antes de adicionar
        }
    }

    file.close();

    return true;
}

// Função para imprimir o grafo (para depuração)
void printGraph(const unordered_map<int, Node> &graph) {
    for (const auto &[index, node] : graph) {
        cout << "Nó " << index << " (linha: " << node.row << ", coluna: " << node.col
             << ", peso: " << node.weight << "): ";
        for (int neighbor : node.neighbors) {
            cout << neighbor << " ";
        }
        cout << endl;
    }
}

// Função que mapeia o método de busca para a função correspondente
bool executeSearchMethod(const string &method, 
const unordered_map<int, 
Node> &graph, 
int xi, int yi, int xf, int yf,
int cols) {
    
    // Mapeamento de identificadores para funções
    unordered_map<string, function<pair<vector<int>, float>(const unordered_map<int, Node> &graph, int, int, int&, double&)>> searchMethods = {
        {"BFS", bfs},
        {"IDS", ids},
        {"UCS", ucs},
        {"Greedy", greedySearch},
        {"Astar", aStar}
    };

    // Verificar se o método é válido
    if (searchMethods.find(method) == searchMethods.end()) {
        cerr << "Erro: Método de busca \"" << method << "\" não reconhecido." << endl;
        return false;
    }

    int start = toIndex(yi, xi, cols);
    int end = toIndex(yf, xf, cols);
    int states = 0;
    double time = 0;
    // Chamar a função correspondente
    pair<vector<int>, float> result = searchMethods[method](graph, start, end, states, time);

    printPath(result, cols);

    return true;
}


// Função principal
int main(int argc, char *argv[]) {
    if (argc < 7) {
        cerr << "Uso: " << argv[0] << " [arquivo_mapa] [metodo] xi yi xf yf" << endl;
        return 1;
    }

    // Capturar os argumentos
    string filename = argv[1];
    string method = argv[2];
    int xi = stoi(argv[3]);
    int yi = stoi(argv[4]);
    int xf = stoi(argv[5]);
    int yf = stoi(argv[6]);

    vector<string> map;
    int rows = 0, cols = 0;

    if (!readMapFromFile(filename, map, rows, cols)) {
        cerr << "Erro ao ler o mapa." << endl;
        return 1;
    }

    // Construir o grafo a partir do mapa
    unordered_map<int, Node> graph = mapToGraph(map, rows, cols);

     // Executar o método de busca
    if (!executeSearchMethod(method, graph, xi, yi, xf, yf, cols)) {
        return 1;
    }

    return 0;
}

   