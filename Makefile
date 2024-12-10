# Nome do executável
EXEC = pathfinder

# Compilador e flags
CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2 -g

# Arquivos fonte e objetos
SRCS = main.cpp algorithms.cpp
OBJS = $(SRCS:.cpp=.o)

# Regra principal
.PHONY: all clean
all: clean $(EXEC)

# Como construir o executável
$(EXEC): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $(OBJS)

# Como limpar os arquivos
clean:
	rm -f $(EXEC) $(OBJS)

# Como compilar arquivos .o a partir de .cpp
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@
