# Compilador e flags
CXX = g++
CXXFLAGS = -I /usr/include/eigen3 -I include -std=c++11 -O2

# Arquivos de origem e objetos
SRCS = src/ForwardKinematicsDH.cpp src/InverseKinematics.cpp src/Joint.cpp example.cpp
OBJS = $(SRCS:.cpp=.o)

# Nome do executável
TARGET = robot_arm

# Regras padrão
all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(OBJS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Regra para limpar os arquivos gerados
clean:
	rm -f $(OBJS) $(TARGET)

# Regra para rodar o executável
run: $(TARGET)
	./$(TARGET)
