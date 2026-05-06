# phi_p3dx_localization

Pacote didático de **localização** para o robô Pioneer 3-DX usando **filtro de partículas (MCL)**.

## Objetivo

Fornecer uma base educacional para implementação de localização Monte Carlo Localization (MCL) em robôs móveis. O código atual implementa a estrutura básica (partículas, publicação, cálculo de média e covariância) sem os componentes avançados do MCL completo (atualização com sensores, reamostragem, modelo de movimento).

## O que está implementado

### LocalizationNode (nó base)

- **Gerenciamento de partículas**: Armazena e inicializa partículas (x, y, theta, weight)
- **Publicação de tópicos**:
  - `/particles` (`geometry_msgs/msg/PoseArray`): Visualização das partículas no RViz
  - `/estimated_pose` (`geometry_msgs/msg/PoseWithCovarianceStamped`): Pose estimada (média das partículas) com covariância
- **Cálculo de estimativa**:
  - Média simples de x e y
  - Média circular para theta (usando `atan2(mean_sin, mean_cos)`)
  - Covariância simples baseada na dispersão das partículas
- **Parâmetros ROS**:
  - `num_particles` (int, default: 100)
  - `initial_x`, `initial_y`, `initial_theta` (double, defaults: 0.0)
  - `initial_perturbation` (double, default: 0.1)
  - `use_perturbation` (bool, default: false)
  - `publish_freq` (double, default: 10.0 Hz)
  - `frame_id` (string, default: "map")

### RotationExample (nó didático)

Subclasse de `LocalizationNode` que rotaciona continuamente as partículas para demonstrar:
- Funcionamento da média de orientação
- Visualização de como a covariância se comporta durante rotação
- Exemplo de como estender o nó base

**Parâmetro adicional:**
- `rotation_speed` (double, default: 0.5 rad/s)

## Como usar

### 1. Compilar

```bash
cd ~/ROS2_WORKSPACES/robotica_2026_ws
colcon build --packages-select phi_p3dx_localization
source install/setup.bash
```

### 2. Executar o nó de localização simples

```bash
ros2 launch phi_p3dx_localization localization.launch.py
```

Isso inicia:
- Nó de localização com 100 partículas estáticas
- RViz com visualização de partículas e pose estimada
- Transformação TF estática (map → odom)

### 3. Executar o exemplo de rotação

```bash
ros2 launch phi_p3dx_localization rotation_example.launch.py
```

Isso inicia:
- Exemplo com rotação contínua das partículas
- As partículas giram continuamente; observe como a pose estimada acompanha a rotação

### 4. Inspecionar tópicos (terminal separado)

```bash
# Ver partículas
ros2 topic echo /particles

# Ver pose estimada
ros2 topic echo /estimated_pose

# Ver frequência de publicação
ros2 topic hz /particles
```

## Estrutura do código

```
include/phi_p3dx_localization/
├── localization_node.hpp       # Classe base do nó de localização
└── rotation_example.hpp        # Exemplo didático

src/
├── localization_node.cpp       # Implementação do LocalizationNode
├── localization_main.cpp       # Ponto de entrada do nó de localização
├── rotation_example.cpp        # Implementação do RotationExample
└── rotation_example_main.cpp   # Ponto de entrada do exemplo

launch/
├── localization.launch.py      # Launch file da localização básica
└── rotation_example.launch.py  # Launch file do exemplo de rotação

rviz/
└── localization.rviz           # Configuração do RViz
```

## Para implementar o MCL completo

Os alunos precisarão adicionar:

1. **Callback de sensor de laser** (para atualizar pesos das partículas)
   - Comparar leitura de laser com mapa conhecido
   - Atualizar `particle.weight` baseado na probabilidade

2. **Modelo de movimento** (predição)
   - Atualizar posição das partículas baseado em odometria ou comando de velocidade
   - Adicionar ruído ao movimento

3. **Reamostragem** (resampling)
   - Remover partículas com baixo peso
   - Duplicar partículas com alto peso
   - Manter número total constante

4. **Integração com ROS**
   - Subscribe a `/odom` para odometria
   - Subscribe a `/scan` para laser
   - Implementar rotinas de atualização no timer existente

## Notas didáticas

- O código utiliza **herança virtual** (`update_particles()`) para permitir fácil extensão
- A estrutura de dados é simples e didática, **não otimizada** para performance
- Os comentários em português facilitam compreensão para alunos brasileiros
- Cada classe tem documentação Doxygen

## Dependências

- ROS 2 (testado em Humble)
- C++17 ou superior
- rviz2
- geometry_msgs
- tf2_ros

## Referências

- **Filtro de Partículas**: Thrun et al., "Probabilistic Robotics"
- **ROS 2**: https://docs.ros.org/en/humble/
- **RViz 2**: https://github.com/ros2/rviz
