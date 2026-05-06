# Guia de Uso e Testes

## Compilação

```bash
cd ~/ROS2_WORKSPACES/robotica_2026_ws
colcon build --packages-select phi_p3dx_localization
source install/setup.bash
```

## Teste 1: Nó de Localização Simples

```bash
# Terminal 1: Launch o nó de localização
ros2 launch phi_p3dx_localization localization.launch.py

# Terminal 2: Verificar os tópicos publicados
ros2 topic list | grep -E "particles|estimated_pose"

# Terminal 3: Ver o conteúdo das partículas
ros2 topic echo /particles --once

# Terminal 4: Ver a pose estimada
ros2 topic echo /estimated_pose --once

# Terminal 5: Ver frequência de publicação
ros2 topic hz /particles
```

## Teste 2: Exemplo com Rotação

```bash
# Terminal 1: Launch o exemplo com rotação
ros2 launch phi_p3dx_localization rotation_example.launch.py

# Terminal 2: Observar como a orientação muda
ros2 topic echo /estimated_pose/pose/pose/orientation
```

## Teste 3: Verificar parâmetros

```bash
# Ver todos os parâmetros do nó
ros2 param list localization_node

# Ver valor específico
ros2 param get localization_node num_particles

# Modificar valor em tempo real
ros2 param set localization_node num_particles 200
```

## RViz

Quando o launch file é executado, RViz abre automaticamente com:
- **Partículas (vermelho)**: PoseArray em `/particles`
- **Pose estimada (verde)**: PoseWithCovarianceStamped em `/estimated_pose`
- **Grid**: Para referência espacial
- **TF**: Árvore de transformações (map → odom)

## Próximos passos para implementação do MCL completo

1. **Subscribe a `/scan`** (dados de laser)
2. **Implementar modelo de sensor** (atualizar pesos)
3. **Subscribe a `/odom`** (odometria)
4. **Implementar modelo de movimento** (predição)
5. **Implementar reamostragem** (normalizar e replicar partículas)
6. **Integrar com mapa** (`nav_msgs/OccupancyGrid`)

## Estrutura para alunos

Os arquivos principales para modificar/estender:

- `include/phi_p3dx_localization/localization_node.hpp`: Adicionar novos publishers/subscribers
- `src/localization_node.cpp`: Implementar callbacks de sensor
- Criar nova subclasse se quiser comportamento diferente
