#ifndef PHI_P3DX_LOCALIZATION_NODE_HPP_
#define PHI_P3DX_LOCALIZATION_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/float32.hpp>
#include <vector>
#include <string>
#include <cmath>
#include <random>

/**
 * @brief Estrutura simples para representar uma partícula na localização.
 * 
 * Cada partícula armazena uma pose (x, y, theta) e um peso.
 * O peso será usado posteriormente para reamostragem no MCL completo.
 */
struct Particle
{
  double x;       ///< Posição X no frame "map"
  double y;       ///< Posição Y no frame "map"
  double theta;   ///< Orientação (radianos)
  double weight;  ///< Peso da partícula (inicialmente 1.0 / num_particles)
};

/**
 * @class LocalizationNode
 * @brief Nó de localização simples baseado em filtro de partículas.
 * 
 * Este nó mantém um conjunto de partículas, publica para visualização em RViz,
 * e estima a pose como a média das partículas com covariância simples.
 * 
 * Este é um código didático base para que alunos implementem o MCL completo.
 * 
 * **Tópicos publicados:**
 * - `/particles` (geometry_msgs/msg/PoseArray): Visualização das partículas
 * - `/estimated_pose` (geometry_msgs/msg/PoseWithCovarianceStamped): Pose estimada com covariância
 * 
 * **Parâmetros:**
 * - `num_particles` (int): Número de partículas (default: 100)
 * - `initial_x`, `initial_y`, `initial_theta` (double): Pose inicial (defaults: 0.0)
 * - `initial_perturbation` (double): Desvio padrão da perturbação gaussiana inicial (default: 0.1)
 * - `use_perturbation` (bool): Se true, usa perturbação gaussiana na inicialização (default: false)
 * - `publish_freq` (double): Frequência de publicação em Hz (default: 10.0)
 */
class LocalizationNode : public rclcpp::Node
{
public:
  /**
   * @brief Construtor do LocalizationNode.
   * 
   * @param node_name Nome do nó no ROS 2 (default: "localization_node")
   */
  explicit LocalizationNode(const std::string &node_name = "localization_node");
  virtual ~LocalizationNode() = default;

protected:
  // ============= Publishers =============
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particles_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr estimated_pose_pub_;

  // ============= Timer =============
  rclcpp::TimerBase::SharedPtr publish_timer_;

  // ============= Estado Interno =============
  std::vector<Particle> particles_;
  int num_particles_;
  std::string frame_id_;

  // ============= Geradores de números aleatórios =============
  std::mt19937 rng_;
  std::normal_distribution<double> normal_dist_;

  // ============= Métodos principais =============

  /**
   * @brief Inicializa o conjunto de partículas.
   * 
   * Se `use_perturbation_` é false, todas as partículas são inicializadas
   * na mesma pose (initial_x, initial_y, initial_theta).
   * 
   * Se `use_perturbation_` é true, aplica pequeno ruído gaussiano
   * baseado no parâmetro `initial_perturbation`.
   */
  void initialize_particles();

  /**
   * @brief Calcula a pose estimada como a média das partículas.
   * 
   * Para a orientação, usa média circular:
   * - Calcula mean_sin = média(sin(theta_i))
   * - Calcula mean_cos = média(cos(theta_i))
   * - Resultado: atan2(mean_sin, mean_cos)
   * 
   * @param mean_x Referência para receber a média de X
   * @param mean_y Referência para receber a média de Y
   * @param mean_theta Referência para receber a média de theta (circular)
   */
  void calculate_mean_pose(double &mean_x, double &mean_y, double &mean_theta) const;

  /**
   * @brief Calcula a covariância simples das partículas.
   * 
   * Calcula as variâncias de x, y e theta e retorna uma matriz
   * 6x6 com os valores diagonais preenchidos (outras posições = 0).
   * 
   * @param mean_x Média de X (calculada previamente)
   * @param mean_y Média de Y (calculada previamente)
   * @param mean_theta Média de theta (calculada previamente)
   * @return Matriz 6x6 com covariância (36 elementos)
   */
  std::array<double, 36> calculate_covariance(
    double mean_x, double mean_y, double mean_theta) const;

  /**
   * @brief Publica o conjunto de partículas como PoseArray.
   * 
   * Usado para visualização em RViz.
   */
  void publish_particles();

  /**
   * @brief Publica a pose estimada com covariância.
   * 
   * Calcula média, covariância e publica em `/estimated_pose`.
   */
  void publish_estimated_pose();

  /**
   * @brief Loop principal executado pelo timer.
   * 
   * Chamado a cada 1/publish_freq segundos.
   * Publica partículas e pose estimada.
   */
  void on_timer();

  /**
   * @brief Método virtual para que subclasses possam implementar atualização.
   * 
   * Será sobrescrito em classes filhas (ex: exemplo com rotação).
   * De momento, não faz nada.
   */
  virtual void update_particles() {}

private:
  // Parâmetros
  double initial_x_;
  double initial_y_;
  double initial_theta_;
  double initial_perturbation_;
  bool use_perturbation_;
  double publish_freq_;
};

#endif  // PHI_P3DX_LOCALIZATION_NODE_HPP_
