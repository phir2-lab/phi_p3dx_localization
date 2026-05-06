#include "phi_p3dx_localization/localization_node.hpp"
#include <cmath>
#include <numeric>

LocalizationNode::LocalizationNode(const std::string &node_name)
  : Node(node_name),
    num_particles_(100),
    frame_id_("map"),
    rng_(std::random_device{}()),
    normal_dist_(0.0, 1.0)
{
  // Declarar e obter parâmetros
  this->declare_parameter<int>("num_particles", 100);
  this->declare_parameter<double>("initial_x", 0.0);
  this->declare_parameter<double>("initial_y", 0.0);
  this->declare_parameter<double>("initial_theta", 0.0);
  this->declare_parameter<double>("initial_perturbation", 0.1);
  this->declare_parameter<bool>("use_perturbation", false);
  this->declare_parameter<double>("publish_freq", 10.0);
  this->declare_parameter<std::string>("frame_id", "map");

  num_particles_ = this->get_parameter("num_particles").as_int();
  initial_x_ = this->get_parameter("initial_x").as_double();
  initial_y_ = this->get_parameter("initial_y").as_double();
  initial_theta_ = this->get_parameter("initial_theta").as_double();
  initial_perturbation_ = this->get_parameter("initial_perturbation").as_double();
  use_perturbation_ = this->get_parameter("use_perturbation").as_bool();
  publish_freq_ = this->get_parameter("publish_freq").as_double();
  frame_id_ = this->get_parameter("frame_id").as_string();

  // Cria publishers
  particles_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
    "particles", rclcpp::QoS(10).transient_local());
  
  estimated_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "estimated_pose", rclcpp::QoS(10));

  // Timer para publicação (ex: 10 Hz = 0.1 s)
  double timer_period = 1.0 / publish_freq_;
  publish_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(timer_period),
    std::bind(&LocalizationNode::on_timer, this));

  // Inicializa partículas
  initialize_particles();

  RCLCPP_INFO(this->get_logger(), 
    "[%s] inicializado com %d partículas. Freq: %.1f Hz. Frame: %s",
    node_name.c_str(), num_particles_, publish_freq_, frame_id_.c_str());
}

void LocalizationNode::initialize_particles()
{
  particles_.clear();
  particles_.resize(num_particles_);

  double weight = 1.0 / num_particles_;

  if (use_perturbation_) {
    // Inicializa com pequena perturbação gaussiana
    for (int i = 0; i < num_particles_; ++i) {
      particles_[i].x = initial_x_ + normal_dist_(rng_) * initial_perturbation_;
      particles_[i].y = initial_y_ + normal_dist_(rng_) * initial_perturbation_;
      particles_[i].theta = initial_theta_ + normal_dist_(rng_) * initial_perturbation_;
      particles_[i].weight = weight;
    }
    RCLCPP_INFO(this->get_logger(), 
      "Partículas inicializadas com perturbação gaussiana (σ=%.3f)", 
      initial_perturbation_);
  } else {
    // Todas na mesma pose
    for (int i = 0; i < num_particles_; ++i) {
      particles_[i].x = initial_x_;
      particles_[i].y = initial_y_;
      particles_[i].theta = initial_theta_;
      particles_[i].weight = weight;
    }
    RCLCPP_INFO(this->get_logger(), 
      "Partículas inicializadas na mesma pose: (%.2f, %.2f, %.2f)", 
      initial_x_, initial_y_, initial_theta_);
  }
}

void LocalizationNode::calculate_mean_pose(double &mean_x, double &mean_y, double &mean_theta) const
{
  if (particles_.empty()) {
    mean_x = 0.0;
    mean_y = 0.0;
    mean_theta = 0.0;
    return;
  }

  // Média simples de X e Y
  mean_x = 0.0;
  mean_y = 0.0;
  double sum_sin = 0.0;
  double sum_cos = 0.0;

  for (const auto &p : particles_) {
    mean_x += p.x;
    mean_y += p.y;
    sum_sin += std::sin(p.theta);
    sum_cos += std::cos(p.theta);
  }

  mean_x /= particles_.size();
  mean_y /= particles_.size();

  // Média circular para theta
  double mean_sin = sum_sin / particles_.size();
  double mean_cos = sum_cos / particles_.size();
  mean_theta = std::atan2(mean_sin, mean_cos);
}

std::array<double, 36> LocalizationNode::calculate_covariance(
  double mean_x, double mean_y, double mean_theta) const
{
  std::array<double, 36> cov;
  cov.fill(0.0);

  if (particles_.empty()) {
    return cov;
  }

  // Variância de X (índice [0,0])
  double var_x = 0.0;
  double var_y = 0.0;
  double var_theta = 0.0;

  for (const auto &p : particles_) {
    double dx = p.x - mean_x;
    double dy = p.y - mean_y;
    
    // Para theta, usar diferença angular
    double dtheta = p.theta - mean_theta;
    // Normalizar para [-π, π]
    while (dtheta > M_PI) dtheta -= 2.0 * M_PI;
    while (dtheta < -M_PI) dtheta += 2.0 * M_PI;

    var_x += dx * dx;
    var_y += dy * dy;
    var_theta += dtheta * dtheta;
  }

  var_x /= particles_.size();
  var_y /= particles_.size();
  var_theta /= particles_.size();

  // Preencher matriz 6x6 (apenas diagonais principais)
  cov[0] = var_x;      // (0,0) - variância de X
  cov[7] = var_y;      // (1,1) - variância de Y
  cov[35] = var_theta; // (5,5) - variância de theta

  return cov;
}

void LocalizationNode::publish_particles()
{
  if (particles_.empty()) {
    return;
  }

  auto msg = std::make_shared<geometry_msgs::msg::PoseArray>();
  msg->header.frame_id = frame_id_;
  msg->header.stamp = this->now();
  msg->poses.resize(particles_.size());

  for (size_t i = 0; i < particles_.size(); ++i) {
    const auto &p = particles_[i];
    msg->poses[i].position.x = p.x;
    msg->poses[i].position.y = p.y;
    msg->poses[i].position.z = 0.0;

    // Converter theta para quaternion (rotação (yaw) apenas em Z)
    double half_theta = p.theta / 2.0;
    msg->poses[i].orientation.x = 0.0;
    msg->poses[i].orientation.y = 0.0;
    msg->poses[i].orientation.z = std::sin(half_theta);
    msg->poses[i].orientation.w = std::cos(half_theta);
  }

  particles_pub_->publish(*msg);
}

void LocalizationNode::publish_estimated_pose()
{
  double mean_x, mean_y, mean_theta;
  calculate_mean_pose(mean_x, mean_y, mean_theta);

  auto cov = calculate_covariance(mean_x, mean_y, mean_theta);

  auto msg = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
  msg->header.frame_id = frame_id_;
  msg->header.stamp = this->now();

  // Posição
  msg->pose.pose.position.x = mean_x;
  msg->pose.pose.position.y = mean_y;
  msg->pose.pose.position.z = 0.0;

  // Orientação (quaternion da média theta)
  double half_theta = mean_theta / 2.0;
  msg->pose.pose.orientation.x = 0.0;
  msg->pose.pose.orientation.y = 0.0;
  msg->pose.pose.orientation.z = std::sin(half_theta);
  msg->pose.pose.orientation.w = std::cos(half_theta);

  // Covariância
  msg->pose.covariance = cov;

  estimated_pose_pub_->publish(*msg);
}

void LocalizationNode::on_timer()
{
  // Permite que subclasses implementem atualização de partículas
  update_particles();

  // Publica partículas e pose estimada
  publish_particles();
  publish_estimated_pose();
}
