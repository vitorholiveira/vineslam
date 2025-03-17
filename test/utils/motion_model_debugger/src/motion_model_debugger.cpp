#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <random>
#include <vineslam/localization/pf.hpp>

const float DEGREE_TO_RAD = static_cast<float>(M_PI / 180.);

class MotionModelDebugger : public rclcpp::Node
{
public:
  MotionModelDebugger() : Node("motion_model_debugger"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
  {
    // Parâmetros para o modelo
    this->declare_parameter<int>("n_particles", 300);
    this->declare_parameter<double>("sigma_xx", 0.2);
    this->declare_parameter<double>("sigma_yy", 0.5);
    this->declare_parameter<double>("sigma_zz", 0.1);
    this->declare_parameter<double>("sigma_RR", 0.1);
    this->declare_parameter<double>("sigma_PP", 0.1);
    this->declare_parameter<double>("sigma_YY", 1.0);
    this->get_parameter("n_particles", n_particles_);
    this->get_parameter("sigma_xx", sigma_xx_);
    this->get_parameter("sigma_yy", sigma_yy_);
    this->get_parameter("sigma_zz", sigma_zz_);
    this->get_parameter("sigma_RR", sigma_RR_);
    this->get_parameter("sigma_PP", sigma_PP_);
    this->get_parameter("sigma_YY", sigma_YY_);

    particles_.resize(n_particles_);
    has_previous_transform_ = false;

    // Publisher para enviar as partículas como PoseArray
    particles_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("particles", 10);
    mean_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("mean", 10);

    // Timer para atualizar as partículas
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&MotionModelDebugger::update_particles, this));
  }

private:
  void update_particles()
  {
    // Tenta receber a transformação map -> base_link
    geometry_msgs::msg::TransformStamped current_transform;
    try
    {
      current_transform = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
    }
    catch (tf2::TransformException& ex)
    {
      RCLCPP_WARN(this->get_logger(), "Could not transform map to base_link: %s", ex.what());
      return;
    }

    // Calcula o incremento de translação e rotação se já temos uma transformação anterior
    if (has_previous_transform_)
    {
      geometry_msgs::msg::TransformStamped delta_transform;
      calculate_transform_increment(previous_transform_, current_transform, delta_transform);
      vineslam::Pose odom_inc;
      odom_inc.x_ = delta_transform.transform.translation.x;
      odom_inc.y_ = delta_transform.transform.translation.y;
      odom_inc.Y_ = tf2::getYaw(delta_transform.transform.rotation);

      // Aplica o modelo de movimento às partículas usando o delta calculado
      apply_motion_model(odom_inc);

      // Publica o array de poses das partículas
      geometry_msgs::msg::PoseArray pose_array;
      geometry_msgs::msg::Pose pose;
      pose_array.header.stamp = this->now();
      pose_array.header.frame_id = "map";

      // Variáveis para acumular a média das posições e orientação
      double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
      tf2::Quaternion sum_orientation(0.0, 0.0, 0.0, 0.0);

      for (const auto& particle : particles_)
      {
        tf2::Quaternion q_particle;
        q_particle.setRPY(particle.p_.R_, particle.p_.P_, particle.p_.Y_);
        q_particle.normalize();

        // Acumula a posição
        sum_x += particle.p_.x_;
        sum_y += particle.p_.y_;
        sum_z += particle.p_.z_;

        // Acumula o quaternion da orientação
        sum_orientation += q_particle;

        // Adiciona a pose individual ao array de partículas
        pose.position.x = particle.p_.x_;
        pose.position.y = particle.p_.y_;
        pose.position.z = particle.p_.z_;
        pose.orientation.x = q_particle.x();
        pose.orientation.y = q_particle.y();
        pose.orientation.z = q_particle.z();
        pose.orientation.w = q_particle.w();
        pose_array.poses.push_back(pose);
      }

      // Publica o array de partículas
      particles_pub_->publish(pose_array);

      // Calcula a pose média
      geometry_msgs::msg::PoseStamped mean_pose;
      mean_pose.header.stamp = this->now();
      mean_pose.header.frame_id = "map";

      mean_pose.pose.position.x = sum_x / particles_.size();
      mean_pose.pose.position.y = sum_y / particles_.size();
      mean_pose.pose.position.z = sum_z / particles_.size();

      // Normaliza o quaternion da orientação média
      sum_orientation.normalize();
      mean_pose.pose.orientation.x = sum_orientation.x();
      mean_pose.pose.orientation.y = sum_orientation.y();
      mean_pose.pose.orientation.z = sum_orientation.z();
      mean_pose.pose.orientation.w = sum_orientation.w();

      // Publica a pose média
      mean_pose_pub_->publish(mean_pose);
    }
    else
    {
      // Initialize all particles
      vineslam::Pose initial_pose;
      initial_pose.x_ = current_transform.transform.translation.x;
      initial_pose.y_ = current_transform.transform.translation.y;
      initial_pose.z_ = 0.0;
      initial_pose.R_ = 0.0;
      initial_pose.P_ = 0.0;
      initial_pose.Y_ = tf2::getYaw(current_transform.transform.rotation);
      for (size_t i = 0; i < n_particles_; i++)
      {
        // Calculate the initial pose for each particle considering
        // - the input initial pose
        // - a sample distribution to spread the particles
        vineslam::Pose m_pose = initial_pose + vineslam::Pose(sampleGaussian(0.2, 0), sampleGaussian(0.2, 0), sampleGaussian(0.1, 0), sampleGaussian(1 * DEGREE_TO_RAD, 0), sampleGaussian(1 * DEGREE_TO_RAD, 0), sampleGaussian(10 * DEGREE_TO_RAD, 0));
        // Compute initial weight of each particle
        float weight = 1.;
        // Insert the particle into the particles array
        particles_[i] = vineslam::Particle(i, m_pose, weight);
      }

    }

    // Armazena a transformação atual para a próxima iteração
    previous_transform_ = current_transform;
    has_previous_transform_ = true;
  }

  void apply_motion_model(const vineslam::Pose& odom_inc)
  {
    float d_trans = odom_inc.norm3D();

    // Innovate particles
    for (auto& particle : particles_)
    {
      // Build pose noise transformation matrix
      vineslam::Pose pose_noise;
      pose_noise.x_ = sigma_xx_ * d_trans * sampleGaussian(1.0, 0);                 // xx
      pose_noise.y_ = sigma_yy_ * d_trans * sampleGaussian(1.0, 0);                 // yy
      pose_noise.z_ = sigma_zz_ * d_trans * sampleGaussian(1.0, 0);                 // zz
      pose_noise.R_ = sigma_RR_ * d_trans * sampleGaussian(1.0, 0);                 // RR
      pose_noise.P_ = sigma_PP_ * d_trans * sampleGaussian(1.0, 0);                 // PP
      // pose_noise.Y_ = sigma_YY_ * std::fabs(odom_inc.Y_) * sampleGaussian(1.0, 0);  // YY
      pose_noise.Y_ = sigma_YY_ * std::log(1 + std::fabs(odom_inc.Y_)) * sampleGaussian(1.0, 0);

      std::array<float, 9> R_noise{};
      pose_noise.toRotMatrix(R_noise);
      vineslam::Tf odom_noise_tf(R_noise, std::array<float, 3>{ pose_noise.x_, pose_noise.y_, pose_noise.z_ });

      // Built odom increment transformation matrix
      std::array<float, 9> R_inc{};
      odom_inc.toRotMatrix(R_inc);
      vineslam::Tf odom_inc_tf(R_inc, std::array<float, 3>{ odom_inc.x_, odom_inc.y_, odom_inc.z_ });

      // Final pose increment
      vineslam::Tf innovation_tf = odom_inc_tf * odom_noise_tf;

      // Save info for the next iteration
      particle.pp_ = particle.p_;
      particle.ptf_ = particle.tf_;

      // Apply transformation
      particle.tf_ = particle.ptf_ * innovation_tf;
      particle.p_ = vineslam::Pose(particle.tf_.R_array_, particle.tf_.t_array_);
    }
  }

  float sampleGaussian(const float& sigma, const unsigned long int& S)
  {
    if (S != 0)
      srand48(S);
    if (sigma == 0)
      return 0.;

    float x1, x2, w;
    float r;

    do
    {
      do
      {
        r = drand48();
      } while (r == 0.0);
      x1 = 2.0 * r - 1.0;
      do
      {
        r = drand48();
      } while (r == 0.0);
      x2 = 2.0 * drand48() - 1.0;
      w = x1 * x1 + x2 * x2;
    } while (w > 1.0 || w == 0.0);

    return (sigma * x2 * sqrt(-2.0 * log(w) / w));
  }

  void calculate_transform_increment(const geometry_msgs::msg::TransformStamped& previous, const geometry_msgs::msg::TransformStamped& current, geometry_msgs::msg::TransformStamped& delta)
  {
    // Calcula o delta de translação
    delta.transform.translation.x = current.transform.translation.x - previous.transform.translation.x;
    delta.transform.translation.y = current.transform.translation.y - previous.transform.translation.y;
    delta.transform.translation.z = current.transform.translation.z - previous.transform.translation.z;

    // Calcula o delta de rotação (Yaw, Roll e Pitch)
    tf2::Quaternion q_previous, q_current, q_delta;
    tf2::fromMsg(previous.transform.rotation, q_previous);
    tf2::fromMsg(current.transform.rotation, q_current);
    q_delta = q_current * q_previous.inverse();  // Delta de rotação

    delta.transform.rotation = tf2::toMsg(q_delta);
  }

  // Parâmetros e variáveis do nó
  int n_particles_;
  double sigma_xx_, sigma_yy_, sigma_zz_;
  double sigma_RR_, sigma_PP_, sigma_YY_;

  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particles_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr mean_pose_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  geometry_msgs::msg::TransformStamped previous_transform_;
  bool has_previous_transform_;

  std::vector<vineslam::Particle> particles_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotionModelDebugger>());
  rclcpp::shutdown();
  return 0;
}