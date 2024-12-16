#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <vector>
#include <cmath>

#define deg2rad(x) ((x) * M_PI / 180.0)

// Offset constants
enum JointOffset
{
    hipLOffset = 105,
    kneeLOffset = 155,
    ankleLOffset = 85,
    hipROffset = 82,
    kneeROffset = 25,
    ankleROffset = 85
};

class BipedJointController : public rclcpp::Node
{
public:
    BipedJointController() : Node("biped_joint_controller")
    {
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/forward_position_controller/commands", 10);
        clock_publisher_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
        system_clock_ = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(20),
                                         std::bind(&BipedJointController::timer_callback, this));

        joint_positions_.resize(6, 0.0);
        leg_length_thigh_ = 0.1;  // 10cm
        leg_length_shin_ = 0.114; // 11.4cm

        // Gait parameters with reduced left leg movement
        this->declare_parameter("step_height", 0.02);
        this->declare_parameter("step_length", 0.02);
        this->declare_parameter("cycle_duration", 2.0);
        this->declare_parameter("left_leg_scale", 0.7); // Scale factor for left leg movement
        this->declare_parameter("ankle_limit", 0.2);    // Limit ankle movement to ±0.2 radians

        current_phase_ = 0.0;
        RCLCPP_INFO(this->get_logger(), "Modified Biped Controller initialized");
    }

private:
    bool calculate_leg_ik(double x, double z, std::vector<double> &angles)
    {
        double L1 = leg_length_thigh_;
        double L2 = leg_length_shin_;
        double D = std::sqrt(x * x + z * z);

        if (D > (L1 + L2))
        {
            RCLCPP_WARN(this->get_logger(), "Target out of reach: D=%.3f > %.3f", D, L1 + L2);
            return false;
        }

        // Knee angle
        double cos_knee = (L1 * L1 + L2 * L2 - D * D) / (2.0 * L1 * L2);
        double knee_angle = M_PI - std::acos(cos_knee);

        // Hip angle
        double alpha = std::atan2(x, z);
        double cos_hip1 = (L1 * L1 + D * D - L2 * L2) / (2.0 * L1 * D);
        double hip_angle = alpha + std::acos(cos_hip1);

        // Ankle angle (limited range)
        double ankle_angle = -(hip_angle + knee_angle);
        double ankle_limit = this->get_parameter("ankle_limit").as_double();
        ankle_angle = std::clamp(ankle_angle, -ankle_limit, ankle_limit);

        angles = {hip_angle, knee_angle, ankle_angle};
        return true;
    }

    void timer_callback()
    {
        double step_height = this->get_parameter("step_height").as_double();
        double step_length = this->get_parameter("step_length").as_double();
        double cycle_duration = this->get_parameter("cycle_duration").as_double();
        double left_leg_scale = this->get_parameter("left_leg_scale").as_double();

        current_phase_ += (0.01 / cycle_duration);
        if (current_phase_ > 1.0)
            current_phase_ -= 1.0;

        // Right Leg Trajectory (unchanged)
        double x_right, z_right;
        if (current_phase_ < 0.5)
        {
            double swing_phase = current_phase_ * 2.0;
            x_right = step_length * std::cos(M_PI * swing_phase);
            z_right = step_height + 0.01 * std::sin(M_PI * swing_phase);
        }
        else
        {
            double stance_phase = (current_phase_ - 0.5) * 2.0;
            x_right = -step_length * std::cos(M_PI * stance_phase);
            z_right = step_height;
        }

        // Left Leg Trajectory (scaled down)
        double x_left, z_left;
        double left_phase = (current_phase_ < 0.5) ? current_phase_ + 0.5 : current_phase_ - 0.5;
        if (left_phase < 0.5)
        {
            double swing_phase = left_phase * 2.0;
            x_left = left_leg_scale * step_length * std::cos(M_PI * swing_phase);
            z_left = step_height + left_leg_scale * 0.01 * std::sin(M_PI * swing_phase);
        }
        else
        {
            double stance_phase = (left_phase - 0.5) * 2.0;
            x_left = -left_leg_scale * step_length * std::cos(M_PI * stance_phase);
            z_left = step_height;
        }

        std::vector<double> left_angles, right_angles;
        if (!calculate_leg_ik(x_left, z_left, left_angles) ||
            !calculate_leg_ik(x_right, z_right, right_angles))
        {
            RCLCPP_ERROR(this->get_logger(), "IK calculation failed");
            return;
        }

        // Apply joint positions with offset transformations
        joint_positions_[0] = deg2rad(JointOffset::hipLOffset) - left_angles[0];
        joint_positions_[1] = deg2rad(JointOffset::kneeLOffset) - left_angles[1];
        joint_positions_[2] = -left_angles[2] + M_PI / 2; // Ankle (limited range applied in IK)

        joint_positions_[3] = deg2rad(JointOffset::hipROffset) - right_angles[0] * 0.75;
        joint_positions_[4] = deg2rad(JointOffset::kneeROffset) + right_angles[1] - M_PI;
        joint_positions_[5] = right_angles[2] - M_PI / 2; // Ankle (limited range applied in IK)

        // Publish commands
        auto msg = std_msgs::msg::Float64MultiArray();
        std_msgs::msg::MultiArrayDimension dim;
        dim.label = "position";
        dim.size = 6;
        dim.stride = 1;
        msg.layout.dim.push_back(dim);
        msg.data = joint_positions_;
        publisher_->publish(msg);

        // Update simulation clock
        auto current_time = system_clock_->now();
        auto clock_msg = rosgraph_msgs::msg::Clock();
        clock_msg.clock = current_time;
        clock_publisher_->publish(clock_msg);
    }

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_publisher_;
    rclcpp::Clock::SharedPtr system_clock_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<double> joint_positions_;
    double current_phase_;
    double leg_length_thigh_;
    double leg_length_shin_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BipedJointController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}