#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <chrono>
#include <memory>
#include <cmath> // For fabs, isnan

using namespace std::chrono_literals;
using std::placeholders::_1;

class BalanceController : public rclcpp::Node {
public:
    BalanceController() : Node("balance_controller"), kp_(0.0), ki_(0.0), kd_(0.0),
                          target_pitch_(0.0), integral_(0.0), prev_error_(0.0),
                          max_integral_(0.0), max_output_vel_(0.0), first_run_(true)
    {
        // --- Parameters ---
        this->declare_parameter<double>("kp", 5000.0);       // Proportional gain (NEEDS TUNING)
        this->declare_parameter<double>("ki", 1.0);        // Integral gain (NEEDS TUNING)
        this->declare_parameter<double>("kd", 0.8);       // Derivative gain (NEEDS TUNING)
        this->declare_parameter<double>("target_pitch", 0.0); // Target angle in radians (0 = upright)
        this->declare_parameter<double>("max_integral", 2.0);  // Anti-windup limit for integral term
        this->declare_parameter<double>("max_output_vel", 1000); // Max linear velocity command (m/s)
        this->declare_parameter<std::string>("imu_topic", "/imu");
        this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
        this->declare_parameter<double>("loop_rate", 5000.0); // Control loop frequency (Hz)

        // Retrieve parameters
        this->get_parameter("kp", kp_);
        this->get_parameter("ki", ki_);
        this->get_parameter("kd", kd_);
        this->get_parameter("target_pitch", target_pitch_);
        this->get_parameter("max_integral", max_integral_);
        this->get_parameter("max_output_vel", max_output_vel_);
        std::string imu_topic = this->get_parameter("imu_topic").as_string();
        std::string cmd_vel_topic = this->get_parameter("cmd_vel_topic").as_string();
        double loop_rate = this->get_parameter("loop_rate").as_double();

        RCLCPP_INFO(this->get_logger(), "PID Gains: Kp=%.2f, Ki=%.2f, Kd=%.2f", kp_, ki_, kd_);
        RCLCPP_INFO(this->get_logger(), "Target Pitch: %.2f rad", target_pitch_);
        RCLCPP_INFO(this->get_logger(), "Max Integral: %.2f", max_integral_);
        RCLCPP_INFO(this->get_logger(), "Max Output Vel: %.2f m/s", max_output_vel_);
        RCLCPP_INFO(this->get_logger(), "Subscribing to IMU topic: %s", imu_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Publishing to CMD_VEL topic: %s", cmd_vel_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Control loop rate: %.1f Hz", loop_rate);


        // --- QoS Profile ---
        // Use sensor data QoS for high-frequency, potentially lossy sensor data like IMU
        rclcpp::QoS sensor_qos = rclcpp::SensorDataQoS();

        // --- Publisher and Subscriber ---
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, sensor_qos, std::bind(&BalanceController::imu_callback, this, _1));

        // --- Timer for Control Loop ---
        // It's often better to run the control logic at a fixed rate using a timer
        // rather than directly in the IMU callback, especially if IMU rate > control rate.
        // However, for simplicity here, we'll compute in the callback based on dt.
        // If issues arise, switch to a timer-based approach.
        last_time_ = this->get_clock()->now();
    }

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        rclcpp::Time current_time = this->get_clock()->now();
        double dt = (current_time - last_time_).seconds();

        // Avoid division by zero or large dt on first run or after delays
        if (dt <= 0.0 || dt > 0.5) {
             RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Unstable dt detected: %.3f. Resetting PID.", dt);
             reset_pid();
             last_time_ = current_time;
            return;
        }

        // Extract pitch angle from quaternion
        tf2::Quaternion q(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

         // Basic check for valid pitch reading
         if (std::isnan(pitch)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Received NaN pitch value. Skipping control step.");
            return;
         }

        // Calculate error
        double error = target_pitch_ - pitch;

        // Calculate integral term (with anti-windup)
        integral_ += error * dt;
        integral_ = std::clamp(integral_, -max_integral_, max_integral_); // Clamp integral term

        // Calculate derivative term
        double derivative = (error - prev_error_) / dt;

        // Calculate PID output (maps to linear velocity)
        double output_vel = kp_ * error + ki_ * integral_ + kd_ * derivative;

        // Clamp output velocity
        output_vel = std::clamp(output_vel, -max_output_vel_, max_output_vel_);

        // Create and publish Twist message
        auto twist_msg = std::make_unique<geometry_msgs::msg::Twist>();
        twist_msg->linear.x = output_vel; // Control forward/backward velocity
        twist_msg->angular.z = 0.0;       // No turning command for now

        cmd_vel_pub_->publish(std::move(twist_msg));

        // Update state for next iteration
        prev_error_ = error;
        last_time_ = current_time;
        first_run_ = false; // No longer the first run
    }

    void reset_pid() {
        integral_ = 0.0;
        prev_error_ = 0.0;
        first_run_ = true;

        // Publish zero velocity when resetting
        auto twist_msg = std::make_unique<geometry_msgs::msg::Twist>();
        twist_msg->linear.x = 0.0;
        twist_msg->angular.z = 0.0;
        cmd_vel_pub_->publish(std::move(twist_msg));
        RCLCPP_INFO(this->get_logger(), "PID controller reset.");
    }


    // PID parameters
    double kp_, ki_, kd_;
    double target_pitch_;
    double max_integral_;
    double max_output_vel_;

    // PID state variables
    double integral_;
    double prev_error_;
    bool first_run_;
    rclcpp::Time last_time_;


    // ROS interfaces
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

// Main function
int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto balance_controller_node = std::make_shared<BalanceController>();
    RCLCPP_INFO(balance_controller_node->get_logger(), "Balance Controller Node Started");
    rclcpp::spin(balance_controller_node);
    rclcpp::shutdown();
    return 0;
}
