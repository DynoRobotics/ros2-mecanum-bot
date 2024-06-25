
#ifndef __DEBICT_MECANUMBOT_CONTROLLER__MECANUMBOT_DRIVE_CONTROLLER_H__
#define __DEBICT_MECANUMBOT_CONTROLLER__MECANUMBOT_DRIVE_CONTROLLER_H__

#include <controller_interface/controller_interface.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <realtime_tools/realtime_buffer.h>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <string>

#include "mecanumbot_controller/mecanumbot_plate.hpp"
#include "mecanumbot_controller/mecanumbot_wheel.hpp"
#include "mecanumbot_controller/mecanumbot_controller_compiler.h"
#include <chrono>

namespace debict
{
    namespace mecanumbot
    {
        namespace controller
        {
            using Twist = geometry_msgs::msg::Twist;
            using Float64 = std_msgs::msg::Float64;
            using Bool = std_msgs::msg::Bool;
            using namespace std::chrono_literals;

            class MecanumbotDriveController
                : public controller_interface::ControllerInterface
            {
            public:
                DEBICT_MECANUMBOT_CONTROLLER_PUBLIC
                MecanumbotDriveController();
                
                DEBICT_MECANUMBOT_CONTROLLER_PUBLIC
                controller_interface::InterfaceConfiguration command_interface_configuration() const override;

                DEBICT_MECANUMBOT_CONTROLLER_PUBLIC
                controller_interface::InterfaceConfiguration state_interface_configuration() const override;

                DEBICT_MECANUMBOT_CONTROLLER_PUBLIC
                controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

                DEBICT_MECANUMBOT_CONTROLLER_PUBLIC
                controller_interface::CallbackReturn on_init() override;

                DEBICT_MECANUMBOT_CONTROLLER_PUBLIC
                controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

                DEBICT_MECANUMBOT_CONTROLLER_PUBLIC
                controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

                DEBICT_MECANUMBOT_CONTROLLER_PUBLIC
                controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

                DEBICT_MECANUMBOT_CONTROLLER_PUBLIC
                controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

                DEBICT_MECANUMBOT_CONTROLLER_PUBLIC
                controller_interface::CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

                DEBICT_MECANUMBOT_CONTROLLER_PUBLIC
                controller_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
            
            protected:
                std::shared_ptr<MecanumbotWheel> get_wheel(const std::string & wheel_joint_name);

                std::shared_ptr<MecanumbotPlate> get_plate(const std::string & plate_front_joint_name, const std::string & plate_rear_joint_name,
                                                           const std::string & gpio_input_name, const std::string & gpio_output_name);

                bool reset();

            protected:
                // timestamp of when we received the last command
                rclcpp::Time last_command_timestamp_;

                rclcpp::Subscription<Twist>::SharedPtr velocity_command_subsciption_;
                realtime_tools::RealtimeBuffer<std::shared_ptr<Twist>> velocity_command_ptr_;

                rclcpp::Subscription<Float64>::SharedPtr plate_height_command_subsciption_;
                realtime_tools::RealtimeBuffer<std::shared_ptr<Float64>> plate_height_command_ptr_;

                rclcpp::Subscription<Float64>::SharedPtr plate_angle_command_subsciption_;
                realtime_tools::RealtimeBuffer<std::shared_ptr<Float64>> plate_angle_command_ptr_;

                rclcpp::Subscription<Bool>::SharedPtr plate_homing_command_subsciption_;
                realtime_tools::RealtimeBuffer<std::shared_ptr<Bool>> plate_homing_command_ptr_;

                rclcpp::Publisher<Float64>::SharedPtr plate_angle_feedback_publisher_;

                rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr wheel_odometry_publisher_;

                rclcpp::TimerBase::SharedPtr feedback_timer_;

                std::shared_ptr<MecanumbotWheel> fl_wheel_;
                std::shared_ptr<MecanumbotWheel> fr_wheel_;
                std::shared_ptr<MecanumbotWheel> rl_wheel_;
                std::shared_ptr<MecanumbotWheel> rr_wheel_;
                std::shared_ptr<MecanumbotPlate> plate_;
                
                std::string fl_wheel_joint_name_;
                std::string fr_wheel_joint_name_;
                std::string rl_wheel_joint_name_;
                std::string rr_wheel_joint_name_;
                std::string plate_front_joint_name_;
                std::string plate_rear_joint_name_;
                std::string gpio_inputs_;
                std::string gpio_outputs_;
                
                double linear_scale_;
                double radial_scale_;
                double wheel_radius_;
                double wheel_distance_width_;
                double wheel_distance_length_;
                double wheel_separation_width_;
                double wheel_separation_length_;
                bool subscriber_is_active_;

                double linear_x_target_;
                double linear_y_target_;
                double angular_z_target_;
                double linear_x_smoothed_;
                double linear_y_smoothed_;
                double angular_z_smoothed_;

            };
        }
    }
}

#endif // __DEBICT_MECANUMBOT_CONTROLLER__MECANUMBOT_DRIVE_CONTROLLER_H__
