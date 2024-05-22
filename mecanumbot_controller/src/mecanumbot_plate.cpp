
#include "mecanumbot_controller/mecanumbot_plate.hpp"
#include <rclcpp/rclcpp.hpp>

using namespace debict::mecanumbot::controller;

MecanumbotPlate::MecanumbotPlate(
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> position_state_front,
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> position_state_rear,
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> velocity_state_front,
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> velocity_state_rear,
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> hardware_gpio_out,
    std::reference_wrapper<hardware_interface::LoanedCommandInterface> position_command_front,
    std::reference_wrapper<hardware_interface::LoanedCommandInterface> position_command_rear,
    std::reference_wrapper<hardware_interface::LoanedCommandInterface> hardware_gpio_in
    )
    : position_state_front_(position_state_front)
    , position_state_rear_(position_state_rear)
    , velocity_state_front_(velocity_state_front)
    , velocity_state_rear_(velocity_state_rear)
    , hardware_gpio_out_(hardware_gpio_out)
    , position_command_front_(position_command_front)
    , position_command_rear_(position_command_rear)
    , hardware_gpio_in_(hardware_gpio_in)
    , MAX_PLATE_HEIGHT_METERS(MAX_ACTUATOR_EXTENSION - ACTUATOR_SEPARATION_METERS * tan(MAX_PLATE_ANGLE_RADIANS) / 2.0)
{

}

void MecanumbotPlate::set_plate_height_decimal(double value)
{
    plate_height_meters_target_ = MAX_PLATE_HEIGHT_METERS * std::max(0.0, std::min(1.0, value));
}

void MecanumbotPlate::set_plate_angle_radians(double value)
{
    plate_angle_radians_target_ = std::max(-MAX_PLATE_ANGLE_RADIANS, std::min(MAX_PLATE_ANGLE_RADIANS, value));
}

void MecanumbotPlate::set_homing(bool value){
    perform_homing_ = value;
}


double MecanumbotPlate::get_plate_height_meters()
{
    double front_height_meters = position_state_front_.get().get_value();
    double rear_height_meters = position_state_rear_.get().get_value();

    return (front_height_meters + rear_height_meters) / 2.0;
}

double MecanumbotPlate::get_plate_angle_radians()
{
    double front_height_meters = position_state_front_.get().get_value();
    double rear_height_meters = position_state_rear_.get().get_value();

    return atan2(rear_height_meters - front_height_meters, ACTUATOR_SEPARATION_METERS);
}

void MecanumbotPlate::update(double dt)
{
    double is_home = hardware_gpio_out_.get().get_value();
    // RCLCPP_INFO(rclcpp::get_logger("MecanumbotDriveController"), "Is home value: %f", is_home);
    if (is_home == 0.0)
    {
        if (perform_homing_)
        {
            hardware_gpio_in_.get().set_value(1.0);
        }

        return;
    } else {
        hardware_gpio_in_.get().set_value(0.0);
        perform_homing_ = false;
    }

    // -------------------------------------------
    // double front_height_meters = position_state_front_.get().get_value();
    // double rear_height_meters = position_state_rear_.get().get_value();

    // RCLCPP_INFO(rclcpp::get_logger("MecanumbotPlate::update"), "front_height_meters: %f", front_height_meters);
    // RCLCPP_INFO(rclcpp::get_logger("MecanumbotPlate::update"), "rear_height_meters: %f", rear_height_meters);

    // -------------------------------------------

    double actual_height = this->get_plate_height_meters();
    double actual_angle = this->get_plate_angle_radians();

    // RCLCPP_INFO(rclcpp::get_logger("MecanumbotPlate::update"), "actual_height: %f", actual_height);
    // RCLCPP_INFO(rclcpp::get_logger("MecanumbotPlate::update"), "actual_angle: %f", actual_angle);

    double safe_plate_angle_radians_target_ = 0.0;
    
    // RCLCPP_INFO(rclcpp::get_logger("MecanumbotPlate::update"), "diff: %f", fabs(actual_height - MAX_PLATE_HEIGHT_METERS));
    
    if (fabs(actual_height - MAX_PLATE_HEIGHT_METERS) < 0.01)
    {
        safe_plate_angle_radians_target_ = plate_angle_radians_target_;
    }

    // RCLCPP_INFO(rclcpp::get_logger("MecanumbotPlate::update"), "safe_plate_angle_radians_target_: %f", safe_plate_angle_radians_target_);

    // -------------------------------------------

    double error_height = plate_height_meters_target_ - plate_height_meters_target_smoothed_;
    double error_angle = safe_plate_angle_radians_target_ - plate_angle_radians_target_smoothed_;

    // RCLCPP_INFO(rclcpp::get_logger("MecanumbotPlate::update"), "error_height: %f", error_height);
    // RCLCPP_INFO(rclcpp::get_logger("MecanumbotPlate::update"), "error_angle: %f", error_angle);


    double height_velocity = std::max(-MAX_HEIGHT_M_PER_S, std::min(MAX_HEIGHT_M_PER_S, error_height / dt));
    double angle_velocity = std::max(-MAX_ANGLE_RAD_PER_S, std::min(MAX_ANGLE_RAD_PER_S, error_angle / dt));

    // RCLCPP_INFO(rclcpp::get_logger("MecanumbotPlate::update"), "height_velocity: %f", height_velocity);
    // RCLCPP_INFO(rclcpp::get_logger("MecanumbotPlate::update"), "angle_velocity: %f", angle_velocity);

    plate_height_meters_target_smoothed_ += height_velocity * dt;
    plate_angle_radians_target_smoothed_ += angle_velocity * dt;

    // RCLCPP_INFO(rclcpp::get_logger("MecanumbotPlate::update"), "plate_height_meters_target_smoothed_: %f", plate_height_meters_target_smoothed_);
    // RCLCPP_INFO(rclcpp::get_logger("MecanumbotPlate::update"), "plate_angle_radians_target_smoothed_: %f", plate_angle_radians_target_smoothed_);

    // ---
    double height_angle_offset = ACTUATOR_SEPARATION_METERS / 2.0 * tan(plate_angle_radians_target_smoothed_);

    double front_height_meters_setpoint = std::max(0.004, std::min(MAX_ACTUATOR_EXTENSION,
        plate_height_meters_target_smoothed_ + height_angle_offset
    ));
    double rear_height_meters_setpoint = std::max(0.004, std::min(MAX_ACTUATOR_EXTENSION,
        plate_height_meters_target_smoothed_ - height_angle_offset
    ));

    // RCLCPP_INFO(rclcpp::get_logger("MecanumbotPlate::update"), "MAX_ACTUATOR_EXTENSION: %f", MAX_ACTUATOR_EXTENSION);
    
    // RCLCPP_INFO(rclcpp::get_logger("MecanumbotPlate::update"), "front_height_meters_setpoint: %f", front_height_meters_setpoint);
    // RCLCPP_INFO(rclcpp::get_logger("MecanumbotPlate::update"), "rear_height_meters_setpoint: %f", rear_height_meters_setpoint);

    position_command_front_.get().set_value(front_height_meters_setpoint);
    position_command_rear_.get().set_value(rear_height_meters_setpoint);
}