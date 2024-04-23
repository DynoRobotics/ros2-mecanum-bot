
#include "mecanumbot_controller/mecanumbot_plate.hpp"

using namespace debict::mecanumbot::controller;

MecanumbotPlate::MecanumbotPlate(
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> position_state_front,
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> position_state_rear,
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> velocity_state_front,
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> velocity_state_rear,
    std::reference_wrapper<hardware_interface::LoanedCommandInterface> position_command_font,
    std::reference_wrapper<hardware_interface::LoanedCommandInterface> position_command_rear
    )
    : position_state_front_(position_state_front)
    , position_state_rear_(position_state_rear)
    , velocity_state_front_(velocity_state_front)
    , velocity_state_rear_(velocity_state_rear)
    , position_command_font_(position_command_font)
    , position_command_rear_(position_command_rear)
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

double MecanumbotPlate::get_plate_height_meters()
{
    double font_height_meters = position_state_front_.get().get_value();
    double rear_height_meters = position_state_rear_.get().get_value();

    return (font_height_meters + rear_height_meters) / 2.0;
}

double MecanumbotPlate::get_plate_angle_radians()
{
    double font_height_meters = position_state_front_.get().get_value();
    double rear_height_meters = position_state_rear_.get().get_value();

    return atan2(rear_height_meters - font_height_meters, ACTUATOR_SEPARATION_METERS);
}

void MecanumbotPlate::update(double dt)
{
    double actual_height = this->get_plate_height_meters();
    double actual_angle = this->get_plate_angle_radians();

    double safe_plate_angle_radians_target_ = 0.0;
    if (fabs(actual_height - MAX_PLATE_HEIGHT_METERS) < 0.01)
    {
        safe_plate_angle_radians_target_ = plate_angle_radians_target_;
    }

    double error_height = plate_height_meters_target_ - actual_height;
    double error_angle = safe_plate_angle_radians_target_ - actual_angle;

    double height_velocity = std::max(-MAX_HEIGHT_M_PER_S, std::min(MAX_HEIGHT_M_PER_S, error_height / dt));

    double angle_velocity = std::max(-MAX_ANGLE_RAD_PER_S, std::min(MAX_ANGLE_RAD_PER_S, error_angle / dt));

    double font_velocity = height_velocity - angle_velocity * ACTUATOR_SEPARATION_METERS / 2.0;
    double rear_velocity = height_velocity + angle_velocity * ACTUATOR_SEPARATION_METERS / 2.0;

    // ---

    double font_height_meters = position_state_front_.get().get_value();
    double rear_height_meters = position_state_rear_.get().get_value();

    double front_height_meters_setpoint = std::max(0.0, std::min(MAX_PLATE_HEIGHT_METERS, font_height_meters + font_velocity * dt));
    double rear_height_meters_setpoint = std::max(0.0, std::min(MAX_PLATE_HEIGHT_METERS, rear_height_meters + rear_velocity * dt));

    position_command_font_.get().set_value(front_height_meters_setpoint);
    position_command_rear_.get().set_value(rear_height_meters_setpoint);
}