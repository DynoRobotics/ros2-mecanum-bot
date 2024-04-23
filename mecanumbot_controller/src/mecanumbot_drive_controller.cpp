
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include "mecanumbot_controller/mecanumbot_drive_controller.hpp"

PLUGINLIB_EXPORT_CLASS(
    debict::mecanumbot::controller::MecanumbotDriveController,
    controller_interface::ControllerInterface
)

using namespace debict::mecanumbot::controller;

MecanumbotDriveController::MecanumbotDriveController()
    : controller_interface::ControllerInterface()
    , velocity_command_subsciption_(nullptr)
    , velocity_command_ptr_(nullptr)
    , plate_height_command_subsciption_(nullptr)
    , plate_height_command_ptr_(nullptr)
    , plate_angle_command_subsciption_(nullptr)
    , plate_angle_command_ptr_(nullptr)
{
    RCLCPP_INFO(rclcpp::get_logger("MecanumbotDriveController"), "Construct MecanumbotDriveController");
}

controller_interface::InterfaceConfiguration MecanumbotDriveController::command_interface_configuration() const
{
    RCLCPP_INFO(rclcpp::get_logger("MecanumbotDriveController"), "Command interface configuration MecanumbotDriveController");

    controller_interface::InterfaceConfiguration command_interfaces_config;
    command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    command_interfaces_config.names.push_back(fl_wheel_joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY);
    command_interfaces_config.names.push_back(fr_wheel_joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY);
    command_interfaces_config.names.push_back(rl_wheel_joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY);
    command_interfaces_config.names.push_back(rr_wheel_joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY);

    command_interfaces_config.names.push_back(plate_front_joint_name_+ "/" + hardware_interface::HW_IF_POSITION);
    command_interfaces_config.names.push_back(plate_rear_joint_name_ + "/" + hardware_interface::HW_IF_POSITION);

    return command_interfaces_config;
}

controller_interface::InterfaceConfiguration MecanumbotDriveController::state_interface_configuration() const
{
    RCLCPP_INFO(rclcpp::get_logger("MecanumbotDriveController"), "State interface configuration for MecanumbotDriveController");

    controller_interface::InterfaceConfiguration state_interfaces_config;
    state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    state_interfaces_config.names.push_back(fl_wheel_joint_name_ + "/" + hardware_interface::HW_IF_POSITION);
    state_interfaces_config.names.push_back(fl_wheel_joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY);
    state_interfaces_config.names.push_back(fr_wheel_joint_name_ + "/" + hardware_interface::HW_IF_POSITION);
    state_interfaces_config.names.push_back(fr_wheel_joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY);
    state_interfaces_config.names.push_back(rl_wheel_joint_name_ + "/" + hardware_interface::HW_IF_POSITION);
    state_interfaces_config.names.push_back(rl_wheel_joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY);
    state_interfaces_config.names.push_back(rr_wheel_joint_name_ + "/" + hardware_interface::HW_IF_POSITION);
    state_interfaces_config.names.push_back(rr_wheel_joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY);

    state_interfaces_config.names.push_back(plate_front_joint_name_ + "/" + hardware_interface::HW_IF_POSITION);
    state_interfaces_config.names.push_back(plate_front_joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY);
    state_interfaces_config.names.push_back(plate_rear_joint_name_ + "/" + hardware_interface::HW_IF_POSITION);
    state_interfaces_config.names.push_back(plate_rear_joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY);

    return state_interfaces_config;
}

controller_interface::CallbackReturn MecanumbotDriveController::on_init()
{
    RCLCPP_INFO(rclcpp::get_logger("MecanumbotDriveController"), "On init for MecanumbotDriveController");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type MecanumbotDriveController::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
    // RCLCPP_INFO(rclcpp::get_logger("MecanumbotDriveController"), "Update MecanumbotDriveController");

    // Get the last velocity command
    auto velocity_command = velocity_command_ptr_.readFromRT();
    if (velocity_command && *velocity_command) {     
        // Calculate the wheel velocity
        // See: http://robotsforroboticists.com/drive-kinematics/

        auto linear = (*velocity_command)->linear;
        auto angular = (*velocity_command)->angular;

        double fl_wheel_velocity = (1 / wheel_radius_) * (linear.x - linear.y - (wheel_separation_width_ + wheel_separation_length_) * angular.z);
        double fr_wheel_velocity = (1 / wheel_radius_) * (linear.x + linear.y + (wheel_separation_width_ + wheel_separation_length_) * angular.z);
        double rl_wheel_velocity = (1 / wheel_radius_) * (linear.x + linear.y - (wheel_separation_width_ + wheel_separation_length_) * angular.z);
        double rr_wheel_velocity = (1 / wheel_radius_) * (linear.x - linear.y + (wheel_separation_width_ + wheel_separation_length_) * angular.z);

        fl_wheel_->set_velocity(fl_wheel_velocity);
        fr_wheel_->set_velocity(fr_wheel_velocity);
        rl_wheel_->set_velocity(rl_wheel_velocity);
        rr_wheel_->set_velocity(rr_wheel_velocity);
    }

    // TODO: Add the lift motor position here
    auto plate_height_command = plate_height_command_ptr_.readFromRT();
    if (plate_height_command && *plate_height_command) {
        RCLCPP_INFO(rclcpp::get_logger("MecanumbotDriveController"), "Plate height in update: %f", (*plate_height_command)->data);
        plate_->set_plate_height_decimal((*plate_height_command)->data);
    }

    auto plate_angle_command = plate_angle_command_ptr_.readFromRT();
    if (plate_angle_command && *plate_angle_command) {
        plate_->set_plate_angle_radians((*plate_angle_command)->data);
    }

    plate_->update(period.seconds());

    return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn MecanumbotDriveController::on_configure(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(rclcpp::get_logger("MecanumbotDriveController"), "Configure MecanumbotDriveController");

    fl_wheel_joint_name_ = get_node()->get_parameter("fl_wheel_joint_name").as_string();
    fr_wheel_joint_name_ = get_node()->get_parameter("fr_wheel_joint_name").as_string();
    rl_wheel_joint_name_ = get_node()->get_parameter("rl_wheel_joint_name").as_string();
    rr_wheel_joint_name_ = get_node()->get_parameter("rr_wheel_joint_name").as_string();

    if (fl_wheel_joint_name_.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("MecanumbotDriveController"), "'fl_wheel_joint_name' parameter was empty");
        return controller_interface::CallbackReturn::ERROR;
    }
    if (fr_wheel_joint_name_.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("MecanumbotDriveController"), "'fr_wheel_joint_name' parameter was empty");
        return controller_interface::CallbackReturn::ERROR;
    }
    if (rl_wheel_joint_name_.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("MecanumbotDriveController"), "'rl_wheel_joint_name' parameter was empty");
        return controller_interface::CallbackReturn::ERROR;
    }
    if (rr_wheel_joint_name_.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("MecanumbotDriveController"), "'rr_wheel_joint_name' parameter was empty");
        return controller_interface::CallbackReturn::ERROR;
    }

    plate_front_joint_name_ = get_node()->get_parameter("plate_front_joint_name").as_string();
    plate_rear_joint_name_ = get_node()->get_parameter("plate_rear_joint_name").as_string();

    if (plate_front_joint_name_.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("MecanumbotDriveController"), "'plate_front_joint_name' parameter was empty");
        return controller_interface::CallbackReturn::ERROR;
    }

    if (plate_rear_joint_name_.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("MecanumbotDriveController"), "'plate_rear_joint_name' parameter was empty");
        return controller_interface::CallbackReturn::ERROR;
    }

    wheel_radius_ = get_node()->get_parameter("wheel_radius").as_double();
    wheel_distance_width_ = get_node()->get_parameter("wheel_distance.width").as_double();
    wheel_distance_length_ = get_node()->get_parameter("wheel_distance.length").as_double();
    if (wheel_radius_ <= 0.0) {
        RCLCPP_ERROR(rclcpp::get_logger("MecanumbotDriveController"), "'wheel_radius' parameter cannot be zero or less");
        return controller_interface::CallbackReturn::ERROR;
    }
    if (wheel_distance_width_ <= 0.0) {
        RCLCPP_ERROR(rclcpp::get_logger("MecanumbotDriveController"), "'wheel_distance.width' parameter cannot be zero or less");
        return controller_interface::CallbackReturn::ERROR;
    }
    if (wheel_distance_length_ <= 0.0) {
        RCLCPP_ERROR(rclcpp::get_logger("MecanumbotDriveController"), "'wheel_distance.length' parameter cannot be zero or less");
        return controller_interface::CallbackReturn::ERROR;
    }
    wheel_separation_width_ = wheel_distance_width_ / 2;
    wheel_separation_length_ = wheel_distance_length_ / 2;

    if (!reset()) {
        RCLCPP_ERROR(rclcpp::get_logger("MecanumbotDriveController"), "failed reset");
        return controller_interface::CallbackReturn::ERROR;
    }

    velocity_command_subsciption_ = get_node()->create_subscription<Twist>("/mecanum_controller/cmd_vel_unstamped", rclcpp::SystemDefaultsQoS(), [this](const Twist::SharedPtr twist)
    {
        velocity_command_ptr_.writeFromNonRT(twist);
    });

    // TODO READ UP/DOWN TOPIC FOR LIFT MOTOR
    plate_height_command_subsciption_ = get_node()->create_subscription<Float64>("/plate_lift_controller/trigger_plate", rclcpp::SystemDefaultsQoS(), [this](const Float64::SharedPtr msg)
    {
        RCLCPP_INFO(rclcpp::get_logger("MecanumbotDriveController"), "Plate height command message callback: %f", msg->data);
        plate_height_command_ptr_.writeFromNonRT(msg);
    });

    // TODO READ TILT MOTOR
    plate_angle_command_subsciption_ = get_node()->create_subscription<Float64>("/plate_tilt_controller/tilt_angle", rclcpp::SystemDefaultsQoS(), [this](const Float64::SharedPtr msg)
    {
        plate_angle_command_ptr_.writeFromNonRT(msg);
    });

    RCLCPP_INFO(rclcpp::get_logger("MecanumbotDriveController"), "Configure done for MecanumbotDriveController");

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MecanumbotDriveController::on_activate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(rclcpp::get_logger("MecanumbotDriveController"), "On activate for MecanumbotDriveController");

    // Initialize the wheels
    fl_wheel_ = get_wheel(fl_wheel_joint_name_);
    fr_wheel_ = get_wheel(fr_wheel_joint_name_);
    rl_wheel_ = get_wheel(rl_wheel_joint_name_);
    rr_wheel_ = get_wheel(rr_wheel_joint_name_);
    plate_ = get_plate(plate_front_joint_name_, plate_rear_joint_name_);

    if (!fl_wheel_ || !fr_wheel_ || !rl_wheel_ || !rr_wheel_/* || !plate_*/) {
        return controller_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("MecanumbotDriveController"), "On activate done for MecanumbotDriveController");
    
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MecanumbotDriveController::on_deactivate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(rclcpp::get_logger("MecanumbotDriveController"), "On deactivate for MecanumbotDriveController");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MecanumbotDriveController::on_cleanup(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(rclcpp::get_logger("MecanumbotDriveController"), "On cleanup for MecanumbotDriveController");
    if (!reset()) {
        return controller_interface::CallbackReturn::ERROR;
    }

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MecanumbotDriveController::on_error(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(rclcpp::get_logger("MecanumbotDriveController"), "On error for MecanumbotDriveController");
    if (!reset()) {
        return controller_interface::CallbackReturn::ERROR;
    }
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MecanumbotDriveController::on_shutdown(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(rclcpp::get_logger("MecanumbotDriveController"), "On shutdown for MecanumbotDriveController");
    return controller_interface::CallbackReturn::SUCCESS;
}

std::shared_ptr<MecanumbotWheel> MecanumbotDriveController::get_wheel(const std::string & wheel_joint_name)
{
    RCLCPP_INFO(rclcpp::get_logger("MecanumbotDriveController"), "Get Wheel MecanumbotDriveController");

    // Lookup the position state interface
    const auto position_state = std::find_if(state_interfaces_.cbegin(), state_interfaces_.cend(), [&wheel_joint_name](const hardware_interface::LoanedStateInterface & interface)
    {
        return interface.get_name() == wheel_joint_name + "/position"
            && interface.get_interface_name() == hardware_interface::HW_IF_POSITION;
    });

    if (position_state == state_interfaces_.cend()) {
        RCLCPP_ERROR(rclcpp::get_logger("MecanumbotDriveController"), "%s position state interface not found", wheel_joint_name.c_str());
        return nullptr;
    }

    // Lookup the velocity state interface
    const auto velocity_state = std::find_if(state_interfaces_.cbegin(), state_interfaces_.cend(), [&wheel_joint_name](const hardware_interface::LoanedStateInterface & interface)
    {
        return interface.get_name() == wheel_joint_name + "/velocity"
            && interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;
    });
    if (velocity_state == state_interfaces_.cend()) {
        RCLCPP_ERROR(rclcpp::get_logger("MecanumbotDriveController"), "%s velocity state interface not found", wheel_joint_name.c_str());
        return nullptr;
    }

    // Lookup the velocity command interface
    const auto velocity_command = std::find_if(command_interfaces_.begin(), command_interfaces_.end(), [&wheel_joint_name](const hardware_interface::LoanedCommandInterface & interface)
    {
        return interface.get_name() == wheel_joint_name + "/velocity"
            && interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;
    });
    if (velocity_command == command_interfaces_.end()) {
        RCLCPP_ERROR(rclcpp::get_logger("MecanumbotDriveController"), "%s velocity command interface not found", wheel_joint_name.c_str());
        return nullptr;
    }

    RCLCPP_INFO(rclcpp::get_logger("MecanumbotDriveController"), "Get Wheel done MecanumbotDriveController");

    // Create the wheel instance
    return std::make_shared<MecanumbotWheel>(
        std::ref(*position_state),
        std::ref(*velocity_state),
        std::ref(*velocity_command)
        );
}

std::shared_ptr<MecanumbotPlate> MecanumbotDriveController::get_plate(const std::string & plate_front_joint_name, const std::string & plate_rear_joint_name)
{
    RCLCPP_INFO(rclcpp::get_logger("MecanumbotDriveController"), "Get plate MecanumbotDriveController");

    // Lookup the position state interface
    const auto plate_front_position_state = std::find_if(state_interfaces_.cbegin(), state_interfaces_.cend(), [&plate_front_joint_name](const hardware_interface::LoanedStateInterface & interface)
    {
        return interface.get_name() == plate_front_joint_name + "/position"
            && interface.get_interface_name() == hardware_interface::HW_IF_POSITION;
    });

    if (plate_front_position_state == state_interfaces_.cend()) {
        RCLCPP_ERROR(rclcpp::get_logger("MecanumbotDriveController"), "%s position state interface not found", plate_front_joint_name.c_str());
        return nullptr;
    }

    // Lookup the velocity state interface
    const auto plate_front_velocity_state = std::find_if(state_interfaces_.cbegin(), state_interfaces_.cend(), [&plate_front_joint_name](const hardware_interface::LoanedStateInterface & interface)
    {
        return interface.get_name() == plate_front_joint_name + "/velocity"
            && interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;
    });
    if (plate_front_velocity_state == state_interfaces_.cend()) {
        RCLCPP_ERROR(rclcpp::get_logger("MecanumbotDriveController"), "%s velocity state interface not found", plate_front_joint_name.c_str());
        return nullptr;
    }

    // Lookup the position command interface
    const auto plate_front_position_command = std::find_if(command_interfaces_.begin(), command_interfaces_.end(), [&plate_front_joint_name](const hardware_interface::LoanedCommandInterface & interface)
    {
        return interface.get_name() == plate_front_joint_name + "/position"
            && interface.get_interface_name() == hardware_interface::HW_IF_POSITION;
    });
    if (plate_front_position_command == command_interfaces_.end()) {
        RCLCPP_ERROR(rclcpp::get_logger("MecanumbotDriveController"), "%s position command interface not found", plate_front_joint_name.c_str());
        return nullptr;
    }

    // ---------------------------------------

    // Lookup the position state interface
    const auto plate_rear_position_state = std::find_if(state_interfaces_.cbegin(), state_interfaces_.cend(), [&plate_rear_joint_name](const hardware_interface::LoanedStateInterface & interface)
    {
        return interface.get_name() == plate_rear_joint_name + "/position"
            && interface.get_interface_name() == hardware_interface::HW_IF_POSITION;
    });

    if (plate_rear_position_state == state_interfaces_.cend()) {
        RCLCPP_ERROR(rclcpp::get_logger("MecanumbotDriveController"), "%s position state interface not found", plate_rear_joint_name.c_str());
        return nullptr;
    }

    // Lookup the velocity state interface
    const auto plate_rear_velocity_state = std::find_if(state_interfaces_.cbegin(), state_interfaces_.cend(), [&plate_rear_joint_name](const hardware_interface::LoanedStateInterface & interface)
    {
        return interface.get_name() == plate_rear_joint_name + "/velocity"
            && interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;
    });
    if (plate_rear_velocity_state == state_interfaces_.cend()) {
        RCLCPP_ERROR(rclcpp::get_logger("MecanumbotDriveController"), "%s velocity state interface not found", plate_rear_joint_name.c_str());
        return nullptr;
    }

    // Lookup the position command interface
    const auto plate_rear_position_command = std::find_if(command_interfaces_.begin(), command_interfaces_.end(), [&plate_rear_joint_name](const hardware_interface::LoanedCommandInterface & interface)
    {
        return interface.get_name() == plate_rear_joint_name + "/position"
            && interface.get_interface_name() == hardware_interface::HW_IF_POSITION;
    });
    if (plate_rear_position_command == command_interfaces_.end()) {
        RCLCPP_ERROR(rclcpp::get_logger("MecanumbotDriveController"), "%s position command interface not found", plate_rear_joint_name.c_str());
        return nullptr;
    }

    RCLCPP_INFO(rclcpp::get_logger("MecanumbotDriveController"), "Get plate done MecanumbotDriveController");

    return std::make_shared<MecanumbotPlate>(
        std::ref(*plate_front_position_state),
        std::ref(*plate_rear_position_state),
        std::ref(*plate_front_velocity_state),
        std::ref(*plate_rear_velocity_state),
        std::ref(*plate_front_position_command),
        std::ref(*plate_rear_position_command)
        );
}


bool MecanumbotDriveController::reset()
{
    RCLCPP_INFO(rclcpp::get_logger("MecanumbotDriveController"), "Reset MecanumbotDriveController");

    subscriber_is_active_ = false;
    velocity_command_subsciption_.reset();
    plate_height_command_subsciption_.reset();
    plate_angle_command_subsciption_.reset();

    fl_wheel_.reset();
    fr_wheel_.reset();
    rl_wheel_.reset();
    rr_wheel_.reset();

    plate_.reset();

    RCLCPP_INFO(rclcpp::get_logger("MecanumbotDriveController"), "Reset done MecanumbotDriveController");

    return true;
}