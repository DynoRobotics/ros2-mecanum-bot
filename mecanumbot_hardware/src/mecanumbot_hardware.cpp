
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <vector>
#include <string>

#include "mecanumbot_hardware/mecanumbot_hardware.hpp"

// https://control.ros.org/master/doc/ros2_control/hardware_interface/doc/writing_new_hardware_component.html

PLUGINLIB_EXPORT_CLASS(
    debict::mecanumbot::hardware::MecanumbotHardware,
    hardware_interface::SystemInterface
)

using namespace debict::mecanumbot::hardware;

// Constructor
MecanumbotHardware::MecanumbotHardware()   
  : SystemInterface(),
    odControlWord(0x6040, 0x00),
    odModesOfOperation(0x6060, 0x00),
    odStatusWord(0x6041, 0x00),
    odTargetPosition(0x607A, 0x00),
    odTargetVelocity(0x60FF, 0x00),
    odVelocityActualValue(0x6044, 0x00),
    odErrorRegister(0x1001, 0x00),
    odErrorField(0x1003, 0x00),
    odErrorCode(0x603F, 0x00),
    odMaxAcceleration(0x60C5, 0x00),
    odMaxDeceleration(0x60C6, 0x00),
    odAccelerationProfile(0x6083, 0x00),
    odDecelerationProfile(0x6084, 0x00),
    odTmp(0x6061, 0x00)
{
    RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Mecanumbot hardware constructor");
}

bool is_lift_motor(hardware_interface::ComponentInfo & joint ){
    return (joint.name == "plate_lift_joint" || joint.name == "plate_tilt_joint");

}

hardware_interface::CallbackReturn MecanumbotHardware::on_init(const hardware_interface::HardwareInfo & hardware_info)
{
    RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "on_init");

    hardware_interface::CallbackReturn baseResult = hardware_interface::SystemInterface::on_init(hardware_info);
    if (baseResult != hardware_interface::CallbackReturn::SUCCESS) {
        return baseResult;
    }

    // info_.hardware_parameters are empty for me, seems to be bugged? enp5s0
    network_interface_name_ = "eno1 (Ethernet interface)"; // "enp86s0 (Ethernet interface)"; // info_.hardware_parameters["network_interface_name"];
    network_interface_protocol_ = "RESTful API"; // info_.hardware_parameters["network_interface_protocol"];

    RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Network interface name: '%s'", network_interface_name_.c_str());
    RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Network interface protocol: '%s'", network_interface_protocol_.c_str());

    motor_ids_.resize(info_.joints.size());
    position_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    velocity_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    position_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    position_commands_saved_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    velocity_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    velocity_commands_saved_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    connectedDeviceHandles.resize(info_.joints.size(), std::nullopt);

    for (hardware_interface::ComponentInfo & joint : info_.joints){   
        // print joint name
        RCLCPP_DEBUG(rclcpp::get_logger("MecanumbotHardware"), "Joint name: %s, is lift motor: %d", joint.name.c_str(), is_lift_motor(joint));

        if (joint.parameters["motor_id"].empty()) {
            RCLCPP_FATAL(rclcpp::get_logger("MecanumbotHardware"), "Motor id not defined for join %s", joint.name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (joint.command_interfaces.size() != 1) {
            RCLCPP_FATAL(rclcpp::get_logger("MecanumbotHardware"), "Invalid number of command interfaces (expected: 1)");
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (!is_lift_motor(joint) && joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
            RCLCPP_FATAL(rclcpp::get_logger("MecanumbotHardware"), "Invalid joint command interface 0 type (expected: velocity)");
            return hardware_interface::CallbackReturn::ERROR;
        } else if(is_lift_motor(joint) && joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION){
            RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), joint.name.c_str());
            RCLCPP_FATAL(rclcpp::get_logger("MecanumbotHardware"), "Invalid joint command interface 0 type (expected: position)");
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (joint.state_interfaces.size() != 2) {
            RCLCPP_FATAL(rclcpp::get_logger("MecanumbotHardware"), "Invalid number of state interfaces (expected: 2)");
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(rclcpp::get_logger("MecanumbotHardware"), "Invalid joint state interface 0 type (expected: position)");
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
            RCLCPP_FATAL(rclcpp::get_logger("MecanumbotHardware"), "Invalid joint state interface 1 type (expected: velocity)");
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    for (size_t i = 0; i < info_.joints.size(); i++) {
        motor_ids_[i] = std::stoul(info_.joints[i].parameters["motor_id"]);
        RCLCPP_INFO_STREAM(rclcpp::get_logger("MecanumbotHardware"), info_.joints[i].name.c_str() << " mapped to motor " << motor_ids_[i]);
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MecanumbotHardware::export_state_interfaces()
{
    RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "export_state_interfaces");

    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < info_.joints.size(); i++) {
        RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Adding position state interface: %s", info_.joints[i].name.c_str());
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]
            )
        );
    }
    for (size_t i = 0; i < info_.joints.size(); i++) {
        RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Adding velocity state interface: %s", info_.joints[i].name.c_str());
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]
            )
        );
    }
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MecanumbotHardware::export_command_interfaces()
{
    RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "export_command_interfaces");

    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < info_.joints.size(); i++) {
        if(is_lift_motor(info_.joints[i])){
            RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Adding position command interface: %s", info_.joints[i].name.c_str());
            command_interfaces.emplace_back(
                hardware_interface::CommandInterface(
                    info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_commands_[i]
                )
            );
        } else{
        RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Adding velocity command interface: %s", info_.joints[i].name.c_str());
        command_interfaces.emplace_back(
            hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]
            )
        );
        }
    }
    return command_interfaces;
}

hardware_interface::CallbackReturn MecanumbotHardware::on_configure(const rclcpp_lifecycle::State & _previous_state)
{
    RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Mecanumbot hardware on_configure ...");

    for (size_t i = 0; i < info_.joints.size(); i++) {
        if (std::isnan(position_states_[i])) {
            position_states_[i] = 0.0f;
        }
        if (std::isnan(velocity_states_[i])) {
            velocity_states_[i] = 0.0f;
        }
        if (std::isnan(position_commands_[i])) {
            position_commands_[i] = 0.0f;
        }
        if (std::isnan(velocity_commands_[i])) {
            velocity_commands_[i] = 0.0f;
        }
        position_commands_saved_[i] = position_commands_[i];
        velocity_commands_saved_[i] = velocity_commands_[i];
    }

	unsigned lineNum = 0;

	try {

		// its possible to set the logging level to a different level
		nanolibHelper.setLoggingLevel(nlc::LogLevel::Off);

        // list all hardware available
        std::vector<nlc::BusHardwareId> busHardwareIds = nanolibHelper.getBusHardware();

        if (busHardwareIds.empty()) {
		    RCLCPP_ERROR(rclcpp::get_logger("MecanumbotHardware"), "No hardware buses found.");
			return hardware_interface::CallbackReturn::FAILURE;
		}

        auto busHwIdIt = std::find_if(busHardwareIds.begin(), busHardwareIds.end(),
			[this](const nlc::BusHardwareId& id) {
				return id.getName() == network_interface_name_ && id.getProtocol() == network_interface_protocol_;
			});

        if (busHwIdIt == busHardwareIds.end()) {
			RCLCPP_ERROR_STREAM(rclcpp::get_logger("MecanumbotHardware"),
                std::endl << std::endl
                << "Could not find bus hardware with name '" 
                << network_interface_name_ 
                << "', and protocol '"
                << network_interface_protocol_
                << "'. Available hardware buses and protocols:");

			lineNum = 0;
			// print out available hardware
			for (const nlc::BusHardwareId &busHwId : busHardwareIds) {
			    RCLCPP_ERROR_STREAM(rclcpp::get_logger("MecanumbotHardware"),
				    lineNum << ". name '" << busHwId.getName() << "', protocol: '" << busHwId.getProtocol() << "'");
				lineNum++;
			}

			return hardware_interface::CallbackReturn::FAILURE;
		}

        nlc::BusHardwareId busHwId = *busHwIdIt;

		// create bus hardware options for opening the hardware
		nlc::BusHardwareOptions busHwOptions = nanolibHelper.createBusHardwareOptions(busHwId);

        RCLCPP_INFO_STREAM(rclcpp::get_logger("MecanumbotHardware"),
		    "Opening bus hardware: " << busHwId.getName() << ", " << busHwId.getProtocol() << std::endl);

		// now able to open the hardware itself
		nanolibHelper.openBusHardware(busHwId, busHwOptions);
		openedBusHardware = busHwId;

		// Scan the bus for available devices
		RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Scanning bus for devices...");

		std::vector<nlc::DeviceId> deviceIds = nanolibHelper.scanBus(busHwId);

		if (deviceIds.empty()) {
			RCLCPP_ERROR_STREAM(rclcpp::get_logger("MecanumbotHardware"), std::endl << std::endl << "No devices found." << std::endl);
			return hardware_interface::CallbackReturn::FAILURE;
		}

        RCLCPP_INFO_STREAM(rclcpp::get_logger("MecanumbotHardware"), "----------- Available devices: -----------");

		lineNum = 0;
		// print out available devices
		for (const nlc::DeviceId &deviceId : deviceIds) {
			RCLCPP_INFO_STREAM(rclcpp::get_logger("MecanumbotHardware"), 
                lineNum << ". " << deviceId.getDescription()
                << " [device id: " << deviceId.getDeviceId()
                << ", hardware: " << deviceId.getBusHardwareId().getName() << "]");
			lineNum++;
		}
        RCLCPP_INFO_STREAM(rclcpp::get_logger("MecanumbotHardware"), "----------------------------------------- \n");

        for (const nlc::DeviceId &deviceId : deviceIds) {
            
            for (size_t i = 0; i < motor_ids_.size(); i++) {
                auto joint_motor_id = motor_ids_.at(i);
                auto joint_name = info_.joints.at(i).name;
                auto joint = info_.joints.at(i);

                if (joint_motor_id == deviceId.getDeviceId()) {
                    
                    // Register the device id
                    nlc::DeviceHandle deviceHandle = nanolibHelper.addDevice(deviceId);

                    // Establishing a connection with the device
                    RCLCPP_INFO_STREAM(rclcpp::get_logger("MecanumbotHardware"),
                        "Connecting joint name " << joint_name << " to nanotec device " << deviceId.getDescription());
                    nanolibHelper.connectDevice(deviceHandle);

                    // Store handle for later
                    connectedDeviceHandles.at(i) = deviceHandle;

                    // Set default values for lift motor
                    if(is_lift_motor(joint)){
                        // TODO: Set default values for lift motors
                    } else { // Default values for wheel motors
                        RCLCPP_INFO_ONCE(rclcpp::get_logger("MecanumbotHardware"), "Setting default values for motor");
                        int max_acceleration{1000000};
                        nanolibHelper.writeInteger(deviceHandle, max_acceleration, odMaxAcceleration, MAX_ACCELERATION_BITS);
                        RCLCPP_INFO_ONCE(rclcpp::get_logger("MecanumbotHardware"), "Max acceleration set to %d", max_acceleration);
                        int max_deceleration{30000};
                        nanolibHelper.writeInteger(deviceHandle, max_deceleration, odMaxDeceleration, MAX_DECELERATION_BITS);
                        RCLCPP_INFO_ONCE(rclcpp::get_logger("MecanumbotHardware"), "Max deceleration set to %d", max_deceleration);

                        int acceleration_profile{300000};
                        nanolibHelper.writeInteger(deviceHandle, acceleration_profile, odAccelerationProfile, ACCELERATION_PROFILE_BITS);
                        RCLCPP_INFO_ONCE(rclcpp::get_logger("MecanumbotHardware"), "Max acceleration profile set to %d", acceleration_profile);
                        int deceleration_profile{300000};
                        nanolibHelper.writeInteger(deviceHandle, deceleration_profile, odDecelerationProfile, DECELERATION_PROFILE_BITS);
                        RCLCPP_INFO_ONCE(rclcpp::get_logger("MecanumbotHardware"), "Max deceleration profile set to %d", deceleration_profile);
                    }
                }
            }
		}

	} catch (const nanolib_exception &e) {
		RCLCPP_ERROR(rclcpp::get_logger("MecanumbotHardware"), e.what());
        return hardware_interface::CallbackReturn::FAILURE;
	}

    RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Mecanumbot hardware on_configure done");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MecanumbotHardware::on_cleanup(const rclcpp_lifecycle::State & previous_state)
{
    RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Mecanumbot hardware on_cleanup ...");

    for (auto deviceHandle : connectedDeviceHandles)
	{
		if (deviceHandle.has_value()) {
			RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Disconnecting nanotec device...");
			nanolibHelper.disconnectDevice(*deviceHandle);
		}
	}

	if (openedBusHardware.has_value()) {
        RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Disconnecting nanotec bus...");
		nanolibHelper.closeBusHardware(*openedBusHardware);
	}

    RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Mecanumbot hardware on_cleanup done");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MecanumbotHardware::on_activate(const rclcpp_lifecycle::State & previous_state)
{
    RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Mecanumbot hardware on_activate ...");

	int64_t CURRENT_STATUSWORD = 0;
    int64_t i{0};
	for (auto deviceHandle : connectedDeviceHandles)
	{
		if (!deviceHandle.has_value()) {
            RCLCPP_WARN(rclcpp::get_logger("MecanumbotHardware"), "Device handle is not set");
            i++;
            continue;
        }

		// Set velocity profile
        if(is_lift_motor(info_.joints.at(i))){
            // Set mode of operation (position mode)
            RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Lift motor - set position mode");
            nanolibHelper.writeInteger(*deviceHandle, MODE_POSITION_PROFILE, odModesOfOperation, MODES_OF_OPERATION_BITS);
        } else {
            RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Wheel motor - set velocity mode");
            // Set mode of operation (velocity mode)
            nanolibHelper.writeInteger(*deviceHandle, MODE_VELOCITY_PROFILE, odModesOfOperation, MODES_OF_OPERATION_BITS);
            // Set target velocity
		    nanolibHelper.writeInteger(*deviceHandle, 0, odTargetVelocity, TARGET_VELOCITY_BITS);
        }

		// Step through state machine startup sequence
		nanolibHelper.writeInteger(*deviceHandle, CONTROL_WORD_READY_TO_SWITCH_ON, odControlWord, CONTROL_WORD_BITS);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

		while ((CURRENT_STATUSWORD = (nanolibHelper.readInteger(*deviceHandle, odStatusWord) & 0xEF)) != 0x21) {
			RCLCPP_WARN_STREAM(rclcpp::get_logger("MecanumbotHardware"), std::endl << CURRENT_STATUSWORD << std::endl);
			std::this_thread::sleep_for(std::chrono::milliseconds(500));
		}

		nanolibHelper.writeInteger(*deviceHandle, CONTROL_WORD_SWITCHED_ON, odControlWord, CONTROL_WORD_BITS);
		std::this_thread::sleep_for(std::chrono::milliseconds(10));

		while ((CURRENT_STATUSWORD = nanolibHelper.readInteger(*deviceHandle, odStatusWord) & 0xEF) != 0x23) {
			RCLCPP_WARN_STREAM(rclcpp::get_logger("MecanumbotHardware"), std::endl << CURRENT_STATUSWORD << std::endl);
			std::this_thread::sleep_for(std::chrono::milliseconds(500));
		}

        if(is_lift_motor(info_.joints.at(i))){
            nanolibHelper.writeInteger(*deviceHandle, CONTROL_WORD_OPERATION_ENABLED_LIFT_MOTOR, odControlWord, CONTROL_WORD_BITS);
        } else {
            nanolibHelper.writeInteger(*deviceHandle, CONTROL_WORD_OPERATION_ENABLED, odControlWord, CONTROL_WORD_BITS);
        }
		std::this_thread::sleep_for(std::chrono::milliseconds(10));

		while ((CURRENT_STATUSWORD = nanolibHelper.readInteger(*deviceHandle, odStatusWord) & 0xEF) != 0x27) {
			RCLCPP_WARN_STREAM(rclcpp::get_logger("MecanumbotHardware"), std::endl << CURRENT_STATUSWORD << std::endl);
			std::this_thread::sleep_for(std::chrono::milliseconds(500));
		}
        i++;
    }

    RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Mecanumbot hardware on_activate done");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MecanumbotHardware::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
    RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Mecanumbot hardware stopping ...");


	for (auto deviceHandle : connectedDeviceHandles)
	{
		if (!deviceHandle.has_value()) {
            RCLCPP_WARN(rclcpp::get_logger("MecanumbotHardware"), "Device handle is not set");
            continue;
        }

		// Zero velocity
		nanolibHelper.writeInteger(*deviceHandle, 0, odTargetVelocity, TARGET_VELOCITY_BITS);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

	for (auto deviceHandle : connectedDeviceHandles)
	{
		if (!deviceHandle.has_value()) {
            RCLCPP_WARN(rclcpp::get_logger("MecanumbotHardware"), "Device handle is not set");
            continue;
        }

		// Stop state machine
		nanolibHelper.writeInteger(*deviceHandle, CONTROL_WORD_STOP_MOTOR, odControlWord, CONTROL_WORD_BITS);
    }

    RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Mecanumbot hardware stopped");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MecanumbotHardware::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{   

    for (size_t i = 0; i < info_.joints.size(); i++) {

        auto deviceHandle = connectedDeviceHandles.at(i);

        if (!deviceHandle.has_value()) {
            continue;
        }

        // read the error register
        int error{0};
        // int number_of_errors{0};
        // int error_last{0};
        int actual_velocity{0};

        // checking for errors
        try{
            error = nanolibHelper.readInteger(*deviceHandle, odErrorRegister);
        } catch (const nanolib_exception &e) {
            RCLCPP_ERROR(rclcpp::get_logger("MecanumbotHardware"), e.what());
        }

        if(error != 0){
            RCLCPP_WARN(rclcpp::get_logger("MecanumbotHardware"), "Error register: %d", error);
            // error_last = nanolibHelper.readInteger(*deviceHandle, odErrorFieldSub1);
            // RCLCPP_WARN(rclcpp::get_logger("MecanumbotHardware"), "Last error: %d", error_last);
            // error_code = nanolibHelper.readInteger(*deviceHandle, odErrorCode);
            // RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Error code: %d", error_code);
        }

        // read the actual velocity
        // try{
        //     actual_velocity = nanolibHelper.readInteger(*deviceHandle, odVelocityActualValue);
        // } catch (const nanolib_exception &e) {
        //     RCLCPP_ERROR(rclcpp::get_logger("MecanumbotHardware"), e.what());
        // }
        // RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Actual velocity: %d", actual_velocity);

        // read the tmp value
        int tmp{0};
        try{
            tmp = nanolibHelper.readInteger(*deviceHandle, odStatusWord);
        } catch (const nanolib_exception &e) {
            RCLCPP_ERROR(rclcpp::get_logger("MecanumbotHardware"), e.what());
        }
        tmp = tmp & 0xEF;
        RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Control word: 0x%x", tmp);

    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type MecanumbotHardware::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{

    for (size_t i = 0; i < info_.joints.size(); i++) {
        // If lift motor
        if (is_lift_motor(info_.joints.at(i))){
            auto deviceHandle = connectedDeviceHandles.at(i);

            if (!deviceHandle.has_value()) {
                // RCLCPP_WARN(rclcpp::get_logger("MecanumbotHardware"), "Device handle is not set");
                continue;
            }
            // Set target position
            RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Try write to lift motor");
            try{
                nanolibHelper.writeInteger(*deviceHandle, 3600*5, odTargetPosition, TARGET_POSITION_BITS);
                RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Target position set to 100");
            } catch (const nanolib_exception &e) {
                RCLCPP_ERROR(rclcpp::get_logger("MecanumbotHardware"), e.what());
            }

            // Set the trigger bit in control word
            try{
                nanolibHelper.writeInteger(*deviceHandle, CONTROL_WORD_RUN_LIFT_MOTOR, odControlWord, CONTROL_WORD_BITS);
                RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Control word set to 0x3F to make motor spin");
            } catch (const nanolib_exception &e) {
                RCLCPP_ERROR(rclcpp::get_logger("MecanumbotHardware"), e.what());
            }

            continue;
        }

        continue;

        // Only send motor commands if the velocity changed
        if (velocity_commands_[i] != velocity_commands_saved_[i]) {
            // RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Motor velocity changed: %.5f", velocity_commands_[i]);

            auto deviceHandle = connectedDeviceHandles.at(i);

            if (!deviceHandle.has_value()) {
                // RCLCPP_WARN(rclcpp::get_logger("MecanumbotHardware"), "Device handle is not set");
                continue;
            }

            // Set target velocity
            // NOTE: 10 * 180/pi = 572.957795131
            // Resulting in 0-1ms in 16 increments
            try{
                nanolibHelper.writeInteger(*deviceHandle, (int64_t)(velocity_commands_[i]*572.957795131), odTargetVelocity, TARGET_VELOCITY_BITS);
            } catch (const nanolib_exception &e) {
                RCLCPP_ERROR(rclcpp::get_logger("MecanumbotHardware"), e.what());
            }

            // Store the current velocity
            velocity_commands_saved_[i] = velocity_commands_[i];
        }
    }

    return hardware_interface::return_type::OK;
}