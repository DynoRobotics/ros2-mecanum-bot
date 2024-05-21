
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

#define UNUSED(x) (void)(x)
#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  ((byte) & 0x80 ? '1' : '0'), \
  ((byte) & 0x40 ? '1' : '0'), \
  ((byte) & 0x20 ? '1' : '0'), \
  ((byte) & 0x10 ? '1' : '0'), \
  ((byte) & 0x08 ? '1' : '0'), \
  ((byte) & 0x04 ? '1' : '0'), \
  ((byte) & 0x02 ? '1' : '0'), \
  ((byte) & 0x01 ? '1' : '0') 

// Constructor
MecanumbotHardware::MecanumbotHardware()   
  : SystemInterface(),
    odErrorRegister(0x1001, 0x00),
    odErrorField(0x1003, 0x00),
    odErrorCode(0x603F, 0x00),
    odVelocityActualValue(0x6044, 0x00),
    odLiftActualPosition(0x6064, 0x00),
    odHomingSpeedSwitchSearch(0x6099, 0x01),
    odHomingSpeedZeroSearch(0x6099, 0x02),
    odHomingMethod(0x6098, 0x00),
    odHomingCurrentThreshold(0x203a, 0x01),
    odLiftMaxVelocity(0x607F, 0x00),
    odLiftMotorDriveSubmodeSelect(0x3202, 0x00),
    odLiftMotorRatedCurrent(0x203B, 0x01),
    odLiftMaximumDurationOfMaxCurrent(0x203B, 0x02),
    odMaxMotorCurrent(0x2031, 0x00),
    odFaultOptionCode(0x605E, 0x00),
    odDigitalInputRouting(0x3242, 0x04),
    odSpecialFunctionEnable(0x3240, 0x01),
    odRoutingEnable(0x3240, 0x08),
    odControlWord(0x6040, 0x00),
    odModesOfOperation(0x6060, 0x00),
    odStatusWord(0x6041, 0x00),
    odLiftTargetPosition(0x607A, 0x00),
    odTargetVelocity(0x60FF, 0x00),
    odMaxAcceleration(0x60C5, 0x00),
    odMaxDeceleration(0x60C6, 0x00),
    odAccelerationProfile(0x6083, 0x00),
    odDecelerationProfile(0x6084, 0x00)
{
    RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Mecanumbot hardware constructor");
}

bool is_lift_motor(hardware_interface::ComponentInfo & joint){
    return (joint.name == "plate_front_joint" || joint.name == "plate_rear_joint");

}

// function for converting meters to poses
// 3600 position per full rotation
// 3.978mm per full rotation
// hight = 0.003978 * positions / 3600
// -> position = hight / 0.003978 * 3600
int64_t lift_hight_to_positions(double hight){
    return hight / 0.003978 * 3600.0;
}

// 
double lift_positions_to_hight(int64_t positions){
    return 0.003978 * static_cast<double>(positions) / 3600.0;
}


hardware_interface::CallbackReturn MecanumbotHardware::on_init(const hardware_interface::HardwareInfo & hardware_info)
{
    RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "on_init");

    hardware_interface::CallbackReturn baseResult = hardware_interface::SystemInterface::on_init(hardware_info);
    if (baseResult != hardware_interface::CallbackReturn::SUCCESS) {
        return baseResult;
    }

    // info_.hardware_parameters are empty for me, seems to be bugged? enp5s0
    // network_interface_name_ = "enp86s0 (Ethernet interface)"; // "enp86s0 (Ethernet interface)"; // info_.hardware_parameters["network_interface_name"];
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
    hardware_gpio_in.resize(info_.gpios.size(), std::numeric_limits<double>::quiet_NaN());
    hardware_gpio_out.resize(info_.gpios.size(), std::numeric_limits<double>::quiet_NaN());
    connectedDeviceHandles.resize(info_.joints.size(), std::nullopt);

    for (hardware_interface::ComponentInfo & joint : info_.joints) {
        // print joint name
        RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Joint name: %s, is lift motor: %d", joint.name.c_str(), is_lift_motor(joint));
    }

    for (hardware_interface::ComponentInfo & joint : info_.joints){

        if (joint.parameters["motor_id"].empty()) {
            RCLCPP_FATAL(rclcpp::get_logger("MecanumbotHardware"), "Motor id not defined for join %s", joint.name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (joint.command_interfaces.size() != 1) {
            RCLCPP_FATAL(rclcpp::get_logger("MecanumbotHardware"), "Invalid number of command interfaces (expected: 1)");
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (!is_lift_motor(joint) && joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
            RCLCPP_FATAL(rclcpp::get_logger("MecanumbotHardware"), "Invalid joint %s command interface 0 type (expected: velocity)", joint.name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        } else if(is_lift_motor(joint) && joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION){
            RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), joint.name.c_str());
            RCLCPP_FATAL(rclcpp::get_logger("MecanumbotHardware"), "Invalid joint %s command interface 0 type (expected: position)", joint.name.c_str());
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

    // handling GPIO inputs
    // MecanumbotHardware has exactly two GPIO components
    if (info_.gpios.size() != 1)
    {
        RCLCPP_FATAL(
        rclcpp::get_logger("MecanumbotHardware"),
        "MecanumbotHardware has '%ld' GPIO components, '%d' expected.", info_.gpios.size(),
        1);
        return hardware_interface::CallbackReturn::ERROR;
    }
    // with exactly 1 command interface and 1 state interface
    for (int i = 0; i < int(info_.gpios.size()); i++){
        if (info_.gpios[i].command_interfaces.size() != 1){
        RCLCPP_FATAL(
            rclcpp::get_logger("MecanumbotHardware"),
            "GPIO component %s has '%ld' command interfaces, '%d' expected.",
            info_.gpios[i].name.c_str(), info_.gpios[i].command_interfaces.size(), 1);
        return hardware_interface::CallbackReturn::ERROR;
        }
        if (info_.gpios[i].state_interfaces.size() != 1){
        RCLCPP_FATAL(
            rclcpp::get_logger("MecanumbotHardware"),
            "GPIO component %s has '%ld' state interfaces, '%d' expected.",
            info_.gpios[i].name.c_str(), info_.gpios[i].state_interfaces.size(), 1);
        return hardware_interface::CallbackReturn::ERROR;
        }
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

    // GPIO state interface
    hardware_gpio_out.resize(info_.gpios.size());
    for (size_t i = 0; i < info_.gpios.size(); i++){
        for (auto state_if : info_.gpios.at(i).state_interfaces){
            RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Adding GPIO state interface: %s", info_.gpios[i].name.c_str());
            state_interfaces.emplace_back(
                hardware_interface::StateInterface(
                    info_.gpios.at(i).name, state_if.name, &hardware_gpio_out[i]
                )
            );
        }
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

    // GPIO command interface
    hardware_gpio_in.resize(info_.gpios.size());
    for (size_t i = 0; i < info_.gpios.size(); i++){
        for (auto command_if : info_.gpios.at(i).command_interfaces){
            RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Adding GPIO command interface: %s", info_.gpios[i].name.c_str());
            command_interfaces.emplace_back(
                hardware_interface::CommandInterface(
                    info_.gpios.at(i).name, command_if.name, &hardware_gpio_in[i]
                )
            );
        }
    }

    return command_interfaces;
}

hardware_interface::CallbackReturn MecanumbotHardware::on_configure(const rclcpp_lifecycle::State & _previous_state)
{
    RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Mecanumbot hardware on_configure ...");
    UNUSED(_previous_state);

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
    }

    for (size_t i = 0; i < info_.gpios.size(); i++){
        if (std::isnan(hardware_gpio_in[i])){
            hardware_gpio_in[i] = 0.0f;
        }
        if (std::isnan(hardware_gpio_out[i])){
            hardware_gpio_out[i] = 0.0f;
        }
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

        //TODO REMOVE
        // return hardware_interface::CallbackReturn::SUCCESS;

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
                        "Connecting joint name " << joint_name << " to nanotec device id " << deviceId.getDeviceId() << ", " << deviceId.getDescription());
                    nanolibHelper.connectDevice(deviceHandle);

                    // Store handle for later
                    connectedDeviceHandles.at(i) = deviceHandle;

                    // Set default values for lift motor
                    if(is_lift_motor(joint)){
                        // Max acceleration 0x60C5
                        int max_acceleration{5000}; // default 5000
                        nanolibHelper.writeInteger(deviceHandle, max_acceleration, odMaxAcceleration, MAX_ACCELERATION_BITS);
                        RCLCPP_INFO_ONCE(rclcpp::get_logger("MecanumbotHardware"), "Max acceleration set to %d", max_acceleration);
                        // Max deceleration 0x60C6
                        int max_deceleration{5000}; // default 5000
                        nanolibHelper.writeInteger(deviceHandle, max_deceleration, odMaxDeceleration, MAX_DECELERATION_BITS);
                        RCLCPP_INFO_ONCE(rclcpp::get_logger("MecanumbotHardware"), "Max deceleration set to %d", max_deceleration);
                        // Max Velocity?
                        int max_velocity{3000}; // default 30000
                        nanolibHelper.writeInteger(deviceHandle, max_velocity, odLiftMaxVelocity, LIFT_MAX_VELOCITY_BITS);
                        RCLCPP_INFO_ONCE(rclcpp::get_logger("MecanumbotHardware"), "Max velocity set to %d", max_velocity);
                        // Max motor current
                        int max_motor_current{6000}; // default 1000 // TODO: Increase this value
                        nanolibHelper.writeInteger(deviceHandle, max_motor_current, odMaxMotorCurrent, MAX_MOTOR_CURRENT_BITS);
                        RCLCPP_INFO_ONCE(rclcpp::get_logger("MecanumbotHardware"), "Max motor current set to %d", max_motor_current);

                        // read and log some motor current limits
                        int64_t motor_rated_current = nanolibHelper.readInteger(deviceHandle, odLiftMotorRatedCurrent);
                        RCLCPP_INFO_ONCE(rclcpp::get_logger("MecanumbotHardware"), "Lift motor rated current: %ld", motor_rated_current);
                        int64_t max_duration_of_max_current = nanolibHelper.readInteger(deviceHandle, odLiftMaximumDurationOfMaxCurrent);
                        RCLCPP_INFO_ONCE(rclcpp::get_logger("MecanumbotHardware"), "Lift motor max duration of max current: %ld", max_duration_of_max_current);

                        // HOMING RELATED CONFIG

                        // set homing current (mA) treshhold
                        nanolibHelper.writeInteger(deviceHandle, 200, odHomingCurrentThreshold, HOMING_CURRENT_THRESHOLD_BITS);

                        // set homing method
                        nanolibHelper.writeInteger(deviceHandle, -17, odHomingMethod, HOMING_METHOD_BITS);

                        // set homing speed
                        nanolibHelper.writeInteger(deviceHandle, 200, odHomingSpeedSwitchSearch, HOMING_SPEED_SWITCH_SEARCH_BITS);
                        nanolibHelper.writeInteger(deviceHandle, 100, odHomingSpeedZeroSearch, HOMING_SPEED_ZERO_SEARCH_BITS);

                        // Software position limit 0x607D ????
                        // Position Range Limit 0x607B ????
                    } else { // Default values for wheel motors
                        RCLCPP_INFO_ONCE(rclcpp::get_logger("MecanumbotHardware"), "Setting default values for wheel motor");
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

                        int max_motor_current{500}; // TODO: Increase this value
                        nanolibHelper.writeInteger(deviceHandle, max_motor_current, odMaxMotorCurrent, MAX_MOTOR_CURRENT_BITS);
                        RCLCPP_INFO_ONCE(rclcpp::get_logger("MecanumbotHardware"), "Max motor current set to %d", max_motor_current);
                    }

                    // Enable hardware emergency stop for both motor types
                    {
                        const int64_t asd = (int64_t)0b0; // All special functions are disabled during configuration
                        nanolibHelper.writeInteger(deviceHandle, asd, odSpecialFunctionEnable, SPECIAL_FUNCTION_ENABLE_BITS);
                        RCLCPP_INFO_ONCE(rclcpp::get_logger("MecanumbotHardware"), "Digital input control set to Interlock");
                    }

                    // What should happen when we get a fault
                    nanolibHelper.writeInteger(deviceHandle, 2, odFaultOptionCode, FAULT_OPTION_CODE_BITS);
                    RCLCPP_INFO_ONCE(rclcpp::get_logger("MecanumbotHardware"), "Emergency break mode set to Braking with quick stop ramp");

                    // Route physical input 1 to bit 3 of 60FDh (subindex 4)
                    {
                        const int64_t asd = (int64_t)0b0001;
                        nanolibHelper.writeInteger(deviceHandle, asd, odDigitalInputRouting, DIGITAL_INPUT_ROUTING_BITS);
                        RCLCPP_INFO_ONCE(rclcpp::get_logger("MecanumbotHardware"), "Digital input routing, digital input 3 set to hardware input 1");
                    }

                    // Enable digital input control
                    nanolibHelper.writeInteger(deviceHandle, 1, odRoutingEnable, ROUTING_ENABLE_BITS);
                    RCLCPP_INFO_ONCE(rclcpp::get_logger("MecanumbotHardware"), "Digital input control Enabled");

                    {
                        const int64_t asd = (int64_t)0b1000; // bit 3 is Interlock, we enable it after all configuration is done
                        nanolibHelper.writeInteger(deviceHandle, asd, odSpecialFunctionEnable, SPECIAL_FUNCTION_ENABLE_BITS);
                        RCLCPP_INFO_ONCE(rclcpp::get_logger("MecanumbotHardware"), "Digital input control set to Interlock");
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
    UNUSED(previous_state);

    for (auto deviceHandle : connectedDeviceHandles)
	{
		if (deviceHandle.has_value()) {
			RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Disconnecting nanotec device...");
			nanolibHelper.disconnectDevice(*deviceHandle);
		}
	}

    RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Disconnecting nanotec bus...");

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
    UNUSED(previous_state);

	int64_t CURRENT_STATUSWORD = 0;

	for (size_t i = 0; i < info_.joints.size(); i++)
	{
        auto deviceHandle = connectedDeviceHandles.at(i);

		if (!deviceHandle.has_value()) {
            RCLCPP_WARN(rclcpp::get_logger("MecanumbotHardware"), "Device handle is not set");
            continue;
        }

        // log device handle name
        RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Device handle name: %s", info_.joints.at(i).name.c_str());

        // Check the current status
        CURRENT_STATUSWORD = (nanolibHelper.readInteger(*deviceHandle, odStatusWord) & 0x6F);

        // Go it state "switched on disabled" if stuck in another state
        RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Current statusword before: 0x%02lX", CURRENT_STATUSWORD);
        if (CURRENT_STATUSWORD == 0b1000) {
            RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Faulty state detected, trying to recover");
            // If we are in a faulty state make an ricing edge on the control word bit 7
            // int64_t controlWord = nanolibHelper.readInteger(*deviceHandle, odControlWord);
            // RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Control word: 0x%02lX", controlWord);
            try{
                nanolibHelper.writeInteger(*deviceHandle, 0x80 | 0x0F, odControlWord, CONTROL_WORD_BITS);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Fault reset sent");
            } catch (const nanolib_exception &e) {
                RCLCPP_WARN(rclcpp::get_logger("MecanumbotHardware"), e.what());
            }
            CURRENT_STATUSWORD = (nanolibHelper.readInteger(*deviceHandle, odStatusWord) & 0x6F);
            RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Current statusword after: 0x%02lX", CURRENT_STATUSWORD);
        } else {
            RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "No faulty state detected");
            // if we are in another state
            nanolibHelper.writeInteger(*deviceHandle, 0x0, odControlWord, CONTROL_WORD_BITS);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
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

		while ((CURRENT_STATUSWORD = (nanolibHelper.readInteger(*deviceHandle, odStatusWord) & 0x6F)) != 0x21) { // 0110 1111 = 0x6F
			RCLCPP_WARN_STREAM(rclcpp::get_logger("MecanumbotHardware"), "Waiting to be ready to be switched on. Status: 0x" << std::hex << CURRENT_STATUSWORD << std::endl);
			std::this_thread::sleep_for(std::chrono::milliseconds(500));
		}

		nanolibHelper.writeInteger(*deviceHandle, CONTROL_WORD_SWITCHED_ON, odControlWord, CONTROL_WORD_BITS);
		std::this_thread::sleep_for(std::chrono::milliseconds(10));

		while ((CURRENT_STATUSWORD = nanolibHelper.readInteger(*deviceHandle, odStatusWord) & 0x6F) != 0x23) { // 0110 1111 = 0x6F
			RCLCPP_WARN_STREAM(rclcpp::get_logger("MecanumbotHardware"), "Waiting to be switched on. Status: 0x" << std::hex << CURRENT_STATUSWORD << std::endl);
			std::this_thread::sleep_for(std::chrono::milliseconds(500));
		}

        if(is_lift_motor(info_.joints.at(i))){
            nanolibHelper.writeInteger(*deviceHandle, CONTROL_WORD_OPERATION_ENABLED_LIFT_MOTOR, odControlWord, CONTROL_WORD_BITS);
        } else {
            nanolibHelper.writeInteger(*deviceHandle, CONTROL_WORD_OPERATION_ENABLED, odControlWord, CONTROL_WORD_BITS);
        }
		std::this_thread::sleep_for(std::chrono::milliseconds(10));

		while ((CURRENT_STATUSWORD = nanolibHelper.readInteger(*deviceHandle, odStatusWord) & 0x6F) != 0x27) { // 0110 1111 = 0x6F
			RCLCPP_WARN_STREAM(rclcpp::get_logger("MecanumbotHardware"), "Waiting to be ready for operation. Status: 0x" << std::hex << CURRENT_STATUSWORD << std::endl);
			std::this_thread::sleep_for(std::chrono::milliseconds(500));
		}
    }

    // TEST THE HOMING
    // perform_homing();

    RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Mecanumbot hardware on_activate done");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MecanumbotHardware::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
    RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Mecanumbot hardware stopping ...");
    UNUSED(previous_state);


	for (size_t i = 0; i < info_.joints.size(); i++)
	{
        auto deviceHandle = connectedDeviceHandles.at(i);

		if (!deviceHandle.has_value()) {
            RCLCPP_WARN(rclcpp::get_logger("MecanumbotHardware"), "Device handle is not set");
            continue;
        }

        if (!is_lift_motor(info_.joints.at(i))) {
            // Zero velocity
            nanolibHelper.writeInteger(*deviceHandle, 0, odTargetVelocity, TARGET_VELOCITY_BITS);
        }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

	for (size_t i = 0; i < info_.joints.size(); i++)
	{
        auto deviceHandle = connectedDeviceHandles.at(i);

		if (!deviceHandle.has_value()) {
            RCLCPP_WARN(rclcpp::get_logger("MecanumbotHardware"), "Device handle is not set");
            continue;
        }

        if (!is_lift_motor(info_.joints.at(i))) {   // Stop wheel motor
            nanolibHelper.writeInteger(*deviceHandle, CONTROL_WORD_STOP_MOTOR, odControlWord, CONTROL_WORD_BITS);
        } else {                                    // Stop lift motor
            nanolibHelper.writeInteger(*deviceHandle, CONTROL_WORD_STOP_MOTOR, odControlWord, CONTROL_WORD_BITS);
        }
    }

    RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Mecanumbot hardware stopped");
    return hardware_interface::CallbackReturn::SUCCESS;
}

void MecanumbotHardware::perform_homing() {

    std::vector<size_t> liftIndexes;

    // COLLECT LIFT INDEXES
    for (size_t i = 0; i < info_.joints.size(); i++)
    {
        auto deviceHandle = connectedDeviceHandles.at(i);

        if (!deviceHandle.has_value()) {
            continue;;
        }

        if (!is_lift_motor(info_.joints.at(i))) {
            continue;
        }

        RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Add to homing list lift motor: %s", info_.joints.at(i).name.c_str());

        liftIndexes.push_back(i);
    }

    // ENTER HOMING MODE
    for (auto i : liftIndexes) {
        auto deviceHandle = connectedDeviceHandles.at(i);
        // set homing mode in modes of operation
        nanolibHelper.writeInteger(*deviceHandle, MODE_HOMING, odModesOfOperation, MODES_OF_OPERATION_BITS);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    

    std::vector<size_t> notAtHomeIndexes = liftIndexes;

    // CHECK IF WE ARE ALREADY HOME
    for (auto i : liftIndexes) {
        auto deviceHandle = connectedDeviceHandles.at(i);

        int16_t status_word = nanolibHelper.readInteger(*deviceHandle, odStatusWord);

        RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"),
        " Status word for %s " BYTE_TO_BINARY_PATTERN " " BYTE_TO_BINARY_PATTERN,
        info_.joints.at(i).name.c_str(),
        BYTE_TO_BINARY(status_word>>8), BYTE_TO_BINARY(status_word));

        // Check if motor already home
        // check bit 13, 12 and 10 in status word to see if homing is complete
        if(((status_word >> 12) & 1) == 1) {

            RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Is already home, removing from list: %s", info_.joints.at(i).name.c_str());

            notAtHomeIndexes.erase(std::remove(notAtHomeIndexes.begin(), notAtHomeIndexes.end(), i), notAtHomeIndexes.end());

        }

    }

    // START HOMING    
    for (auto i : notAtHomeIndexes)
    {
        auto deviceHandle = connectedDeviceHandles.at(i);

        nanolibHelper.writeInteger(*deviceHandle, CONTROL_WORD_OPERATION_ENABLED_LIFT_MOTOR & ~(1<<4), odControlWord, CONTROL_WORD_BITS);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        nanolibHelper.writeInteger(*deviceHandle, CONTROL_WORD_OPERATION_ENABLED_LIFT_MOTOR | (1<<4), odControlWord, CONTROL_WORD_BITS);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

    }

    while (notAtHomeIndexes.size() > 0)
    {
        std::vector<size_t> reachedHome;

        for (auto i : notAtHomeIndexes)
        {
            auto deviceHandle = connectedDeviceHandles.at(i);

            int16_t status_word = nanolibHelper.readInteger(*deviceHandle, odStatusWord);

            RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"),
            " Status word for %s " BYTE_TO_BINARY_PATTERN " " BYTE_TO_BINARY_PATTERN,
            info_.joints.at(i).name.c_str(),
            BYTE_TO_BINARY(status_word>>8), BYTE_TO_BINARY(status_word));

            if(((status_word >> 12) & 1) == 1) {
                RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Homing complete: %s", info_.joints.at(i).name.c_str());

                reachedHome.push_back(i);
            }
        }

        for (auto i : reachedHome) {
            notAtHomeIndexes.erase(std::remove(notAtHomeIndexes.begin(), notAtHomeIndexes.end(), i), notAtHomeIndexes.end());
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    // SET MODE OF OPERATION TO POSITION PROFILE
    for (auto i : liftIndexes) {
        auto deviceHandle = connectedDeviceHandles.at(i);

        nanolibHelper.writeInteger(*deviceHandle, 0, odLiftTargetPosition, TARGET_POSITION_BITS);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        nanolibHelper.writeInteger(*deviceHandle, MODE_POSITION_PROFILE, odModesOfOperation, MODES_OF_OPERATION_BITS);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        nanolibHelper.writeInteger(*deviceHandle, CONTROL_WORD_OPERATION_ENABLED_LIFT_MOTOR & ~(1<<4), odControlWord, CONTROL_WORD_BITS);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        nanolibHelper.writeInteger(*deviceHandle, CONTROL_WORD_OPERATION_ENABLED_LIFT_MOTOR | (1<<4), odControlWord, CONTROL_WORD_BITS);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    hardware_gpio_out[0] = 1.0;

}

hardware_interface::return_type MecanumbotHardware::read(const rclcpp::Time & time, const rclcpp::Duration & period){   
    UNUSED(time);
    UNUSED(period);

    if (this->get_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    {
        return hardware_interface::return_type::OK;
    }

    for (size_t i = 0; i < info_.joints.size(); i++) {

        auto deviceHandle = connectedDeviceHandles.at(i);

        if (!deviceHandle.has_value()) {
            continue;
        }

        // read the error register
        int error{0};
        // int number_of_errors{0};
        // int error_last{0};

        // checking for errors
        try{
            error = nanolibHelper.readInteger(*deviceHandle, odErrorRegister);
        } catch (const nanolib_exception &e) {
            RCLCPP_ERROR(rclcpp::get_logger("MecanumbotHardware"), " %s: %s", info_.joints.at(i).name.c_str(), e.what());
        }

        if(error != 0){
            RCLCPP_WARN(rclcpp::get_logger("MecanumbotHardware"), "Motor: %s Error register: %d", info_.joints.at(i).name.c_str(), error);
            // error_last = nanolibHelper.readInteger(*deviceHandle, odErrorFieldSub1);
            // RCLCPP_WARN(rclcpp::get_logger("MecanumbotHardware"), "Last error: %d", error_last);
            // error_code = nanolibHelper.readInteger(*deviceHandle, odErrorCode);
            // RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Error code: %d", error_code);
        }

        // read feedback and put into state variables
        if(is_lift_motor(info_.joints.at(i))){          // lift motor interface states
            try{     // read position and set position state


                // int32_t target_position = nanolibHelper.readInteger(*deviceHandle, odLiftTargetPosition);
                // RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Target position: %d", target_position);


                // int32_t position = nanolibHelper.readInteger(*deviceHandle, odLiftActualPosition);
                // position_states_[i] = lift_positions_to_hight(position);
                // RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Actual position: %d", position);

                // int16_t status_word = nanolibHelper.readInteger(*deviceHandle, odStatusWord);
                // RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"),
                // " Status word for %s " BYTE_TO_BINARY_PATTERN " " BYTE_TO_BINARY_PATTERN,
                // info_.joints.at(i).name.c_str(),
                // BYTE_TO_BINARY(status_word>>8), BYTE_TO_BINARY(status_word));

            } catch (const nanolib_exception &e) {
                RCLCPP_ERROR(rclcpp::get_logger("MecanumbotHardware"), e.what());
            }
        } else{                                         // wheel motor interface states
            try{    // read the actual velocity
                int32_t actual_velocity_ticks = nanolibHelper.readInteger(*deviceHandle, odVelocityActualValue);
                double actual_velocity_radians_per_second = actual_velocity_ticks / 572.957795131;
                velocity_states_[i] = actual_velocity_radians_per_second;
            } catch (const nanolib_exception &e) {
                RCLCPP_ERROR(rclcpp::get_logger("MecanumbotHardware"), e.what());
            }
        }

    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type MecanumbotHardware::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
    UNUSED(time);
    UNUSED(period);

    if (this->get_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    {
        return hardware_interface::return_type::OK;
    }

    for (size_t i = 0; i < info_.joints.size(); i++) {
        auto deviceHandle = connectedDeviceHandles.at(i);
        // continue;

        if (!deviceHandle.has_value()) {
            // RCLCPP_WARN(rclcpp::get_logger("MecanumbotHardware"), "Device handle is not set");
            continue;
        }

        // If lift motor
        if (is_lift_motor(info_.joints.at(i)) && position_commands_[i] != position_commands_saved_[i]){
            // Set target position
            // RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Try write to lift motor");
            try{
                // RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "position command meters: %f", position_commands_[i]);

                int32_t target_position = lift_hight_to_positions(position_commands_[i]);

                // RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "position command ticks: %d", target_position);

                nanolibHelper.writeInteger(*deviceHandle, target_position, odLiftTargetPosition, TARGET_POSITION_BITS);

                nanolibHelper.writeInteger(*deviceHandle, CONTROL_WORD_OPERATION_ENABLED_LIFT_MOTOR & ~(1<<4), odControlWord, CONTROL_WORD_BITS);
                nanolibHelper.writeInteger(*deviceHandle, CONTROL_WORD_OPERATION_ENABLED_LIFT_MOTOR | (1<<4), odControlWord, CONTROL_WORD_BITS);
                
            } catch (const nanolib_exception &e) {
                RCLCPP_ERROR(rclcpp::get_logger("MecanumbotHardware"), e.what());
            }

            // Store the current position
            position_commands_saved_[i] = position_commands_[i];
        }
        else if (!is_lift_motor(info_.joints.at(i)) && velocity_commands_[i] != velocity_commands_saved_[i]) {    // Only send motor commands if the velocity changed
            // Set target velocity
            // NOTE: 10 * 180/pi = 572.957795131
            try{
                double v = velocity_commands_[i];
                std::string name = info_.joints.at(i).name;
                if (name.find("left") != std::string::npos) {
                    // hack to reverse direction of left motors
                    v = -v;
                }
                nanolibHelper.writeInteger(*deviceHandle, (int32_t)(v*572.957795131), odTargetVelocity, TARGET_VELOCITY_BITS);
            } catch (const nanolib_exception &e) {
                RCLCPP_ERROR(rclcpp::get_logger("MecanumbotHardware"), e.what());
            }

            // Store the current velocity
            velocity_commands_saved_[i] = velocity_commands_[i];
        }

    }


    // We can only home lift motors
    
    // RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Homing? %f %f", hardware_gpio_out[0], hardware_gpio_in[0]);

    if (hardware_gpio_out[0] != 1.0 && hardware_gpio_in[0] == 1.0)
    {
        RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Homing requested %f %f", hardware_gpio_out[0], hardware_gpio_in[0]);
        perform_homing();
    }

    //     // check if hardware gpio in changed
    //     if(hardware_gpio_out[0] != 1.0 && hardware_gpio_in[0] == 1.0){
    //         RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Homing requested %s %f %f", info_.joints.at(i).name.c_str(), hardware_gpio_out[0], hardware_gpio_in[0]);
    //         try{
    //             RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Performing homing");
    //             // perform_homing(i);
    //             RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Homing done");
    //         } catch (const nanolib_exception &e) {
    //             RCLCPP_ERROR(rclcpp::get_logger("MecanumbotHardware"), e.what());
    //         }
    //     }
    // }

    return hardware_interface::return_type::OK;
}