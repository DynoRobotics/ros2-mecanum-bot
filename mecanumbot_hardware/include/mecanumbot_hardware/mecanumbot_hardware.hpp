
#ifndef __DEBICT_MECANUMBOT_HARDWARE__MECANUMBOT_HARDWARE_H__
#define __DEBICT_MECANUMBOT_HARDWARE__MECANUMBOT_HARDWARE_H__

//#include <hardware_interface/base_interface.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/visibility_control.h>
#include <rclcpp/macros.hpp>
#include <vector>
#include <optional>

#include "mecanumbot_hardware/mecanumbot_serial_port.hpp"
#include "mecanumbot_hardware/mecanumbot_hardware_compiler.h"
#include "mecanumbot_hardware/nanolib_helper.hpp"

namespace debict
{
    namespace mecanumbot
    {
        namespace hardware
        {
            
            class MecanumbotHardware
                : public hardware_interface::SystemInterface
            {
            public:
                RCLCPP_SHARED_PTR_DEFINITIONS(MecanumbotHardware)

                MecanumbotHardware();

                DEBICT_MECANUMBOT_HARDWARE_PUBLIC
                virtual hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & hardware_info) override;

                //on_configure;
                //on_cleanup;
                //on_activate;
                //on_deactivate;
                //on_shutdown;
                //on_error;
                
                DEBICT_MECANUMBOT_HARDWARE_PUBLIC
                virtual std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
                
                DEBICT_MECANUMBOT_HARDWARE_PUBLIC
                virtual std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
                
                DEBICT_MECANUMBOT_HARDWARE_PUBLIC
                virtual hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

                DEBICT_MECANUMBOT_HARDWARE_PUBLIC
                virtual hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

                DEBICT_MECANUMBOT_HARDWARE_PUBLIC
                virtual hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
                
                DEBICT_MECANUMBOT_HARDWARE_PUBLIC
                virtual hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
                
                DEBICT_MECANUMBOT_HARDWARE_PUBLIC
                virtual hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
                
                DEBICT_MECANUMBOT_HARDWARE_PUBLIC
                virtual hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

            private:
                std::vector<size_t> motor_ids_;
                std::vector<double> position_states_;
                std::vector<double> velocity_states_;
                std::vector<double> velocity_commands_;
                std::vector<double> velocity_commands_saved_;
                std::vector<double> position_commands_; // stored in meter
                std::vector<double> position_commands_saved_;

                std::string network_interface_name_;
                std::string network_interface_protocol_;

                NanoLibHelper nanolibHelper;
                std::optional<nlc::BusHardwareId> openedBusHardware;
                std::vector<std::optional<nlc::DeviceHandle>> connectedDeviceHandles;

                const int64_t CONTROL_WORD_READY_TO_SWITCH_ON = 0x6;
                const int64_t CONTROL_WORD_SWITCHED_ON = 0x7;
                const int64_t CONTROL_WORD_OPERATION_ENABLED = 0xF;
                const int64_t CONTROL_WORD_STOP_MOTOR = 0x0;

                const int64_t CONTROL_WORD_OPERATION_ENABLED_LIFT_MOTOR = 0x2F;
                const int64_t CONTROL_WORD_RUN_LIFT_MOTOR = 0x3F;

                const int64_t MODE_POSITION_PROFILE = 0x1;
                const int64_t MODE_VELOCITY_PROFILE = 0x3;

                const int64_t LIFT_MOTOR_TOP_POSITION = 3600;
                const int64_t LIFT_MOTOR_BOTTOM_POSITION = 0;

                // read registers

                const nlc::OdIndex odErrorRegister;
                const unsigned int ERROR_REGISTER_BITS = 8;

                const nlc::OdIndex odErrorField;
                const unsigned int ERROR_FIELD_BITS = 8;

                const nlc::OdIndex odErrorCode;
                const unsigned int ERROR_CODE_BITS = 16;

                const nlc::OdIndex odVelocityActualValue;
                const unsigned int VELOCITY_ACTUAL_VALUE_BITS = 32;

                const nlc::OdIndex odLiftActualPosition;
                const unsigned int LIFT_POSITION_ACTUAL_VALUE_BITS = 32;


                // write registers

                const nlc::OdIndex odControlWord;
                const unsigned int CONTROL_WORD_BITS = 16;

                const nlc::OdIndex odModesOfOperation;
                const unsigned int MODES_OF_OPERATION_BITS = 8;

                const nlc::OdIndex odStatusWord;
                const unsigned int STATUS_WORD_BITS = 16;

                const nlc::OdIndex odLiftTargetPosition;
                const unsigned int TARGET_POSITION_BITS = 32;

                const nlc::OdIndex odTargetVelocity;
                const unsigned int TARGET_VELOCITY_BITS = 32;

                const nlc::OdIndex odMaxAcceleration;
                const unsigned int MAX_ACCELERATION_BITS = 32;

                const nlc::OdIndex odMaxDeceleration;
                const unsigned int MAX_DECELERATION_BITS = 32;

                const nlc::OdIndex odAccelerationProfile;
                const unsigned int ACCELERATION_PROFILE_BITS = 32;

                const nlc::OdIndex odDecelerationProfile;
                const unsigned int DECELERATION_PROFILE_BITS = 32;

                const nlc::OdIndex odTmp;
                const unsigned int TMP_BITS = 32;

            };
        }
    }
}

#endif // __DEBICT_MECANUMBOT_HARDWARE__MECANUMBOT_HARDWARE_H__