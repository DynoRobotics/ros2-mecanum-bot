
#ifndef __DEBICT_MECANUMBOT_CONTROLLER__MECANUMBOT_PLATE_H__
#define __DEBICT_MECANUMBOT_CONTROLLER__MECANUMBOT_PLATE_H__

#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <math.h>

namespace debict
{
    namespace mecanumbot
    {
        namespace controller
        {

            class MecanumbotPlate
            {
            public:
                MecanumbotPlate(
                    std::reference_wrapper<const hardware_interface::LoanedStateInterface> position_state_front,
                    std::reference_wrapper<const hardware_interface::LoanedStateInterface> position_state_rear,
                    std::reference_wrapper<const hardware_interface::LoanedStateInterface> velocity_state_front,
                    std::reference_wrapper<const hardware_interface::LoanedStateInterface> velocity_state_rear,
                    std::reference_wrapper<const hardware_interface::LoanedStateInterface> hardware_gpio_out,
                    std::reference_wrapper<hardware_interface::LoanedCommandInterface> position_command_front,
                    std::reference_wrapper<hardware_interface::LoanedCommandInterface> position_command_rear,
                    std::reference_wrapper<hardware_interface::LoanedCommandInterface> hardware_gpio_in
                    );

                void set_plate_height_decimal(double value);
                void set_plate_angle_radians(double value);
                void set_homing(bool value);
                void update(double dt);

                double get_plate_height_meters();
                double get_plate_angle_radians();

            private:
                std::reference_wrapper<const hardware_interface::LoanedStateInterface> position_state_front_;
                std::reference_wrapper<const hardware_interface::LoanedStateInterface> position_state_rear_;
                std::reference_wrapper<const hardware_interface::LoanedStateInterface> velocity_state_front_;
                std::reference_wrapper<const hardware_interface::LoanedStateInterface> velocity_state_rear_;
                std::reference_wrapper<const hardware_interface::LoanedStateInterface> hardware_gpio_out_;
                std::reference_wrapper<hardware_interface::LoanedCommandInterface> position_command_front_;
                std::reference_wrapper<hardware_interface::LoanedCommandInterface> position_command_rear_;
                std::reference_wrapper<hardware_interface::LoanedCommandInterface> hardware_gpio_in_;

                bool perform_homing_{false};

                double plate_height_meters_target_{0.0};
                double plate_angle_radians_target_{0.0};

                double plate_height_meters_target_smoothed_{0.0};
                double plate_angle_radians_target_smoothed_{0.0};

                const double MAX_PLATE_ANGLE_RADIANS{5.0 * M_PI / 180.0};
                const double MAX_ACTUATOR_EXTENSION{0.068}; // 0.068 for old mechanism, 0.090 for new mechanism
                const double ACTUATOR_SEPARATION_METERS{0.424};
                const double MAX_PLATE_HEIGHT_METERS; // computed

                const double MAX_HEIGHT_M_PER_S{0.03}; // 0.06
                const double MAX_ANGLE_RAD_PER_S{4.0 * M_PI / 180.0}; // 4.0
            };
        }
    }
}

#endif // __DEBICT_MECANUMBOT_CONTROLLER__MECANUMBOT_PLATE_H__