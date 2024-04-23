
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
                    std::reference_wrapper<hardware_interface::LoanedCommandInterface> position_command_font,
                    std::reference_wrapper<hardware_interface::LoanedCommandInterface> position_command_rear
                    );

                void set_plate_height_decimal(double value);
                void set_plate_angle_radians(double value);
                void update(double dt);

                double get_plate_height_meters();
                double get_plate_angle_radians();

            private:
                std::reference_wrapper<const hardware_interface::LoanedStateInterface> position_state_front_;
                std::reference_wrapper<const hardware_interface::LoanedStateInterface> position_state_rear_;
                std::reference_wrapper<const hardware_interface::LoanedStateInterface> velocity_state_front_;
                std::reference_wrapper<const hardware_interface::LoanedStateInterface> velocity_state_rear_;
                std::reference_wrapper<hardware_interface::LoanedCommandInterface> position_command_font_;
                std::reference_wrapper<hardware_interface::LoanedCommandInterface> position_command_rear_;

                double plate_height_meters_target_{0.0};
                double plate_angle_radians_target_{0.0};

                const double MAX_PLATE_ANGLE_RADIANS{10.0 * M_PI / 180.0};
                const double MAX_ACTUATOR_EXTENSION{0.1};
                const double ACTUATOR_SEPARATION_METERS{0.424};
                const double MAX_PLATE_HEIGHT_METERS; // computed

                const double MAX_HEIGHT_M_PER_S{0.01};
                const double MAX_ANGLE_RAD_PER_S{1.0 * M_PI / 180.0};
            };
        }
    }
}

#endif // __DEBICT_MECANUMBOT_CONTROLLER__MECANUMBOT_PLATE_H__