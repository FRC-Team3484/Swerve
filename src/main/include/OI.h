#ifndef OI_H
#define OI_H

#include "Constants.h"

#include <frc/MathUtil.h>
#include <frc/XboxController.h>

class OI {
    public:
        double GetLeftY();
        double GetLeftX();
        double GetRightX();
        double GetA();
        bool GetYPressed();
        bool GetBPressed();
        bool GetXPressed();
    
    private:
    frc::XboxController _driver_controller{ControllerConstants::DRIVER_CONTROLLER_PORT};
 };

#endif