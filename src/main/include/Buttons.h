#define BUTTONS_H_
#include <frc/XboxController.h>
#include <frc/MathUtil.h>
#include "Constants.h"

namespace BUTTON{
 
   frc::XboxController stick {0};

    inline bool CONSTANTS::STATE::STOWED(){
        return stick.GetBButtonReleased();
    }

    inline bool CONSTANTS::STATE::INTAKE(){
        return stick.GetLeftBumperReleased();
    }
    
    inline bool CONSTANTS::STATE::EXTAKE(){
        return stick.GetRightBumperReleased();
    }
}
