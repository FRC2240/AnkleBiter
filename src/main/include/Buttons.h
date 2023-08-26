#define BUTTONS_H_
#include <frc/XboxController.h>
#include <frc/MathUtil.h>
#include "Constants.h"
#include "Robot.h"

namespace BUTTON{
 
   frc::XboxController stick {0};

    inline bool STOWED(){
        return stick.GetBButtonReleased();
    }

    inline bool INTAKE(){
        return stick.GetLeftBumperReleased();
    }
    
    inline bool EXTAKE(){
        return stick.GetRightBumperReleased();
    }

    inline bool MAN_INTAKE(){
        return stick.GetXButtonReleased();
    }
}
