#define BUTTONS_H_
#include <frc/XboxController.h>
#include <frc/MathUtil.h>
#include "Constants.h"
#include "Robot.h"

namespace BUTTON{
 
   frc::XboxController stick {0};
   namespace DRIVETRAIN
   {
    inline double LX(){
        return stick.GetLeftX();
    }    
    inline double LY(){
        return stick.GetLeftY();
    }
    inline double RX(){
        return stick.GetRightX();
    }
    inline double RY(){
        return stick.GetRightY();
    }
   }
   

    inline bool STOWED(){
        return stick.GetBButtonReleased();
    }

    inline bool INTAKE(){
        return stick.GetLeftBumperReleased();
    }
    
    inline bool EXTAKE_LOW(){
        return stick.GetRightBumperReleased();
    }

    inline bool MAN_INTAKE(){
        return stick.GetXButtonReleased();
    }
    inline bool EXTAKE_MID(){
        return stick.GetAButtonReleased();
    }
    inline bool EXTAKE_HIGH(){
        return stick.GetYButtonReleased();
    }
}
