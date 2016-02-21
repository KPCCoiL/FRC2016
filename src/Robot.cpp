#include "Robot.hpp"
#include "WPILib.h"

Robot::Robot() :
    stick(0),
    lw(LiveWindow::GetInstance()),
    leftMotor(0),
    rightMotor(1),
    arm(2),
    func_id(0)
{
    leftMotor.SetInverted(true);
}

void Robot::AutonomousInit() {
    startTime = std::chrono::system_clock::now();
}

void Robot::AutonomousPeriodic() {
    /*
       if(autoLoopCounter < 100) //Check if we've completed 100 loops (approximately 2 seconds)
       {
       using namespace std::literals;
       if (startTime.time_since_epoch() < 2000ms) {
       myRobot.Drive(-0.5, 0.0); 	// drive forwards half speed
       autoLoopCounter++;
       }
       else {
       myRobot.Drive(0.0, 0.0); 	// stop robot
       }
       */
}

void Robot::TeleopInit() {

}

void Robot::TeleopPeriodic() {
    //move robot itself
    constexpr double LoweredPower = 0.3;
    double leftPower = stick.GetRawButton(ButtonL) ? LoweredPower : 1.0,
           rightPower = stick.GetRawButton(ButtonR) ? LoweredPower : 1.0,
           leftSpeed = stick.GetRawAxis(1) * leftPower, //left joystick, y axis
           rightSpeed = stick.GetRawAxis(5) * rightPower;    //right Joystick, y axis
    leftMotor.Set(funcs.at(func_id).first(leftSpeed));
    rightMotor.Set(funcs.at(func_id).first(rightSpeed));
    
    //control of arm
    bool btnAPressed = stick.GetRawButton(ButtonA),
         btnBPressed = stick.GetRawButton(ButtonB);
    if (btnAPressed ^ btnBPressed)
        arm.Set(btnAPressed ? 0.5 : -0.5);
    else arm.Set(0.0);

    static bool yPressed = false;
    bool currentY = stick.GetRawButton(ButtonY);
    if (!yPressed && currentY) {
        func_id = (func_id + 1) % funcs.size();
        SmartDashboard::PutString("Function", funcs.at(func_id).second);
    }
    yPressed = currentY;
}

void Robot::TestPeriodic() {
    lw->Run();
}

START_ROBOT_CLASS(Robot)
