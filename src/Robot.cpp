#include "Robot.hpp"
#include "WPILib.h"

Robot::Robot() :
    stick(0),
    lw(LiveWindow::GetInstance()),
    leftMotor(0),
    rightMotor(1)
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
    SmartDashboard::PutNumber("number", 0.01);
    //myRobot.MecanumDrive_Cartesian(stick.GetRawAxis(0), stick.GetRawAxis(1), 0);
    constexpr double LoweredPower = 0.5;
    double leftPower = stick.GetRawButton(5) ? LoweredPower : 1.0,
           rightPower = stick.GetRawButton(6) ? LoweredPower : 1.0,
           leftSpeed = stick.GetRawAxis(1) * leftPower, //left joystick, y axis
           rightSpeed = stick.GetRawAxis(5) * rightPower;    //right Joystick, y axis
    leftMotor.Set(leftSpeed);
    rightMotor.Set(rightSpeed);
}

void Robot::TestPeriodic() {
    lw->Run();
}

START_ROBOT_CLASS(Robot)
