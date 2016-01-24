#include "Robot.hpp"
#include "WPILib.h"

Robot::Robot() :
    myRobot(0, 1, 2, 3),
    stick(0),
    lw(LiveWindow::GetInstance()),
    autoLoopCounter(0),
    canControl(1)   //FIXME : set correct device ID
{
    myRobot.SetExpiration(0.1);
    myRobot.SetInvertedMotor(RobotDrive::kFrontRightMotor, true);
    myRobot.SetInvertedMotor(RobotDrive::kRearRightMotor, true);
}

void Robot::AutonomousInit() {
    autoLoopCounter = 0;
    startTime = std::chrono::system_clock::now();
}

void Robot::AutonomousPeriodic() {
    /*
    if(autoLoopCounter < 100) //Check if we've completed 100 loops (approximately 2 seconds)
    {
    */
    using namespace std::literals;
    if (startTime.time_since_epoch() < 2000ms) {
        myRobot.Drive(-0.5, 0.0); 	// drive forwards half speed
        autoLoopCounter++;
    }
    else {
        myRobot.Drive(0.0, 0.0); 	// stop robot
    }
}

void Robot::TeleopInit() {

}

void Robot::TeleopPeriodic() {
    SmartDashboard::PutNumber("number", 0.01);
    myRobot.MecanumDrive_Cartesian(stick.GetRawAxis(0), stick.GetRawAxis(1), 0);
}

void Robot::TestPeriodic() {
    lw->Run();
}

START_ROBOT_CLASS(Robot)
