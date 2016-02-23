#include "Robot.hpp"
#include "WPILib.h"
#include <limits>
#include <chrono>

Robot::Robot() :
    stick(0),
    lw(LiveWindow::GetInstance()),
    leftMotor(0),
    rightMotor(1),
    arm(2),
    armUpperLimit(0),
    armLowerLimit(1),
    camera(CameraServer::GetInstance()),
    funcID(0)
{
    leftMotor.SetInverted(true);
    SmartDashboard::init();
    camera->SetQuality(50);
    camera->StartAutomaticCapture("cam0");
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
    SmartDashboard::PutNumber("ArmLimitedOmega", 0.4);
}

void Robot::Advance(double leftSpeed, double rightSpeed) {
    leftMotor.Set(funcs.at(funcID).first(leftSpeed));
    rightMotor.Set(funcs.at(funcID).first(rightSpeed));
}

void Robot::UpdateFunc() {
    static bool yPressed = false;
    bool currentY = stick.GetRawButton(ButtonY);
    if (!yPressed && currentY) {
        funcID = (funcID + 1) % funcs.size();
        SmartDashboard::PutString("Function", funcs.at(funcID).second);
    }
    yPressed = currentY;
}

void Robot::MoveWheels() {
    constexpr double LoweredPower = 0.3,
                     Epsilon = std::numeric_limits<double>::epsilon();
    double leftRotate = stick.GetRawAxis(LeftTrigger),
           rightRotate = stick.GetRawAxis(RightTrigger),
           leftPower = stick.GetRawButton(ButtonL) ? LoweredPower : 1.0,
           rightPower = stick.GetRawButton(ButtonR) ? LoweredPower : 1.0,
           leftSpeed = stick.GetRawAxis(LeftYAxis) * leftPower,
           rightSpeed = stick.GetRawAxis(RightYAxis) * rightPower;
    if (leftRotate > Epsilon) Advance(leftRotate, -leftRotate);
    else if (rightRotate > Epsilon) Advance(-rightRotate, rightRotate);
    else Advance(leftSpeed, rightSpeed);
    UpdateFunc();
}

void Robot::MoveArm() {
    double const ArmLimitedOmega = SmartDashboard::GetNumber("ArmLimitedOmega", 0.4);
    bool btnAPressed = stick.GetRawButton(ButtonA),
         btnBPressed = stick.GetRawButton(ButtonB),
         btnXPressed = stick.GetRawButton(ButtonX);
    auto const ArmOmega = btnXPressed ? 1.0 : ArmLimitedOmega;
    if (btnAPressed ^ btnBPressed) {
        if (btnAPressed && armUpperLimit.Get()) arm.Set(ArmOmega);
        else if (btnBPressed && armLowerLimit.Get()) arm.Set(-ArmOmega);
    }
    else arm.Set(0.0);
}

void Robot::TeleopPeriodic() {
    MoveWheels();
    MoveArm();
}

void Robot::TestPeriodic() {
    SmartDashboard::PutBoolean("upper limit sensor pressed", !armUpperLimit.Get());
    SmartDashboard::PutBoolean("lower limit sensor pressed", !armLowerLimit.Get());
    lw->Run();
}

START_ROBOT_CLASS(Robot)
