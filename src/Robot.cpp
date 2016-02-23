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
    shooter(0), //TODO: set correct port ID
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
    std::chrono::milliseconds const waitDuration(
            static_cast<long long int>(
                SmartDashboard::GetNumber("autonomous running time (in ms)", 5000)));
    while (startTime.time_since_epoch() < waitDuration)
        Advance(0.5, 0.5);
}

void Robot::TeleopInit() {
    SmartDashboard::PutNumber("ArmLimitedOmega", 0.4);
}

void Robot::Advance(double leftSpeed, double rightSpeed) {
    leftMotor.Set(funcs.at(funcID).first(leftSpeed));
    rightMotor.Set(funcs.at(funcID).first(rightSpeed));
}

void Robot::UpdateFunc() {
    static bool startPressed = false;
    bool currentStart = stick.GetRawButton(ButtonStart);
    if (!startPressed && currentStart) {
        funcID = (funcID + 1) % funcs.size();
        SmartDashboard::PutString("Function", funcs.at(funcID).second);
    }
    startPressed = currentStart;
}

void Robot::MoveWheels() {
    constexpr auto Epsilon = std::numeric_limits<double>::epsilon();
    double leftRotate = stick.GetRawAxis(LeftTrigger),
           rightRotate = stick.GetRawAxis(RightTrigger),
           leftSpeed = stick.GetRawAxis(LeftYAxis),
           rightSpeed = stick.GetRawAxis(RightYAxis);
    if (leftRotate > Epsilon) Advance(leftRotate, -leftRotate);
    else if (rightRotate > Epsilon) Advance(-rightRotate, rightRotate);
    else Advance(leftSpeed, rightSpeed);
    UpdateFunc();
}

template <class Controller>
void Robot::MoveArmLike(Controller& motor, StickButtons forward, StickButtons backward) {
    double const ArmLimitedOmega = SmartDashboard::GetNumber("ArmLimitedOmega", 0.4);
    bool goForward = stick.GetRawButton(forward),
         goBackward = stick.GetRawButton(backward),
         unlimited = stick.GetRawButton(ButtonR);
    auto const ArmOmega = unlimited ? 1.0 : ArmLimitedOmega;
    if (goForward ^ goBackward) {
        if (goForward) motor.Set(ArmOmega);
        else if (goBackward) motor.Set(-ArmOmega);
    }
    else motor.Set(0.0);
}

void Robot::TeleopPeriodic() {
    MoveWheels();
    MoveArmLike(arm, ButtonA, ButtonB);
    MoveArmLike(shooter, ButtonX, ButtonY);
}

void Robot::TestPeriodic() {
    lw->Run();
}

START_ROBOT_CLASS(Robot)
