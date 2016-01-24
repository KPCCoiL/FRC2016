#ifndef STRONGHOLD_ROBOT_HPP
#define STRONGHOLD_ROBOT_HPP

#include "WPILib.h"
#include <chrono>

class Robot: public IterativeRobot {

	RobotDrive myRobot; // robot drive system
	Joystick stick; // only joystick
	LiveWindow* lw;
    CANTalonSRX canControl;
	int autoLoopCounter;
    std::chrono::time_point<std::chrono::system_clock> startTime;

public:
	Robot();
    static constexpr double UpdatePeriod = 0.010;

private:
	void AutonomousInit();
	void AutonomousPeriodic();
	void TeleopInit();
	void TeleopPeriodic();
	void TestPeriodic();
};

#endif //end of include guard
