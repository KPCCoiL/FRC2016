#ifndef STRONGHOLD_ROBOT_HPP
#define STRONGHOLD_ROBOT_HPP

#include "WPILib.h"
#include <chrono>
#include <array>
#include <functional>
#include <cmath>
#include <string>

constexpr double sig(double x) {
    return std::signbit(x) ? -1 : 1;
}

class Robot: public IterativeRobot {

        Joystick stick; // only joystick
        LiveWindow* lw;
        CANTalon leftMotor, rightMotor, arm;
        DigitalInput armUpperLimit, armLowerLimit;
        std::chrono::time_point<std::chrono::system_clock> startTime;
        std::size_t funcID;
        std::array<std::pair<std::function<double(double)>, std::string>, 5> const funcs = {{
            {[](auto x) { return std::tan(x) / std::tan(1); }, "y=tanx/tan1"},
            {[](auto x) { return sig(x) * x * x; }, "y=sig(x)x^2"},
            {[](auto x) { return sig(x) * x * std::tan(x) / std::tan(1); }, "y=sig(x)xtanx/tan1"},
            {[](auto x) { return x * x * x; }, "y=x^3"},
            {[](auto x) { return std::pow(x, 5); }, "y=x^5"}
        }};

        enum StickButtons {
            ButtonA = 1,
            ButtonB,
            ButtonX,
            ButtonY,
            ButtonL,
            ButtonR,
            ButtonBack,
            ButtonStart,
            ButtonLeftJoyStick,
            ButtonRightJoyStick,
        };

        void AutonomousInit();
        void AutonomousPeriodic();
        void TeleopInit();
        void TeleopPeriodic();
        void TestPeriodic();
        void MoveWheels();
        void MoveArm();


    public:
        Robot();
        static constexpr double UpdatePeriod = 0.010;

};

#endif //end of include guard
