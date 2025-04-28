#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include "SampleAppsHelper.h"
#include "rsi.h"

using namespace RSI::RapidCode;

constexpr double SENSOR_DISTANCE = 0.1;  // meters
constexpr double GRAVITY = 9.81;
constexpr double RAMP_HEIGHT = 0.2;
constexpr double RAMP_LENGTH = 0.5;

constexpr double ANGLE_UNITS = 67108864;  // user unit scaling

MotionController* controller = nullptr;
Axis* motorRamp = nullptr;
Axis* motorDoor = nullptr;
Axis* motorCatcher = nullptr;

void InitMotor(Axis* axis) {
    axis->UserUnitsSet(ANGLE_UNITS);
    axis->ErrorLimitTriggerValueSet(0.5);
    axis->ErrorLimitActionSet(RSIAction::RSIActionNONE);
    axis->HardwareNegLimitTriggerStateSet(1);
    axis->HardwarePosLimitTriggerStateSet(1);
    axis->HardwareNegLimitActionSet(RSIAction::RSIActionNONE);
    axis->HardwarePosLimitActionSet(RSIAction::RSIActionNONE);
    axis->HardwareNegLimitDurationSet(2);
    axis->HardwarePosLimitDurationSet(2);
    axis->ClearFaults();
    axis->AmpEnableSet(true);
    axis->CommandPositionSet(0.0);
}

void MoveTo(Axis* axis, double pos) {
    try {
        axis->MoveSCurve(pos);
        std::cout << "MoveSCurve -> " << pos << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Move failed: " << e.what() << std::endl;
    }
}

// Placeholder sensor input
double readSensor(int id) {
    std::cout << "Reading sensor " << id << std::endl;
    return 0.0;  // Replace with real input
}

double calculateSpeed(double t1, double t2) {
    return (t2 > t1) ? SENSOR_DISTANCE / (t2 - t1) : 0.0;
}

double calculateLanding(double speed, double angleDeg) {
    double angleRad = angleDeg * M_PI / 180.0;
    double vx = speed * cos(angleRad);
    double vy = speed * sin(angleRad);
    double flightTime = (2 * vy) / GRAVITY;
    return vx * flightTime;
}

int main() {
    // Initialize controller
    MotionController::CreationParameters params;
    strncpy(params.RmpPath, "/rsi/", params.PathLengthMaximum);
    strncpy(params.NicPrimary, "enp1s0", params.PathLengthMaximum);
    controller = MotionController::Create(&params);
    SampleAppsHelper::StartTheNetwork(controller);

    // Assign motors
    motorRamp = controller->AxisGet(0);
    motorDoor = controller->AxisGet(1);
    motorCatcher = controller->AxisGet(2);

    InitMotor(motorRamp);
    InitMotor(motorDoor);
    InitMotor(motorCatcher);

    while (true) {
        std::cout << "\n--- New Run ---\n";
        double rampAngle = 20.0;  // could be user-set
        MoveTo(motorRamp, rampAngle);

        // Open gate
        MoveTo(motorDoor, 1.0);  // Open
        std::this_thread::sleep_for(std::chrono::seconds(2));
        MoveTo(motorDoor, 0.0);  // Close (releases car)

        // Wait for sensors (mocked)
        double t1 = 0.0, t2 = 0.0;
        while (t1 == 0.0) { t1 = readSensor(1); std::this_thread::sleep_for(std::chrono::milliseconds(1)); }
        while (t2 == 0.0) { t2 = readSensor(2); std::this_thread::sleep_for(std::chrono::milliseconds(1)); }

        double speed = calculateSpeed(t1, t2);
        double landing = calculateLanding(speed, rampAngle);
        landing = std::max(0.0, std::min(landing, 1.0));  // constrain to rail

        std::cout << "Speed: " << speed << " m/s | Landing: " << landing << " m\n";
        MoveTo(motorCatcher, landing);

        std::this_thread::sleep_for(std::chrono::seconds(5));  // wait for next
    }

    controller->Delete();
    return 0;
}
