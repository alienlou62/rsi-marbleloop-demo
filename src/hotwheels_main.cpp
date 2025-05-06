#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <csignal>
#include "SampleAppsHelper.h"
#include "rsi.h"

using namespace RSI::RapidCode;
using namespace std;

// === CONSTANTS ===
constexpr double SENSOR_DISTANCE = 0.1; // meters
constexpr double GRAVITY = 9.81;
constexpr double UNITS_PER_DEGREE = 67108864;
// constexpr double MAX_CATCHER_POSITION = 1.0;
// constexpr double MIN_CATCHER_POSITION = 0.0;
constexpr bool DEBUG_MODE = true;

// === ENUMS ===
enum AxisID
{
    RAMP = 0,
    DOOR = 1,
    CATCHER = 2
};

// === GLOBALS ===
MotionController *controller = nullptr;
Axis *motorRamp = nullptr;
Axis *motorDoor = nullptr;
// Axis *motorCatcher = nullptr;
volatile sig_atomic_t gShutdown = 0;

// === SIGNAL HANDLING ===
void SignalHandler(int signal)
{
    cout << "[Signal] Shutdown requested." << endl;
    gShutdown = 1;
    if (motorRamp)
        motorRamp->AmpEnableSet(false);
    if (motorDoor)
        motorDoor->AmpEnableSet(false);
    // if (motorCatcher)
    //     motorCatcher->AmpEnableSet(false);
}

// === RMP SETUP ===
void InitMotor(Axis *axis)
{
    axis->UserUnitsSet(UNITS_PER_DEGREE);
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

void MoveSCurve(Axis *axis, double pos)
{
    try
    {
        axis->MoveSCurve(pos);
    }
    catch (const std::exception &e)
    {
        cerr << "[Error] Move failed: " << e.what() << endl;
    }
}

void SetupRMP()
{
    MotionController::CreationParameters p;
    strncpy(p.RmpPath, "/rsi/", p.PathLengthMaximum);
    strncpy(p.NicPrimary, "enp1s0", p.PathLengthMaximum); // UPDATE NIC IF NEEDED

    controller = MotionController::Create(&p);
    SampleAppsHelper::StartTheNetwork(controller);

    motorRamp = controller->AxisGet(RAMP);
    motorDoor = controller->AxisGet(DOOR);
    // motorCatcher = controller->AxisGet(CATCHER);

    InitMotor(motorRamp);
    InitMotor(motorDoor);
    // InitMotor(motorCatcher);

    cout << "[RMP] Motors initialized.\n";
}

double ReadSensor(int sensorID)
{
    int ioNode = 0;
    int bitIndex = (sensorID == 1) ? 0 : 1;
    IO *digitalIO = controller->IOGet(ioNode);

    auto start = chrono::steady_clock::now();
    while (true)
    {
        if (digitalIO->BitGet(bitIndex) == 1)
        {
            return chrono::duration<double>(chrono::steady_clock::now().time_since_epoch()).count();
        }
        if (chrono::steady_clock::now() - start > chrono::seconds(5))
        {
            cerr << "[Warning] Sensor " << sensorID << " timeout." << endl;
            return 0.0; // Timeout fallback
        }
        this_thread::sleep_for(chrono::milliseconds(1));
    }
}

// === PHYSICS ===
double ComputeSpeed(double t1, double t2)
{
    return (t2 > t1) ? SENSOR_DISTANCE / (t2 - t1) : 0.0;
}

double ComputeLandingPosition(double speed, double angleDeg)
{
    double angleRad = angleDeg * M_PI / 180.0;
    double vx = speed * cos(angleRad);
    double vy = speed * sin(angleRad);
    double timeOfFlight = (2 * vy) / GRAVITY;
    return vx * timeOfFlight;
}

int main()
{
    std::signal(SIGINT, SignalHandler);
    cout << "[HotWheels] Starting demo...\n";

    try
    {
        SetupRMP();

        while (!gShutdown)
        {
            cout << "\n=== New Launch ===" << endl;

            double rampAngle;
            cout << "Enter ramp angle (degrees): ";
            cin >> rampAngle;

            // 1. Set ramp angle
            MoveSCurve(motorRamp, rampAngle);

            // 2. Wait for sensor 1 — car approaching gate
            double t1 = 0.0, t2 = 0.0;
            cout << "[Sensor] Waiting for sensor 1..." << endl;
            while (t1 == 0.0)
            {
                t1 = ReadSensor(1);
                this_thread::sleep_for(chrono::milliseconds(1));
            }

            // 3. Open door to let car through
            cout << "[Gate] Opening door!" << endl;
            MoveSCurve(motorDoor, 1.0);

            // 4. Wait for sensor 2 — car passed
            cout << "[Sensor] Waiting for sensor 2..." << endl;
            while (t2 == 0.0)
            {
                t2 = ReadSensor(2);
                this_thread::sleep_for(chrono::milliseconds(1));
            }

            // 5. Close door again
            cout << "[Gate] Closing door." << endl;
            MoveSCurve(motorDoor, 0.0);

            // 6. Compute physics
            double speed = ComputeSpeed(t1, t2);
            double landing = ComputeLandingPosition(speed, rampAngle);
            // landing = std::clamp(landing, MIN_CATCHER_POSITION, MAX_CATCHER_POSITION);

            cout << "[Physics] Speed: " << speed << " m/s | Landing: " << landing << " m" << endl;

            // 7. Move catcher
            // MoveSCurve(motorCatcher, landing);

            this_thread::sleep_for(chrono::seconds(3));
        }
    }
    catch (const std::exception &ex)
    {
        cerr << "[Fatal] Exception: " << ex.what() << endl;
    }

    // --- Shutdown Cleanup ---
    cout << "[Shutdown] Cleaning up...\n";
    if (controller)
    {
        try
        {
            motorRamp->AmpEnableSet(false);
            motorDoor->AmpEnableSet(false);
            // motorCatcher->AmpEnableSet(false);
            controller->Delete();
        }
        catch (...)
        {
            cerr << "[Cleanup] Error disabling motors or deleting controller.\n";
        }
    }

    cout << "[HotWheels] Demo finished.\n";
    return 0;
}