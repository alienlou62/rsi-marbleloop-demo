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
constexpr double UNITS_PER_DEGREE = 186413.5111;

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
IOPoint *sensor1Input = nullptr;
IOPoint *sensor2Input = nullptr;

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
        //  Motion parameters — tune as needed
        double velocity = 50.0;         // deg/sec
        double acceleration = 300.0;    // deg/sec²
        double deceleration = 300.0;    // deg/sec²
        double jerkPercent = 0.0;       // 0 = trapezoidal

        axis->MoveSCurve(pos, velocity, acceleration, deceleration, jerkPercent);
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
    strncpy(p.NicPrimary, "enp6s0", p.PathLengthMaximum);
    p.CpuAffinity = 3;

    controller = MotionController::Create(&p);
    SampleAppsHelper::CheckErrors(controller);

    SampleAppsHelper::StartTheNetwork(controller);

    // Motor setup
    motorRamp = controller->AxisGet(RAMP);
    motorDoor = controller->AxisGet(DOOR);
    InitMotor(motorRamp);
    InitMotor(motorDoor);
    cout << "[RMP] Motors initialized.\n";

    try
    {
        int sensorNodeIndex = 1; // AKD = second node on the network

        // ✅ Create IOPoint from network node (not axis)
        sensor1Input = IOPoint::CreateDigitalInput(controller->NetworkNodeGet(sensorNodeIndex), 0); // Input 0
        sensor2Input = IOPoint::CreateDigitalInput(controller->NetworkNodeGet(sensorNodeIndex), 1); // Input 1

        cout << "[I/O] Digital inputs created successfully.\n";
    }
    catch (const std::exception &e)
    {
        cerr << "[ERROR] Failed to create digital inputs: " << e.what() << endl;
        exit(1);
    }
}

double ReadSensor(IOPoint *sensorInput)
{

    if (!sensorInput)
    {
        cerr << "[ERROR] Sensor pointer is null.\n";
        return 0.0;
    }

    auto start = chrono::steady_clock::now();

    while (true)
    {
        try
        {
            bool val = sensorInput->Get();
            cout << "[Debug] Sensor value: " << val << endl;
            if(val == 0.0){
                return 0.0;
            }
            return chrono::duration<double>(chrono::steady_clock::now().time_since_epoch()).count();
        }
        catch (const std::exception &ex)
        {
            cerr << "[ERROR] Sensor read failed: " << ex.what() << " | Pointer: " << sensorInput << endl;
            return 0.0;
        }

        catch (const std::exception &ex)
        {
            cerr << "[ERROR] Sensor read failed: " << ex.what() << endl;
            return 0.0;
        }

        if (chrono::steady_clock::now() - start > chrono::seconds(5))
        {
            cerr << "[Warning] Sensor timeout.\n";
            return 0.0;
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
    // motorRamp->AmpEnableSet(false);

    try
    {
        SetupRMP();
        if (!controller)
        {
            cerr << "[Fatal] Controller pointer is null after SetupRMP." << endl;
            return 1;
        }

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
                t1 = ReadSensor(sensor1Input);
                cout << "[Debug] t1 value: " << t1 << endl;
                this_thread::sleep_for(chrono::milliseconds(1));
            }

            // 3. Open door to let car through
            cout << "[Gate] Opening door!" << endl;
            MoveSCurve(motorDoor, 1);

            // 4. Wait for sensor 2 — car passed
            cout << "[Sensor] Waiting for sensor 2..." << endl;
            while (t2 == 0.0)
            {
                t2 = ReadSensor(sensor2Input);
                cout << "[Debug] t2 value: " << t2 << endl;
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
