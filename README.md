# RSI Hot Wheels Demo

A mechanical demonstration showcasing the precision of the RMP system at Robotics Systems Integration (RSI).  
This system uses a motorized, user-set ramp, sensor-triggered gate, and floor-based catcher to predict and intercept a Hot Wheels car in real time.

## Features
- Adjustable ramp angle via motor and encoder  
- Real-time speed sensing with dual sensors  
- Predictive control of gate and catcher using RMP  

## Build Instructions
```bash
cd build
cmake ..
make
./HotWheels_Demo
