cmake_minimum_required(VERSION 3.10)
project(RSIHotWheelsDemo)

set(CMAKE_CXX_STANDARD 14)

# --- Set the path to the RSI RapidCode SDK ---
# IMPORTANT: Make sure this matches where your SDK is installed.
# It sounds like your laser demo already uses "$HOME/rsi/RapidCode", so we match that.
set(RAPIDCODE_ROOT "$ENV{HOME}/rsi/RapidCode")

# --- Include and link RSI SDK ---
include_directories(${RAPIDCODE_ROOT}/include)
link_directories(${RAPIDCODE_ROOT}/lib)

# --- Add the main executable ---
add_executable(HotWheelsDemo src/hotwheels_main.cpp)

# --- Link against RSI libraries and required system libraries ---
target_link_libraries(HotWheelsDemo
    rsi
    pthread
    rt
)
