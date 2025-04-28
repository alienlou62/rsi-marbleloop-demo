cmake_minimum_required(VERSION 3.15)

set(CMAKE_CXX_STANDARD 20)

project(HotWheelsDemo)

# Include RSI SDK headers
include_directories(/rsi/examples/C++/include)

# Add your source files
add_executable(HotWheelsDemo
    src/hotwheels_main.cpp
)

# Set RMP Default Path
target_compile_definitions(HotWheelsDemo PUBLIC RMP_DEFAULT_PATH="/rsi")

# Link libraries AFTER creating the target
target_link_libraries(HotWheelsDemo PRIVATE /rsi/librapidcode.so)

# Optional: suppress warnings if needed
target_compile_options(HotWheelsDemo PRIVATE "-Wno-deprecated-enum-enum-conversion")
