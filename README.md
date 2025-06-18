# Electronic Stability Control (ESC) ECU Implementation

This project implements a complete Electronic Stability Control (ESC) system for automotive applications in C. The ESC system helps prevent vehicle skidding and loss of control by monitoring vehicle dynamics and automatically applying individual wheel brakes and reducing engine torque when necessary.

## Features

### Core ESC Functionality
- **Yaw Stability Control**: Detects and corrects understeer and oversteer conditions
- **Rollover Prevention**: Monitors lateral acceleration to prevent vehicle rollover
- **Sensor Fusion**: Processes data from multiple sensors (wheel speed, steering angle, yaw rate, lateral acceleration)
- **Real-time Control**: 100Hz control loop for responsive intervention
- **Vehicle Dynamics Model**: Bicycle model for reference yaw rate calculation

### Advanced Sensor Interface Module
- **Asynchronous Data Acquisition**: Independent sensor threads running at optimal frequencies (1000Hz wheel speed, 200Hz IMU, 100Hz steering)
- **Multi-Interface Support**: ADC analog, digital/PWM, CAN bus, SPI, and UART sensor interfaces
- **Hardware Abstraction Layer**: Clean separation between sensor hardware and ESC algorithms
- **Unit Conversion Logic**: Automatic conversion from raw sensor values to engineering units
- **Digital Filtering**: Configurable low-pass, median, moving average, and debounce filters per sensor
- **Signal Quality Monitoring**: Real-time assessment of sensor signal integrity and noise levels

### Safety & Diagnostics
- **Self-Test**: System initialization with built-in diagnostics
- **Advanced Fault Detection**: Per-sensor health monitoring with status reporting (stuck, noisy, out-of-range)
- **Sensor Calibration**: Runtime calibration capabilities with validation
- **Graceful Degradation**: Safe operation modes when faults are detected
- **Warning Systems**: ESC warning light and brake light control

### Implementation Details
- **Modular Design**: Clean separation between sensor acquisition, control algorithms, and actuators
- **Thread-Safe Architecture**: Mutex-protected data structures for multi-threaded operation
- **Configurable Parameters**: Vehicle-specific and sensor-specific parameters for different configurations
- **Advanced Signal Processing**: Multi-stage filtering with adaptive adjustment based on signal quality
- **Precise Timing**: High-resolution timing for consistent control loop execution
- **Hardware Simulation**: Built-in sensor simulation for development and testing

## System Architecture

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   Sensors       │────│   ESC ECU        │────│   Actuators     │
│                 │    │                  │    │                 │
│ • Wheel Speed   │    │ • Sensor Fusion  │    │ • Brake Pressure│
│ • Steering Angle│    │ • Vehicle Model  │    │ • Engine Torque │
│ • Yaw Rate      │    │ • Control Logic  │    │ • Warning Lights│
│ • Lateral Accel │    │ • Diagnostics    │    │                 │
└─────────────────┘    └──────────────────┘    └─────────────────┘
```

## Building and Running

### Prerequisites
- GCC compiler with C99 support
- Make utility
- POSIX-compliant system (Linux, macOS, Unix)

### Build Instructions
```bash
# Build the ESC simulation
make

# Build and run
make run

# Run with verbose output (shows status every cycle)
make run-verbose

# Clean build artifacts
make clean
```

### Manual Build
```bash
gcc -Wall -Wextra -std=c99 -O2 -g main.c esc_ecu.c -o esc_ecu -lm -lrt
```

## Usage

### Basic Operation
```bash
./esc_ecu          # Run simulation with periodic status updates
./esc_ecu -v       # Run with verbose output (every cycle)
```

### Control During Execution
- **Ctrl+C**: Gracefully shutdown the simulation
- The simulation runs continuously, displaying system status and ESC interventions

### Sample Output
```
=== ESC System Status (Cycle: 1234) ===
Status: ACTIVE
Vehicle Speed: 22.50 m/s
Steering Angle: 0.085 rad (4.9 deg)
Yaw Rate: 0.125 rad/s
Lateral Accel: 2.81 m/s²
Reference Yaw Rate: 0.095 rad/s
Yaw Rate Error: 0.030 rad/s
Understeer: NO, Oversteer: YES, Rollover Risk: NO
Brake Pressures [FL, FR, RL, RR]: [45.2, 0.0, 0.0, 0.0] bar
Engine Torque Reduction: 15.3%
*** ESC WARNING LIGHT ACTIVE ***
```

## Configuration

### Vehicle Parameters
Edit the `init_vehicle_parameters()` function in `main.c` to configure for different vehicles:

```c
params->wheelbase = 2.7f;                    // meters
params->track_width = 1.5f;                  // meters  
params->mass = 1500.0f;                      // kg
params->inertia_z = 2500.0f;                // kg*m^2
params->cg_height = 0.5f;                   // meters
params->cornering_stiffness_front = 60000.0f; // N/rad
params->cornering_stiffness_rear = 55000.0f;  // N/rad
params->wheel_radius = 0.32f;               // meters
```

### Control Parameters
Adjust thresholds in `esc_ecu.c`:

```c
#define YAW_RATE_THRESHOLD 0.1f        // rad/s
#define LATERAL_ACCEL_THRESHOLD 4.0f   // m/s^2
#define SLIP_ANGLE_THRESHOLD 0.1f      // radians
#define ROLLOVER_THRESHOLD 0.8f        // lateral acceleration ratio
#define MIN_VEHICLE_SPEED 2.0f         // m/s
```

## File Structure

- **`esc_ecu.h`**: Main ESC header file with data structures and function prototypes
- **`esc_ecu.c`**: Core ESC implementation with control algorithms
- **`sensor_interface.h`**: Sensor interface module header with hardware abstraction
- **`sensor_interface.c`**: Asynchronous sensor acquisition and processing implementation
- **`digital_filters.c`**: Digital filtering algorithms (low-pass, median, moving average, debounce)
- **`main.c`**: Main application with simulation loop and vehicle simulation
- **`Makefile`**: Build configuration for multi-file project
- **`README.md`**: This documentation file

## Control Algorithms

### Understeer Correction
- Detects when actual yaw rate is less than reference yaw rate
- Applies brake to inside rear wheel to induce corrective yaw moment
- Helps vehicle follow intended steering input

### Oversteer Correction  
- Detects when actual yaw rate exceeds reference yaw rate
- Applies brake to outside front wheel to reduce yaw moment
- Reduces engine torque to help stabilize the vehicle

### Rollover Prevention
- Monitors lateral acceleration relative to rollover threshold
- Applies brakes to all wheels when rollover risk is detected
- Significantly reduces engine torque for maximum stability

## Integration Notes

For integration into a real ECU system:

1. **Sensor Interface**: Replace simulation code with actual hardware interfaces (CAN, SPI, ADC)
2. **Actuator Control**: Implement actual brake pressure control and engine torque requests
3. **Memory Management**: Consider stack usage and dynamic allocation for embedded systems
4. **Timing**: Ensure deterministic execution times for real-time operation
5. **Calibration**: Load vehicle parameters from EEPROM or calibration files
6. **Compliance**: Ensure adherence to automotive standards (ISO 26262, MISRA-C)

## Safety Considerations

This implementation is for educational and demonstration purposes. Production ESC systems require:

- Extensive validation and testing
- Redundant sensors and fail-safe mechanisms  
- Compliance with automotive safety standards
- Hardware-in-the-loop testing
- Vehicle testing under various conditions
- Certification by automotive authorities

## License

This code is provided for educational purposes. Use in production systems requires proper validation, testing, and compliance with applicable safety standards.