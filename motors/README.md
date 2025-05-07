# MotorAPI – High-Level Modbus RTU Motor Control

This C++ library provides a high-level interface to control motors via Modbus RTU using [`libmodbus`](https://libmodbus.org/). It abstracts low-level register operations into simple function calls like `setSpeed()`, `setTorque()`, and `getActualPosition()`.

## Features

- Connects to motor over Modbus RTU via serial
- Set motor torque, velocity, and position
- Read actual motor torque, velocity, and position
- Set operation modes (position, velocity, torque)
- Includes safety checks and debug logging
- GTest-compatible for unit testing #TODO

## Dependencies

- [libmodbus](https://libmodbus.org/)
- C++17 or later
- GoogleTest (optional for testing)

## Usage

### Initialization

<pre><code class="language-cpp">
Motor motor("/dev/ttyUSB0", 1);
if (!motor.initializeMotor()) {
    std::cerr &lt;&lt; "Failed to connect to motor.\n";
}
</code></pre>

### Set Operation Mode

<pre><code class="language-cpp">
motor.setOperationMode(PV_MODE);  // PV_MODE, PT_MODE, or PP_MODE
</code></pre>

### Control Commands

<pre><code class="language-cpp">
motor.setTargetVelocity(1000);     // in INC/s
motor.setTorque(500);              // in ‰ (permille)
motor.setPosition(100000);         // in INC
</code></pre>

### Read Feedback

<pre><code class="language-cpp">
int32_t pos = motor.getActualPosition();
int32_t speed = motor.getActualVelocity();
int16_t torque = motor.getActualTorque();
</code></pre>

### Cleanup

<pre><code class="language-cpp">
motor.stopMotor();
motor.disconnectMotor();
</code></pre>

## Running Tests (Optional)

If hardware is connected and available:

```bash
mkdir build && cd build
cmake ..
make
./motor_api_test
```

## Notes

- `setTorque()` enforces bounds between -3000‰ and +3000‰.
- `setMaxTorque()` only allows values up to 3000‰.
- All `read*()` functions return zero if a read fails, and error messages are printed to `stderr`.
- The motor must be placed in the correct operation mode (`PP_MODE`, `PV_MODE`, `PT_MODE`) before calling `setPosition()`, `setTargetVelocity()`, or `setTorque()`.
- This API assumes motors follow the Modbus RTU protocol and register map of the EZmotion PRS/SRS R2 series.
- Always verify that your power supply voltage and current limits match the motor specifications.
- Communication defaults:
  - Baud rate: `115200`
  - Parity: Even (`'E'`)
  - Data bits: 8
  - Stop bits: 1

## Reference

This API is designed based on the **PRS SRS R2 Series User Guide** documentation.  
Refer to **Section 6: Register Table** in the official User Guide for detailed register definitions and explanations.
