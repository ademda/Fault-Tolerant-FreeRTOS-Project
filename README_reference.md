# STM32 Extended Kalman Filter (EKF) for Attitude Estimation

This repository documents my journey implementing various Digital Signal Processing (DSP) algorithms on STM32F401RE microcontroller. While the repo contains implementations of FIR filters, IIR filters, and moving average filters, this README focuses specifically on the **Extended Kalman Filter (EKF)** implementation for roll and pitch angle estimation using MPU6050 sensor fusion.

*Note: Detailed documentation for other DSP algorithms (FIR, IIR, Moving Average) will be added in future updates.*

## Table of Contents
- [Overview](#overview)
- [Hardware Setup](#hardware-setup)
- [MPU6050 Sensor Integration](#mpu6050-sensor-integration)
- [Extended Kalman Filter Implementation](#extended-kalman-filter-implementation)
- [Mathematical Background](#mathematical-background)
- [Results and Performance](#results-and-performance)
- [Project Structure](#project-structure)
- [Getting Started](#getting-started)
- [Configuration](#configuration)

## Overview

This project demonstrates Extended Kalman Filter implementation for real-time attitude estimation in embedded systems. The EKF optimally fuses gyroscope and accelerometer data from an MPU6050 sensor to provide accurate, drift-free roll and pitch angle estimates.

### Key Features
- **Sensor Fusion**: Combines gyroscope and accelerometer data optimally
- **Drift Compensation**: Eliminates long-term gyroscope integration drift
- **Real-time Processing**: Optimized for microcontroller constraints
- **Mathematical Rigor**: Complete EKF implementation with Jacobian matrices



## Hardware Setup

- **Microcontroller**: STM32F401RE (Nucleo board)
- **IMU Sensor**: MPU6050 6-axis motion tracking device
- **Communication**: I2C interface with DMA support
- **Debug Output**: UART2 for serial monitoring
- **Optional**: Interrupt pin for data-ready signaling

### Wiring
```
MPU6050 -> STM32F401RE
VCC     -> 3.3V
GND     -> GND
SCL     -> PB8 (I2C1_SCL)
SDA     -> PB9 (I2C1_SDA)
INT     -> PC2 (Optional - Data Ready Interrupt)
```

### MPU6050 Advanced Features
- **DMA Support**: I2C transactions can utilize DMA for non-blocking data transfers
- **Interrupt Pin**: INT pin generates interrupts on data ready by the sensor.

## MPU6050 Sensor Integration

The MPU6050 is a 6-axis motion tracking device that combines a 3-axis gyroscope and 3-axis accelerometer in a single package.

### Gyroscope Data
- **Purpose**: Measures angular velocities (°/s) around X, Y, Z axes
- **Advantages**: High frequency response with accurate short-term measurements
- **Disadvantages**: Subject to integration drift over extended periods
- **Usage**: Primary input for the prediction step of the Kalman filter

### Accelerometer Data
- **Purpose**: Measures linear accelerations including the gravity vector
- **Advantages**: Provides absolute orientation reference when device is stationary
- **Disadvantages**: Noisy measurements that are affected by external accelerations
- **Usage**: Measurement input for the correction step of the Kalman filter

### Sensor Fusion Challenge
The key challenge in this implementation is optimally combining these two complementary sensors, each with their own strengths and weaknesses:
- **Gyroscope**: Provides excellent short-term accuracy but suffers from poor long-term stability due to integration drift
- **Accelerometer**: Offers good long-term reference for absolute orientation but has poor short-term precision due to measurement noise

### MPU6050 Library Features

#### DMA-Enabled I2C Communication
The MPU6050 library is designed to support DMA-based I2C transactions, which enables efficient and non-blocking data transfer from the sensor:
```c
// Non-blocking I2C read with DMA
HAL_I2C_Mem_Read_DMA(&hi2c1, MPU6050_ADDR, MPU6050_REG_ACCEL_XOUT_H, 
                     I2C_MEMADD_SIZE_8BIT, buffer, 14);
```
**Key Benefits of DMA Implementation**:
- The CPU remains free to execute other tasks while data transfer occurs in the background
- Provides consistent timing characteristics essential for real-time filtering applications
- Significantly reduces interrupt overhead compared to blocking I2C operations
- Improves overall system responsiveness and allows for higher sampling rates

#### Interrupt-Driven Data Ready Signal
The MPU6050's INT pin provides hardware-level synchronization capabilities that enable precise timing control:
```c
// Configure MPU6050 interrupt for data ready signal
MPU6050_WriteReg(MPU6050_REG_INT_ENABLE, 0x01);  // Enable data ready interrupt
HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);            // Handle interrupt on PC2
```
**Available Interrupt Sources**:
- **Data Ready**: Signals when new sensor data is available for reading
- **Motion Detection**: Triggers when acceleration exceeds a programmable threshold  

**Key Advantages of Interrupt-Based Operation**:
- Provides precise timing synchronization between sensor sampling and data processing
- Enables power-efficient operation by allowing the CPU to sleep between measurements
- Guarantees data freshness by signaling exactly when new measurements are available
- Significantly reduces polling overhead and improves system efficiency

## Kalman Filter Theory and Implementation

### Linear vs Extended Kalman Filter

#### Linear Kalman Filter (LKF)
The standard Kalman filter is designed to work optimally with linear systems that have Gaussian noise characteristics:
- **System Model**: `x(k+1) = A*x(k) + B*u(k) + w(k)` where all relationships are linear
- **Measurement Model**: `z(k) = H*x(k) + v(k)` using linear measurement equations
- **Optimal Performance**: Provides mathematically optimal estimates for linear systems with additive white Gaussian noise
- **Computational Requirements**: Relatively low computational cost involving only standard matrix operations
- **Best Applications**: Simple sensor configurations and linear system dynamics such as position and velocity tracking

#### Extended Kalman Filter (EKF)
The Extended Kalman Filter handles nonlinear systems by linearizing them around the current state estimate at each time step:
- **System Model**: `x(k+1) = f(x(k), u(k)) + w(k)` where f() can be any nonlinear function
- **Measurement Model**: `z(k) = h(x(k)) + v(k)` allowing for nonlinear measurement relationships
- **Linearization Process**: Uses Jacobian matrices F = ∂f/∂x and H = ∂h/∂x to approximate the nonlinear functions locally
- **Computational Requirements**: Higher computational cost due to the need for Jacobian matrix calculations at each iteration
- **Ideal Applications**: Nonlinear systems such as attitude estimation where trigonometric functions are involved in the mathematics
- **Gaussian Assumption Limitations**: When the relationship between state and measurement is nonlinear, the uncertainty of the state will no longer follow a Gaussian distribution, which can affect filter optimality

### EKF for Attitude Estimation

The EKF is specifically chosen for this application because attitude estimation involves trigonometric functions in both the process and measurement models, making it inherently nonlinear.

### State Vector
The filter estimates two angles:
```
x = [φ, θ]ᵀ = [roll, pitch]ᵀ
```

### Process Model
The nonlinear state transition is based on Euler angle kinematics:

```
φ̇ = p + tan(θ)(sin(φ)q + cos(φ)r)
θ̇ = cos(φ)q - sin(φ)r
```

Where:
- `p, q, r` = body angular rates from gyroscope (rad/s)
- `φ, θ` = roll and pitch angles (rad)

### Measurement Model

### Accelerometer Measurement Model

The accelerometer measures the complete acceleration vector:

```
a_measured = a_gravity + a_linear + n_accel(noise)
```

For attitude estimation, assuming quasi-static conditions (a_linear ≈ 0), the measurement model simplifies to:

```
h(x) = [g·sin(θ), -g·cos(θ)·sin(φ), -g·cos(θ)·cos(φ)]ᵀ
```

Where `g ≈ 9.81 m/s²` is the gravitational acceleration.

### EKF Algorithm Steps

#### 1. Prediction Step
```
x̂ₖ₊₁|ₖ = x̂ₖ|ₖ + T·f(x̂ₖ|ₖ, uₖ)
Pₖ₊₁|ₖ = AₖPₖ|ₖAₖᵀ + Q
```

Where:
- `Aₖ = I + T·F` (discrete-time transition matrix)
- `F` = Jacobian of the process model
- `Q` = process noise covariance (2×2)
- `T` = sampling period

#### 2. Update Step
```
yₖ = zₖ - h(x̂ₖ₊₁|ₖ)           (innovation)
Sₖ = CₖPₖ₊₁|ₖCₖᵀ + R          (innovation covariance)
Kₖ = Pₖ₊₁|ₖCₖᵀSₖ⁻¹           (Kalman gain)
x̂ₖ₊₁|ₖ₊₁ = x̂ₖ₊₁|ₖ + Kₖyₖ     (state update)
Pₖ₊₁|ₖ₊₁ = (I - KₖCₖ)Pₖ₊₁|ₖ   (covariance update)
```

Where:
- `Cₖ` = Jacobian of measurement model (3×2)
- `R` = measurement noise covariance (3×3)
- `zₖ` = accelerometer measurements

### Jacobian Matrices

#### Process Model Jacobian (F)
```
F = [∂f₁/∂φ   ∂f₁/∂θ]
    [∂f₂/∂φ   ∂f₂/∂θ]

where:
∂f₁/∂φ = tan(θ)(cos(φ)q - sin(φ)r)
∂f₁/∂θ = sec²(θ)(sin(φ)q + cos(φ)r)
∂f₂/∂φ = -(q·sin(φ) + r·cos(φ))
∂f₂/∂θ = 0
```

#### Measurement Model Jacobian (C)
```
C = [∂h₁/∂φ   ∂h₁/∂θ ]   [0           g·cos(θ)    ]
    [∂h₂/∂φ   ∂h₂/∂θ ] = [-g·cos(θ)·cos(φ)  g·sin(θ)·sin(φ)]
    [∂h₃/∂φ   ∂h₃/∂θ ]   [g·cos(θ)·sin(φ)   g·sin(θ)·cos(φ)]
```

## Challenges and Observations

### Sensor Interfacing
 You can find in the MPU6050 library two kinds of interfacing with IMU:
- I2C DMA Mode which enables non blocking method for data acquisition
- I2C Polling Mode which is a blocking method for data acquisition
1. DMA Mode is better in terms of computation, however this mode requires 
error handling in case of inconsistent Power for the sensor or noisy I2C line 
which may hang the DMA and freeze the communication.
2. Polling mode is straight forward and less complicated, perfect for testing 
purposes, However its less optimal because of its blocking mode causing non 
deterministic execution periods for prediction and update functions of ekf caused by 
data acquisition waits from the Sensor.  

### Covariance Values tuning 
1. Tuning the values of Initial covariance: since low values: 
0.01f for example will result in stable less noisy ekf output but slower 
convergence towards the minimal covariance.
but higher P values will result in faster convergence but higher noise and
more influence of the measurment

2. Tuning the R(measurment covariance) and Q(process covariance) values:
Since the two R and Q don't share same units the order of the values differ.
More importantly, I witnessed a covariance collapse where the ekf converges
towards trusting the measurment cause by over reduction of P value over time
![Over Fitting](img/overFitting.png)
#### Solutions
- Add a covariance term in the update step:P = (I - K C) P + KRK' to avoid covariance collapse but always adding a positive term to P matrix
- Increase R value to ensure that the ekf doesn't over trust the measurment values
 
### Update Model choice
1. The question was what should be the input of the update step:
- A non linear system with the input as (Ax, Ay, Az) and using jacobian approximation
and h matrix for measurment-state transition and covariance update 
- A linear system with input as (φ, θ) as measurments and doing the conversion of 
(Ax, Ay, Az)->(φ, θ) outside the ekf thus avoiding the jacobian approximation since our 
system is linear.
#### Reasoning
- feeding (φ, θ) to the ekf will reduce its flexibility.Since those states hide the 
acceleration error so we can't check if the Ai goes out of reasonable values in case 
of linear acceleration happening.
- Also (φ, θ) doesn't scale well if we want to add gyro bias for example as a state element.Because in this case the (φ, θ,Bias) will become non linear thus resulting in jacobian approximation again and losing this linearity feature
#### Decision: Raw acceleromter values will be fed to the kalman filter

## Results and Performance

### Roll Angle Performance

#### Gyroscope Drift Issue
![Gyro Drift](img/roll/gyro%20drift.png)
*The gyroscope-only integration shows significant drift over time, demonstrating the need for sensor fusion.*

#### Complementary Filter vs EKF
![Complementary Filter](img/roll/complementary.png)
*Comparison between complementary filter and EKF approaches for roll estimation.*
*Using Complementary filter didn't reduce noise coming from acceleromter but EKF did.*
#### Final Roll Results
![Final Roll Result](img/roll/final_result.png)
*EKF successfully combines gyroscope and accelerometer data for accurate roll estimation.*

![Final Roll Static](img/roll/final_result_static.png)
*Static test showing stable roll angle estimation without drift.*

### Pitch Angle Performance

![Final Pitch Test](img/pitch/final_test.png)
*Pitch angle estimation performance showing the EKF's ability to track dynamic motion while maintaining stability.*

### Key Performance Metrics
- **Update Rate**: 100 Hz prediction
- **Convergence Time**: < 2 seconds from initialization
- **Drift Elimination**: Long-term stability maintained through accelerometer fusion
- **Dynamic Response**: Fast tracking of actual motion changes
- **Noise Reduction**: Significant improvement over raw sensor data

## Project Structure

```
├── Core/
│   ├── Inc/
│   │   ├── main.h
│   │   ├── stm32f4xx_hal_conf.h
│   │   └── stm32f4xx_it.h
│   └── Src/
│       ├── main.c              # Main application loop
│       ├── stm32f4xx_hal_msp.c
│       ├── stm32f4xx_it.c
│       └── system_stm32f4xx.c
├── Filters/
│   ├── Inc/
│   │   ├── KalmanRollPitch.h       # EKF header
│   │   ├── matrix_op.h             # Matrix operations for EKF
│   │   └── Other_DSP_Filters.h     # FIR, IIR
│   └── Src/
│       ├── KalmanRollPitch.c       # EKF implementation
│       └── Other_DSP_Filters.c     # FIR, IIR
├── MPU6050/
│   ├── Inc/
│   │   └── mpu6050.h           # MPU6050 driver header
│   └── Src/
│       └── mpu6050.c           # MPU6050 driver implementation
├── Drivers/                    # STM32 HAL drivers
├── img/                        # Performance plots and results
└── README.md
```

## Getting Started

### Prerequisites
- STM32CubeIDE
- STM32F401RE Nucleo board
- MPU6050 breakout board
- Jumper wires

### Building the Project
1. Clone this repository
2. Open the project in STM32CubeIDE
3. Build the project (Ctrl+B)
4. Flash to the STM32 board
5. Open serial monitor (115200 baud) to view angle estimates


## Configuration

### Kalman Filter Tuning Parameters

```c
// In main.h or configuration file
#define KALMAN_P_INIT  0.01f      // Initial covariance
#define KALMAN_Q       0.005f     // Process noise (gyroscope uncertainty)
#define KALMAN_R       17.01f     // Measurement noise (accelerometer uncertainty)

// Timing parameters
#define KALMAN_PREDICT_PERIOD_MS  10    
#define KALMAN_UPDATE_PERIOD_MS   10    
```



### Parameter Tuning Guidelines

- **Increase Q (process noise)**: If the filter responds too slowly to changes
- **Decrease Q**: If the filter is too noisy or unstable
- **Increase R (measurement noise)**: If accelerometer readings are very noisy
- **Decrease R**: If you want stronger accelerometer influence

### MPU6050 Configuration
- **Accelerometer Range**: ±2g (most sensitive for gravity measurement)
- **Gyroscope Range**: ±250°/s (sufficient for most applications)
- **Digital Low Pass Filter**: Enabled for noise reduction
- **Sample Rate**: 1kHz internal, decimated as needed

## Future Improvements

### Extended Kalman Filter Enhancements
- [ ] Add gyroscope bias estimation to the state vector :
    The bias integration in the state space will drastically reduce the drift 
    and reduce its effect on the ekf output
- [ ] Implement adaptive noise covariance estimation:
    Adaptive algorithms can automatically tune Q and R matrices based on innovation sequences, improving filter performance across different motion scenarios and environmental conditions without manual parameter adjustment.


### System Optimizations
- [ ] Optimize matrix operations for fixed-point arithmetic
- [ ] Implement parallel processing for prediction and update steps
- [ ] Add automatic filter parameter tuning based on motion characteristics
- [ ] Integrate with STM32's hardware math accelerator units

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

- STMicroelectronics for the HAL drivers and development tools
- InvenSense for MPU6050 documentation and sensor specifications
- Digital Signal Processing theory and filter design references
- Extended Kalman Filter theory and implementation references
- Matrix operations library found in this repo:https://github.com/simondlevy/TinyEKF

---
