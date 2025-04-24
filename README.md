# SURYA SANDEEP AKELLA
## 1. DYNAMIC LOAD BALANCER FOR SATELLITE MICROGRID SYSTEMS:

## Problem Statement

In multi-load systems, power distribution can become inefficient or unstable when load demand fluctuates. The objective was to design a dynamic load balancing system that could actively monitor load conditions and adjust power distribution in real time to improve efficiency and reliability.

---

## My Contributions

### System Design
- Designed a block-level architecture including sensing, control, and switching stages.
- Implemented a feedback loop using current sensors to monitor real-time load behavior.
- Integrated a microcontroller to process sensor data and control switching dynamically.

### Schematic Design
- Developed full schematics using Altium 365. 
- Selected and integrated components including MOSFETs, current sensors INA3221A, battery monitoring sensor (BQ27441) and voltage regulators.
- Included ESD protection and filtering circuits for signal integrity and robustness.

### PCB Design and DRC
- Designed and routed a 4-layer PCB with proper grounding and power distribution strategies.
- Completed and passed Design Rule Checks (DRC) and Electrical Rule Checks (ERC).
- Focused on minimizing EMI and thermal issues through layout optimization.

### Board Bring-Up
- Assembled and soldered components using standard reflow and hand-soldering techniques.
- Verified voltage rails, signal paths, and component functionality using multimeter and oscilloscope.
- Debugged firmware-hardware integration during initial power-up.

### Testing and Validation
- Performed load switching tests under varying load scenarios. 
- Validated dynamic response time and stability during rapid load transitions.
- Collected and analyzed current and voltage data to confirm correct system behavior.

### Embedded Connectivity & Cloud Integration
- Configured Wi-Fi connectivity on the SAMW25 MCU for IoT communication.
- Established MQTT-based messaging with a local and remote broker for real-time sensor updates and control signals.
- Set up a Node-RED dashboard to visualize system performance, power flow, and diagnostics.
- Implemented over-the-air firmware update (FOTA) support via Wi-Fi for remote maintenance and debugging.
---

## Tools and Technologies Used

-  Altium (Schematic & PCB Design)
-  SAMW25 
- FreeRTOS 
- Oscilloscope, Multimeter, Bench Power Supply
- I2C, SPI, UART communication protocols


## 2. Pipelined 5-Stage RISC-V Processor (RV32IM)

A custom-designed, 32-bit RISC-V processor implemented in SystemVerilog with a fully functional 5-stage pipeline. This processor supports RV32I and RV32M instruction sets, excluding division and multiplication instructions for this stage of development.

---

## Overview

The goal of this project was to design a clean and efficient RV32I-compatible processor pipeline that maximizes instruction throughput while handling data hazards, control flow, and basic memory operations. The design was built entirely in SystemVerilog and verified using simulation testbenches.

---

## My Contributions

### Processor Architecture and Design
- Designed and implemented a **5-stage pipelined datapath** with the following stages:
  - Instruction Fetch (IF)
  - Instruction Decode (ID)
  - Execute (EX)
  - Memory Access (MEM)
  - Writeback (WB)

- Built the **Arithmetic Logic Unit (ALU)** to handle operations including add, sub, and, or, xor, sll, srl, sra, and comparisons.

- Excluded multiplication and division support at this stage to focus on verifying core pipelining, forwarding, and hazard detection.

### Pipeline Control and Hazard Handling
- Implemented data hazard resolution through **data forwarding (bypassing)**.
- Developed simple **stalling and flushing logic** for load-use hazards and branch mispredictions.
- Created control logic for **branch instructions and jump handling** with early branch resolution.

### Verification and Simulation
- Used testbenches and provided instruction trace files to verify correctness and pipeline behavior.
- Ran extensive simulations to validate control flow, hazard resolution, and ALU instruction correctness.
- Visualized pipeline timing and trace outputs for debugging and performance observation.

---

## Tools and Technologies

- SystemVerilog (Processor Design)
- Verilator Simulator
- RISC-V Instruction Set Specification (RV32I subset)
- Git and VS Code for version control and development

---
## 3. MimicArm – 4DOF Robotic Arm with Gesture Control

A gesture-controlled 4 Degree-of-Freedom robotic arm developed using the ATmega328PB microcontroller and inertial sensors. The project enables real-time motion mapping from a controller to the arm using I2C, PWM, and ADC-based sensing.

---

## Project Overview

The goal of this project was to create a low-cost, responsive robotic arm capable of mimicking human hand gestures in real-time. It is controlled via wired communication, using inertial data from an MPU6050 IMU and a flex sensor to drive multiple servo motors for arm and claw motion.

---

## My Contributions

### System Architecture and Hardware Design
- Designed the control system using an **ATmega328PB microcontroller** with **bare-metal programming**.
- Integrated **MPU6050 (IMU)** via I2C to capture hand orientation and motion data.
- Used a **flex sensor** to measure finger bending and control claw grip.
- Configured **PWM outputs** to drive four servo motors controlling base rotation, shoulder, elbow, and claw.

### Firmware Development
- Wrote the full firmware in C using **bare-metal programming**, avoiding Arduino libraries to improve performance and control.
- Implemented **ADC-based sensing** for flex input and mapped analog values to servo angles.
- Parsed and smoothed accelerometer and gyroscope data for reliable gesture recognition.
- Calibrated sensor thresholds to distinguish between fine and coarse movements for precise control.

### Testing and Calibration
- Tuned PID-like behavior in servo movement to avoid jitter and overshoot.
- Verified end-to-end system accuracy with real-time testing of motion replication.
- Optimized servo response to different gesture speeds and angles for smooth operation.

---

## Tools and Technologies

- ATmega328PB Microcontroller
- MPU6050 (Accelerometer + Gyroscope) via I2C
- Flex Sensor via ADC
- Servo Motors (PWM Control)
- Bare-Metal C Programming
- Oscilloscope, DMM, Serial Debugging Tools

---

## 4. Pong Game with Wireless Control using ATmega328PB and ESP32

A fully functional Pong game implemented on an ATmega328PB microcontroller with real-time LCD rendering, joystick input, and wireless control using ESP32 and the Blynk IoT platform. The game logic, display updates, and communication were all developed using bare-metal C.

---

## Project Overview

This project replicates the classic Pong arcade game in an embedded environment. The game features real-time paddle movement via joystick (Player 2) and remote wireless control (Player 1) over Wi-Fi using an ESP32 module and Blynk. The game runs on bare-metal firmware and communicates via SPI and GPIO interfaces.

---

## My Contributions

### Embedded Game Design
- Designed and implemented full **game logic** including paddle movement, ball bounce, edge collision, and scoring.
- Used **bare-metal programming** on the ATmega328PB to ensure tight control of game timing and performance.
- Rendered game visuals (paddles, ball, score) on an **SPI-connected LCD screen**.

### Hardware Interface & Peripherals
- Used **joystick (ADC)** to control Player 2's paddle position with smooth analog input.
- Configured **PWM and GPIO** to drive a buzzer for game event sounds (goal, bounce).
- Designed simple game menu and screen refresh control using optimized SPI writes.

### Wireless Control Integration
- Enabled **Player 1 paddle control wirelessly** using an **ESP32 connected via Wi-Fi to the Blynk platform**.
- Used a logic level shifter to interface 3.3V ESP32 signals with the 5V ATmega328PB safely.
- Transmitted paddle data over UART or GPIO between ESP32 and ATmega for real-time wireless input.

### Optimization and Testing
- Tuned game loop timing to maintain consistent frame rates and responsive paddle motion.
- Calibrated joystick ADC values for precision and accuracy.
- Validated gameplay smoothness and wireless latency under real conditions.

---

## Tools and Technologies

- ATmega328PB (Bare-Metal C, MPLABX)
- ESP32 (Wi-Fi Communication, Blynk Platform)
- LCD Display (SPI)
- Joystick (ADC)
- UART, SPI, PWM, GPIO
- Logic Level Shifting for MCU interfacing
- Oscilloscope, DMM, Serial Debugging


---

## 5. RowHammer Mitigation using PARA (Probabilistic Adjacent Row Activation)

This ongoing project explores the RowHammer vulnerability in DRAM and implements PARA (Probabilistic Adjacent Row Activation) as a mitigation strategy. The goal is to simulate and quantify the effectiveness of PARA in reducing bit flips during aggressive memory access, while evaluating performance trade-offs.

---

## Project Status

**Currently in progress**  
Actively working on simulation experiments, performance benchmarking, and mitigation strategy evaluation using open-source DRAM simulators.

---

## Objectives

- Understand the behavior of double-sided RowHammer attacks on DRAM.
- Integrate PARA (Probabilistic Adjacent Row Activation) into DRAM simulation tools.
- Quantitatively analyze mitigation impact on bit flip rates.
- Measure and visualize trade-offs in terms of latency and bandwidth overhead.

---

## Working:

### Simulation Setup
- Set up and configure DRAM simulators such as **DRAMSim2** and **Ramulator** for controlled RowHammer experiments.
- Create memory access patterns simulating **aggressive row activation** to trigger bit flips in adjacent rows.

### PARA Integration
- Implement **PARA logic** into the simulator: probabilistically refreshing adjacent rows based on activation frequency.
- Parameterize the refresh probability to observe trade-offs between mitigation strength and system performance.

### Evaluation and Analysis
- Collect and compare bit-flip statistics across baseline and PARA-mitigated runs.
- Measure latency and bandwidth impacts to assess mitigation overhead.
- Generate **heatmaps and visual reports** to highlight vulnerable regions and PARA effectiveness.

---

## Tools and Technologies

- DRAMSim2 / Ramulator
- C++, Python (for data analysis)
- Memory profiling scripts
- Graphing libraries for heatmap and performance plots (e.g., matplotlib, seaborn)

---



