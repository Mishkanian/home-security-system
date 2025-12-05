# Home Security System  
Author: **Michael Mishkanian**  

---

## Overview  
This project implements a simple home security system using FreeRTOS on the STM32L475E-IOT01A Discovery Board. The system has three states: **Disarmed**, **Armed**, and **Alarm**. Motion detected while in Armed mode triggers a countdown before entering Alarm mode. The system state is also available over WiFi through a small web interface.


## Features

### RTOS Functionality  
- Multiple FreeRTOS tasks for state control, motion detection, WiFi communication, and LED behavior  
- Message queues for passing sensor events  
- Thread flags for signaling  
- Semaphores for protecting shared data  
- Software timers for LED blinking, countdown, and PWM control of the buzzer  

### State Machine  
States:  
- **Disarmed:** System idle, button press arms system  
- **Armed:** Motion detection active  
- **Alarm:** LEDs blink, buzzer via PWM timer  

Motion from the accelerometer triggers the countdown before Alarm.

### Hardware Interaction  
- Accelerometer (BSP driver) for detecting shake  
- User button interrupt processed in a deferred task  
- Onboard LEDs for status feedback  
- WiFi module hosting a simple web interface  

### WiFi Interface  
The web page allows users to:  
- View current system state  
- Enter disarm code  
- Disarm system remotely  

When using an iPhone hotspot, **Maximize Compatibility** must be enabled to allow the board to connect.


## Technologies Used  
- **Board:** STM32L475E-IOT01A  
- **IDE:** STM32CubeIDE  
- **RTOS:** FreeRTOS  
- **Libraries:** accelero.h, stm32l475e_iot01.h, queue.h, semphr.h, task.h, timers.h, event_groups.h  
- **Serial Debugging:** PuTTY at 115200 baud  
- **Host System:** MacBook Air M2, macOS 14.0  


## Architecture

### Tasks  
- **State Task:** Controls state machine  
- **Timer Task:** Sets up timers for LEDs and buzzer  
- **Motion Task:** Reads accelerometer and sends events  
- **WiFi Task:** Communicates with mobile devices  
- **Button Task:** Handles pushbutton interrupt events  

### Timers  
- Software timer for LED blinking in Alarm  
- PWM timer for buzzer  
- Countdown timer for code entry timeout  

### Communication  
- **Message Queue:** Sensor events to the state task  
- **Thread Flags:** Fast event signaling  
- **Semaphore:** Protects shared variables  


## LED Behavior  
- **DISARMED:** Green LED off  
- **ARMED:** Green LED solid  
- **ALARM:** Red LED blinking  


## Memory Usage  
- **Total memory used by this application:** 49216 bytes  
- **Max task stack usage (WiFi Task):** 4096 bytes


## Challenges and Solutions  
- **Thread creation failure:** Fixed by increasing FreeRTOS heap to 40000 bytes  
- **LED timer interference from WiFi module:** Fixed by raising FreeRTOS timer task priority  
- **Hotspot connection issues:** Required enabling Maximize Compatibility  
- **Task freezes due to stack limits:** Solved by increasing task stack sizes  


## References  
- ECEN 501 course materials by Dr. Radhika Grover  
- STM32L4 HAL documentation (UM1884)  

