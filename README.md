Inverted Pendulum Control System

Overview

This repository contains the code for controlling an inverted pendulum using an Arduino Due. The system utilizes a stepper motor as the actuator and an optical encoder to sense the pendulum's angular displacement. The control system uses a Linear Quadratic Regulator (LQR) to stabilize the pendulum, with noise filtering achieved through a low-pass filter combined with a moving average.

Files and Directories

"Controller.ino": The main Arduino file containing the setup and loop functions.

"Controller.h": The header file declaring the functions and variables used in the control system.

"Controller.cpp": The implementation file where the control logic and functions are defined.
