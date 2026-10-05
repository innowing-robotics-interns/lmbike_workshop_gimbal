# Bike workshop

This repository is the workshop package. It has two parts: a MuJoCo bicycle simulation, and the firmware for a phone gimbal. Clone it, then open the folder you are working on.

The clone command is the same on Windows, macOS, and Linux. Install Git first, using the section for your system. The download lands in a new folder named `lmbike_workshop_gimbal`.

```bash
git clone https://github.com/innowing-robotics-interns/lmbike_workshop_gimbal.git
cd lmbike_workshop_gimbal
```

If this machine already has a GitHub SSH key, use this address instead:

```bash
git clone git@github.com:innowing-robotics-interns/lmbike_workshop_gimbal.git
```

## Windows

1. Install Git for Windows from [https://git-scm.com/download/win](https://git-scm.com/download/win). The installer can keep the default options.
2. Close any terminal that was already open, then open PowerShell or Git Bash so `git` is on the path.
3. Move to the folder where you want the project (for example `cd $HOME\Documents`), then run the clone commands above.

## macOS

1. Open Terminal.
2. Run `git --version`. If Git is missing, macOS offers the Xcode command-line tools. Install those, then run `git --version` again.
3. `cd` to the folder where you want the project, then run the clone commands above.

## Linux

1. Install Git from your package manager.

   Debian or Ubuntu:

   ```bash
   sudo apt update
   sudo apt install git
   ```

   Fedora:

   ```bash
   sudo dnf install git
   ```

2. `cd` to the folder where you want the project, then run the clone commands above.

## What the two folders contain

### `bike_mujoco_sim`

A MuJoCo simulation of a bicycle that has to stay upright. The lessons are Jupyter notebooks in `tutorials/`, from setup (`00_setup_and_install.ipynb`) through PID, tuning, and driving the bike by hand. `sim/` is the Python controller and the simulation loop. `models/` and `configs/` are the bicycle models and the gain settings those lessons load.

After the clone, start with [bike_mujoco_sim/HOW_TO_RUN_NOTEBOOK.md](bike_mujoco_sim/HOW_TO_RUN_NOTEBOOK.md) and `bike_mujoco_sim/tutorials/00_setup_and_install.ipynb`.

### `gimbal`

Firmware for the phone gimbal, in `gimbal/servo_test`. It is an STM32CubeIDE project for an STM32F103. An IMU reports roll and pitch. Two servos on a PCA9685 tilt the phone back toward level. The control law is a PID in `gimbal/servo_test/Core/Src/gimbal_control.c`. That is the file to edit. The drivers stay behind the hardware API.

The function reference is [gimbal/servo_test/CONTROL_API.md](gimbal/servo_test/CONTROL_API.md). Open `gimbal/servo_test` in STM32CubeIDE to build and flash.
