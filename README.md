![1408Hyper 2024-25 repo banner](https://raw.githubusercontent.com/1408Hyper/creative-assets/refs/heads/main/logos/final/SWED/2025CodeBannerDarkBgOnly1.PNG)

<center><h1> ⚡<em><strong>1408H</em></strong>yper @ VRC 2025-26</center></h1>

[![Github Actions Makefile CI](https://github.com/helloworld3200/1408Hyper-2024VRC-Code/actions/workflows/c-cpp.yml/badge.svg)](https://github.com/helloworld3200/1408Hyper-2024VRC-Code/actions/workflows/c-cpp.yml)

> 1408H's source code for the 2025-26 season of VRC.

### Sponsored by [Teehee Dental Works](https://teehee.sg/)

**2025Code** was designed with portability in mind - we understand that situations can quickly change at any competition.  

Our code is hot-swappable, making heavy use of **abstract classes** and **templates**
to allow for us to rapidly change and test different pieces of code when time is of the essence.

Additionally, our code features a [**custom built CI/CD testing suite via GitHub Actions.**](https://github.com/helloworld3200/1408Hyper-2024VRC-Code/actions/workflows/c-cpp.yml) - the only purpose-made one that exists in the entire world at the time of writing.

## Primary Features

- **Custom-built & tuned PID Controller (integrated with IMU, IME and dedicated odometry encoders)** for autonomous routines
- **Multi-mode operator drivetrain control** with easy switching between Arcade, Tank, Curved Arcade including built-in **sinusodial sensitivity smoothing curves**
- **Component-based class system (together with a plethora of base abstract classes)** also provides easier programming of newly engineered subsystems
- *More coming soon further into the season!*

## Chain of Command

_This list does not contain all roles, only those most relevant to programming._

1. **Principal _Software Engineer_** > [helloworld3200](https://github.com/helloworld3200)
2. **Senior _Software Engineer_** > [krishma2](https://github.com/krishma2)
3. **Assistant _Software Engineer_** > [Aadi-L](https://github.com/Aadi-L)

-  **Principal _Mechanical Engineer_** > [1408H-Builder](https://github.com/1408H-Builder)
- **Senior _Mechanical Engineer_** > [YuvrajVerma09](https://github.com/YuvrajVerma09)

## Included Libraries

- Created with [`PROS` API](https://github.com/purduesigbots/pros)
as this has better documentation than the official VEX API.
- Outside of `PROS` and the C++ standard library, our code uses **no external libraries** to write our code - all features are **custom-built** entirely *in-house*.

## Project Structure

To run Make, first install [ARM G++](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads).

- **Main file** at `src/main.cpp`.
- **Includes** in `include/main.h`.
- **Options _(e.g. motor ports)_** in `include/options.h`
- **Legacy/unused code _(e.g. old LiftMech)_** in `legacy/` directory