2474X-CATS CompetitionBot — 2025–26 Push Back Season

Author: Jackson Scott (primary developer)
Game: VEX Robotics Competition – Push Back
Robot: Competition-ready V5 robot with modular architecture, command-based control, and advanced motion planning.

🚀 Overview

This repository contains the control software for Team 2474X-CATS’ 2025–26 VEX Push Back robot. The system is designed for robust autonomous routines, precise driver control, and modular subsystems, making it easy to scale, debug, and iterate during competition.

Key Features:

Command–Subsystem Architecture — separates hardware, behavior, and control logic.

Centralized Command Manager — organizes and schedules robot actions.

Subsystem Abstraction — drivebase, intake, indexer, matchloader, hooks.

Advanced Control Layer — feedforward, PID, and trajectory controllers for precise motion.

Telemetry Layer — centralized diagnostics and sensor reporting.

Utilities — helpers like Location for field positioning.

📁 File Structure
src/ 
├── main.cpp<br/>
├── commands.cpp / commands.h  # Command manager and registry <br/>
├── architecture/
│   ├── command.cpp / command.h  # Base command interface <br/>
│   ├── subsystem.cpp / subsystem.h  # Base subsystem interface <br/>
│   ├── robot.cpp / robot.h  # Central robot controller <br/>
│   ├── telemetry.cpp / telemetry.h  # Telemetry and diagnostics <br/>
├── utilities/ <br/>
│   └── location.cpp / location.h # Robot field position helper <br/>
├── subsystems/ <br/> 
│   ├── drivebase.cpp / drivebase.h <br/>
│   ├── intake.cpp / intake.h <br/>
│   ├── indexer.cpp / indexer.h <br/>
│   ├── matchloader.cpp / matchloader.h <br/>
│   └── hooks.cpp / hooks.h <br/>
└── control/ <br/>
    ├── feedforward.cpp / feedforward.h <br/>
    ├── pidcontroller.cpp / pidcontroller.h <br/>
    └── trajectorycontroller.cpp / trajectorycontroller.h <br/>

🏗 Architecture Overview

The software is built with modularity, reusability, and clarity in mind.

Core Concepts:

Subsystems — Encapsulate hardware and low-level control for each robot mechanism.

Commands — Represent discrete robot actions; scheduled by the Robot controller.

Robot — Orchestrates subsystems and commands; handles competition states.

Control Layer — Handles motion control using PID, feedforward, and trajectory tracking.

Telemetry — Provides diagnostics and sensor feedback.

Utilities — Provide helper functions like robot positioning.

Subsystems
Subsystem	Purpose
Drivebase	Robot drivetrain control
Intake	Acquire game objects
Indexer	Move blocks internally for scoring
Matchloader	Feed blocks into scoring position
Hooks	Endgame hooks for field interaction
Commands & Command Manager

Commands encapsulate high-level robot behaviors such as:

Drive to a location

Run intake

Score blocks

Deploy hooks

The command manager centralizes creation and scheduling, making autonomous sequences reusable and modular.

Control Layer

Feedforward — Predicts required motor output based on desired speed/acceleration.

PID Controller — Feedback loop to correct errors in motion.

Trajectory Controller

The robot uses a command–subsystem architecture with a centralized command manager and telemetry. Subsystems handle hardware, commands define actions, and the control layer ensures precise autonomous and driver performance.