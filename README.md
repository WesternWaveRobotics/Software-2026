# Software-2026

<details>
  <summary>Table of Contents</summary>
  <ul>
    <li><a href="#development-setup">Development Setup</a></li>
    <li><a href="#hardware-structure">Hardware Structure</a></li>
    <li><a href="#software-structure">Software Structure</a></li>
    <li><a href="#roadmap">Roadmap</a></li>
    <li><a href="#acknowledgments">Acknowledgments</a></li>
  </ul>
</details>

## MATE ROV 2026 - Control Software

This repository contains the full software stack for our MATE ROV 2026 competition ROV. It includes:
- Serial communication between topside and ROV
- Motor control with ardunio
- **W.I.P:** Image recognition with [Ultralytics YOLO26](https://docs.ultralytics.com/models/yolo26/)
- TO BE CONTINUED...

## Development Setup
Reccomended installation and setup for development.

### 1. Clone the repository:
```
git clone https://github.com/Western-Wave-Robotics/Software-2026.git
cd Software-2026
```

### 2. Create and activate a virtual environment:
```ps
python -m venv .venv
.venv/Scripts/activate
```
### 3. Install the pyproject inc. dev dependencies:
```ps
python -m pip install -e ".[dev]"
```
## Hardware Structure
Work in progress...

## Software Structure
```
Software-2026/                <- Project root
|
├── models/                   <- ML models (img rec)
├── src/              
|   ├── ardunio_control/      <- Onboard arduino control
|   ├── utils/                
|   ├── controller_math.py    <- Thrust calculation + scaling
|   ├── main_window.py        <- Main GUI thread
|   └── workers.py            <- Background worker threads
|
└── run.py                    <- Project entry point                    
```

## Roadmap
- [x] **ROV Control:** Controller math, thruster esc scaling, controller value outputs, etc.
- [ ] **UI Polish:** Refine UI, live video feed display, debug console.
- [ ] **Task 2.1:** Implement image recognition using Ultralytics YOLO26.

## Acknowledgments
- [PythonGUIs](https://www.pythonguis.com/) - amazing free PyQt6 introduction (as well as many others)