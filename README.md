# HexapodMasters
## Usage
To launch simulation run simLaunch.py (On Linux run as sudo, the vscode jason file is set up to do this automaticially. This is due to using the keyboard library, this might be removed later).

### MuJoCo simulation viewer
The main viewer will open on the left side of the screen, press F1 for controls

### Control interface
The control interface will open to the right of the MuJoCo viewer with controls as follows.
- Left click around 2D hexapod visual to set target direction.
- Right click to set direction to stationary.
- Scroll mouse wheel to change target speed
- Space and Shift to increase/decrease target height

## Requirments
Note, as of now the automatic tiling of the MuJoCo viewer and Control Interface does not work on Windows OS. This is to be fixed.

Python3 is the base requirment for this project to function. Also make sure to update pip.
```
python3 -m pip install --upgrade pip
```
---
[MuJoCo](https://github.com/deepmind/mujoco) is used for physics simulation.
```
python3 -m pip install mujoco
```
---
[RayLib](https://www.raylib.com/) python bindings are used for rendering the control interface.
```
python3 -m pip install raylib
```
---
[Python StateMachine](https://github.com/fgmacedo/python-statemachine) is used for handeling state machines
```
python3 -m pip install python-statemachine
```
or if you want to be able to automaticially generate state machine diagrams
```
python3 -m pip install python-statemachine[diagrams]
```
---
[Quaternion](https://github.com/moble/quaternion) is used for handeling quatrenion calculations.
```
python3 -m pip install --upgrade --force-reinstall numpy-quaternion
```
---
[Screeninfo](https://github.com/rr-/screeninfo) is used for retreiving info about the screen, used for automaticially tiling windows.
```
python3 -m pip install screeninfo
```

## Possible References
- A Fast Voxel Traversal Algorithm for Ray Tracing: http://www.cse.yorku.ca/~amana/research/grid.pdf