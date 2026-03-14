# ReBel Cobot Inverse Kinematics Solver

A Python implementation of forward and inverse kinematics for a 4-DOF (Degree of Freedom) collaborative robotic arm using Denavit-Hartenberg parameters.

## Overview

This project solves both forward kinematics (FK) and inverse kinematics (IK) problems for the ReBel Cobot arm. Given a target end-effector position and orientation, the inverse kinematics solver computes the required joint angles. The forward kinematics transformation verifies the solution.

## Project Structure

```
├── main.py           # Main script with FK computation and IK verification
├── ik.py            # Inverse kinematics solver implementation
└── README.md        # This file
```

## Features

- **Forward Kinematics (FK)**: Computes the end-effector position and orientation given joint angles using Denavit-Hartenberg transformation matrices
- **Inverse Kinematics (IK)**: Solves for joint angles given a target end-effector position (x, y, z) and orientation (theta)
- **Symbolic Computation**: Uses SymPy for symbolic transformation matrix generation
- **Numerical Verification**: Validates IK solutions by computing FK with the resulting joint angles

## Requirements

```
numpy
sympy
```

Install dependencies with:
```bash
pip install numpy sympy
```

## Robot Parameters

The ReBel Cobot is a 4-DOF arm with the following link parameters (in mm):

| Link | Length |
|------|--------|
| h (base height) | 252.2 |
| l1 (shoulder-elbow) | 241.519 |
| l2 (elbow-wrist) | 246.8 |
| l3 (wrist-end effector) | 62 |

## Usage

### Basic Example

```python
python main.py
```

### Input Specification

Modify the `state_input` in `main.py` to specify your target:

```python
state_input = np.array([x, y, z, theta])
# x, y, z: End-effector position in mm
# theta: End-effector orientation in radians
```

### Example

```python
state_input = np.array([5, 450, 262, -np.pi/2])
# Target: Position (5, 450, 262) mm with -90° orientation
```

### Output

The script outputs:
1. Forward Kinematics transformation matrix
2. Input target position and orientation
3. Computed joint angles (in degrees)
4. Verification of end-effector position using FK

## Algorithm Details

### Forward Kinematics

The FK transformation is computed using the Denavit-Hartenberg (DH) convention:

```
T = T0→1 × T1→2 × T2→3 × T3→4
```

Where each ``Ti → i+1`` is constructed from DH parameters: a (link length), α (link twist), d (link offset), θ (joint angle).

### Inverse Kinematics

The IK solver uses geometric analysis:

1. **First joint (t1)**: Solved using the XY-plane projection
2. **Third joint (t3)**: Derived from the distance equation in the XZ plane using the law of cosines
3. **Second joint (t2)**: Calculated from the constraint that the arm must reach the target point
4. **Fourth joint (t4)**: Ensures the end-effector maintains the desired orientation

## Example Output

```
Input Target Position (XYZ) and Orientation:
[5, 450, 262] mm, Theta: -90.0 degrees

Joint Angles (degrees):
[0.01, 30.45, 25.30, -65.75]

Output End Effector Position:
[5, 450, 262]
```

## References

- **Denavit-Hartenberg Convention**: Standard method for modeling serial robotic manipulators
- **Inverse Kinematics**: Geometric approach based on analytical solution of joint constraints

## Notes

- Joint angles are returned in radians; use `np.degrees()` for degree conversion
- The IK solution assumes the arm workspace contains the target position
- Singularities may occur at certain configurations where analytical IK cannot find a solution

## Author

ReBel Cobot Project

## License

[Specify your license here]
