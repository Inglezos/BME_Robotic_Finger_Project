# BME Robotic Finger Project

## Overview
This project focuses on modeling and controlling a robotic finger. The code encompasses both forward and inverse kinematics, ensuring precise control and validation of the robotic finger's movements. Additionally, GUI files are included for enhanced user interaction.

## File Descriptions

### MATLAB Scripts and Functions

- **check_valid_angles.m**: Checks if the given joint angles are within the operational limits of the robotic finger. Performs validation check for all four angles, ensuring they are inside the valid ranges specified.

- **dynamics_torques.mlx**: Calculates the torques symbolic expressions, using the forward transformation matrix and the Jacobian.

- **forward_kinematics.mlx**: Computes the position and orientation of the robotic finger's end-effector given specific joint angles. Using a predetermined modified DH parameters table and a given transformation matrix between consecutive links, solves the forward kinematics problem for the 4DOF robotic finger for the three joints, for user-specified theta angles and phalanges lengths. Usually this script is used for verifying (sanity check) the results of the inverse kinematics script.

- **get_forward_transformation_matrix.m**: Returns as symbolic expression the complete transformation matrix for a 4DOF robotic finger, for a predetermined modified DH parameters table and a given general transformation matrix between two consecutive links.

- **get_jacobian.m**: Computes the Jacobian matrix, relating joint velocities to the end-effector's linear and angular velocities. Calculates and returns the jacobian matrix for the positions and angles of a 4DOF robotic finger's fingertip (end effector), which are the linear and angular velocities, depending on the passed argument 'option' ("linear" or "angular"). The positions of the fingertip are extracted from the total transformation matrix denotes as Q, which is passed as argument.
  
- **inverse_kinematics_equations.mlx**: Derives and solves the inverse kinematics equations to calculate the joint angles required to reach a desired position and orientation of the end-effector. This script solves the inverse kinematics problem for a 4DOF robotic finger and calculates the angles of the four joints, using the closed-form equations derived from geometric analysis, given the fingertip's end position (x, y, z) and orientation (angle).

- **velocities.mlx**: Given the transformation matrix (extracts position), this script forms the full Jacobian matrix and calculates the linear and angular velocities of the fingertip (of a 4DOF robotic finger), in symbolic expressions.


### GUI Development Files

- **GUI_forward_kinematics.m**: Solves the forward kinematics problem for given phalanges lengths and joints angles, returning to the GUI the end position of the fingertip. Uses a predetermined modified DH parameters table and the general
 transformation matrix between consecutive links.
  
- **GUI_inverse_kinematics.m**: Solves the inverse kinematics problem for given phalanges lengths, end effector's (fingertip) position and orientation (x, y, z, total angle with respect to x-axis in x-z plane) and returns to the GUI the 4 angles solutions, along with some additional information about their validity check, calling internally check_valid_angles().
  
- **GUI_get_all_positions.m**: Returns to the GUI the positions for all 3 joints (MCP, PIP, DIP) and for the end effector (fingertip), using a predetermined modified DH parameters table and the general transformation matrix and given as
 argument the lengths of the three phalanges and the angles that must be calculated first during inverse kinematics solution. The GUI uses this
 function to visualize explicitly each phalange in 3D plot.

- **check_valid_angles.m**: Called inside GUI_inverse_kinematics. Checks if the given joint angles are within the operational limits of the robotic finger. Performs validation check for all four angles, ensuring they are inside the valid ranges specified.

- **Robotic_Finger_GUI_App.mlapp**: The graphical user interface (GUI) application, created in Matlab App Designer. It is intuitive with a simplified 3D representation of the finger (joints and phalanges). The user can set values for the lengths of each phalange (L1, L2, L3) and can specify the desired end state of the fingertip (position x, y, z and angle ϵ) through dedicated text fields and sliders. The calculated values for the inverse kinematics for all four angles are displayed in the "Theta Values" field. These values afterwards can be transmitted to the Team B via the MQTT server. In case the end position is not valid (due to the limited phalanges lengths or due to out-of-range angles), a proper error pop-up message is displayed to the user.

- **Robotic_Finger_GUI_App.m**: The exported ".m" equivalent file of the GUI App ".mlapp" file.


## How to Use

0. **Prerequisites**:  
   Developed and tested on Matlab R2023b. The files must be in the same folder, especially the GUI files in order the GUI App to run.

1. **Clone the Repository**:  
   git clone https://github.com/Inglezos/BME_Robotic_Finger_Project.git  
   cd BME_Robotic_Finger_Project
