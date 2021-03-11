# ESE-551-Linear-Dynamic-Systems
Code for ESE 551 Linear Dynamic Systems: Code is written in Matlab and Simulink.

## Table of Contents
* [State Observer Feedback](#State-Observer-Feedback)
  * [File Purposes](#File-Purposes)   

* [LQR](#LQR)
  * [File Purposes](#File-Purposes)   

## State Observer Feedback
The following files design and implement state observer feedback.
* controllerDesign.m  
* run_animation.m
* roll_dynamics_controller_simulink.slx
* airplane_icon.png
* sky.jpg

### File Purposes
* controllerDesign.m solves for the gain matrices of the state and observer using pole placement.
* roll_dynamics_controller_simulink.slx implements the state observer feedback controller with the gain matrices solved from controllerDesign.m with a simple dynamics model depicting the roll dynamics of an airplane.
* run_animation.m uses the data collected from the simulation and animates the roll. airplane_icon.png and sky.png are used as images in the animation.

## LQR
the following files uses LQR to design a state feedback controller and implements the controller on different models of a 2 rotor drone.
* Project2Code.m
* LQRnonlin.m
* nonlinearSys.m
* project2Linear.slx
* project2Nonlinear.slx

### File Purposes
* Project2Code.m solves for the gain matrix for stateback using pole placement and LQR. Then each controller is applied to a nonlinear dynamics model of a 2 rotor drone and a linearized version of the model. The step response for each case is plotted.
* LQRnonlin.m contains the nonlinear dynamics model with the controller solution solved through LQR.
* nonlinearSys.m contains the nonlinear dynamics model with the controller solution solved through pole placement
* project2Linear.slx and project2Nonlinear.slx are Simulinks that can simulate the closed loop feedback of a state feedback controller and linear and nonlinear dynamics model respectively. However, this already done directly in Project2Code.m


