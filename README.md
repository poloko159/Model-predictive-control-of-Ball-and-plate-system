# Project Title

Model predictive control of Ball and plate system

## In this project:
I have used many tools such as ACADO, Matlab MPC toolbox, MPT. However, I make this project because I like the MPC and I want to understand how exactly MPC works, also I want to make it myself.

Motor controller:
* 3 stepper motors with driven by 3 Tb6560 modules and a STM32H7 nucleo (microstep mode: 1/16):
* The timers 1,2,3 used as PWM sources and master timers.
* The timers 4.5.8 used as slave timers which return the number of pulses generated by timer 1,2,3. 
* The slave timers can count up or down depended on the setup of the 4th bit of TIMx->CR1. 
* The number of pulses generated will equivalent to TIMx-> CNT.
* In my project, I used it instead of encoder for motors.
* The serial com used a array of 9 char -> 3 target angle which received from host computer.
* The STM32H7 Nucleo with control the motor using PID control.

Fast model predictive control for nonlinear system:

Fisrt, the system will be linearizated at the current points and generate 20 matrix of A, B in next 20 nodes:
*    using classic Runge-Kutta method (RK4) to calculate next 20 variables of X.
*    Calculate A(k), B(k), k=0,...,N-1
*    saved
Second: Caculate Gradient and Hessian Matrix: in the last project, I used a large number of variables: N * (Nx+Nu)+Nx, so the controller is slowed down and could not control the system. In this project, I used condensing N2 to reduce number of variables: N*Nu.
* From the result above ( A(k), B(k), X(k)) I calculated the Hessian and Gradient matrix with use the control variables u as the optimization variables.
* then, the qpoases used to solve the quadratic function with constrains
* last step: upload the u and apply the first u to system
The ball tracking with opencv is very simple so there is no discuss about it.

## Getting Started

Control of ball on plate using model predictive control with sequential quadratic programming and qpoases quadratic programming solver. 

### Prerequisites
Software:

* [Visual Studio](https://visualstudio.microsoft.com/downloads/) - Visual Studio 2017
* [qpOASES](https://projects.coin-or.org/qpOASES) - QP solver
* [Opencv 3.3.1](https://opencv.org/releases.html)- opencv
Hardware:

* [Stm32H7 Nucleo](https://www.st.com/en/evaluation-tools/nucleo-h743zi.html) - the Motor controller
* [TB6560](https://www.google.com/search?q=TB6560&rlz=1C1CHBF_enVN806VN806&oq=TB6560&aqs=chrome..69i57j69i60j69i59l3.2159j0j4&sourceid=chrome&ie=UTF-8)
* [Webcam a4tech PK 720G](https://www.google.com/search?q=webcam+a4tech+PK+720G&rlz=1C1CHBF_enVN806VN806&oq=webcam+a4tech+&aqs=chrome.1.69i59l3.16687j0j1&sourceid=chrome&ie=UTF-8)


## Running the tests

In my full project, I will use a GUI made by QT tool in visual studio 2017 with this project to display and control system

The tests are controlling the ball which has the trajectories:

* Rectangle
* Circle
* Stable at the known point
* As many as I can make
## Deployment

Add additional notes about how to deploy this on a live system

## Built With

* Visual studio 2017
* Opencv 331
* STM32H7 Nucleo
* Logitech Webcam webcam a4tech PK 720G ( will buy another if I have better budget)
* Stepper motor, ...

## Contributing
In the future, I will write a paper about my project.

## Versioning

This project is made from my old projects (deleted). This is the ... I don't know, may be 26th.

## Authors

* **Poloko** - *Initial work* - [Poloko](https://github.com/poloko159)

Just the nickname, I will post my information later

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details

Just the copy script in the internet, just look awesome ))

## Acknowledgments

* Thank to Google, Wiki, Github, ..., I can make this project.
* In the future, I will make a Quadcopter which using MPC
* Special thanks to the many lecture videos, books and paper about MPC
