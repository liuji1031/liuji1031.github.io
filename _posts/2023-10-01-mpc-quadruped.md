# Model Predictive Control for Quadruped Robots

## Overview
Model Predictive Control (MPC) is a powerful control technique that optimizes the input to the plant or system being controlled, based on an explicit model of the system and some desired outcome or trajectory[1]. At each iteration, MPC simulates the behavior of the system over a finite horizon $H$ given the control input $u_{0:H-1}$ over the same period. The sequence of the control input is optimized according to a predefined loss function given the desired behavior of the system, e.g., tracking of the desired trajectory. After the optimization is completed, the first control input of the sequence of control input is applied to the system, while the rest is discarded. At the next iteration, the optimization is performed again given the new feedback information. Therefore, MPC uses the predicted behavior of the system to guide the search of the optimal control, and thus the name. This method has been extensively applied in the field of robotics due to its flexibility to incorporate for example constraints on the system state and control variables.

MPC has been applied to control mobile robots, including legged robots such as quadrupeds. Compared to other mobile robots, legged robots especially quadrupeds have several advantages. First, they provide a higher maneuverability and traversability over non-flat and other complex terrains [2]. Their bio-inspired gait also proves to be more energy efficient on uneven terrain. Therefore, legged robots are particularly suitable for applications such as off-road payload carrying, rescue mission, etc., as these tasks might require accessing regions hard to reach for conventional wheel robots. However, controlling a legged robot's locomotion can be really challenging given the high degree of freedom (DOF) of the robot as well as the unknown contact dynamics of the surface being traversed on. Therefore, controlling legged robot requires more advanced control technique and MPC is suitable for the task, due to its flexibility in the problem formulation as well as the constant replanning during the deployment that allows the robot to quickly adapt to the dynamic changing environment. Applying MPC to legged robots such as quadruped has witnessed quite a few successful examples over recent years, and this tutorial aims to provide an entrypoint into the general framework of MPC in quadrupeds. 

## Problem Formulation
In general, the MPC aims to solve the following optimal control problem:

$$
\begin{alignat}{3}
&\!\min_{u_{0:H-1}} &\qquad& p(x_H) + \sum_{t=0}^{H-1} q(x_t, u_t)\\
&\text{s.t.} &      & x_{t+1} = g(x_t, u_t), t \in T=\{0,1,...,H-1\} \\
&            &      & h(x_t, u_t)\leq0, t \in T \\
&            &      & x_0 = x(0) \\
&            &      & x_H \in X \\
\end{alignat}
$$

In this discrete formulation of the MPC problem, $x_t$ is the state variable, while $u_t$ is the control variable. In the cost function, the $p(x_H)$ term represents the terminal cost, while the $q(x_t, u_t)$ term represents the stage cost at each time point. $x_{t+1} = g(x_t, u_t)$ specifies the system dynamics, and $h(x_t, u_t)\leq0$ specifies the inequality constraints the state and control varialbes are subject to. $x_0 = x(0)$ specifies the initial state while $x_H \in X$ specifies the desired region $X$ the terminal state $x_H$ needs to be in at $t=H$ [3]. Together, the optimization problem searches for the optimal control over the horizon $H$ such that the state $x_t$ follows the system dynamics, while obeying the boundary conditions and the inequality constraints imposed on the state and control variable. 

Specifically for quadruped robots, the usual formulation 

Reference: <br />
[1]https://www.mathworks.com/help/mpc/gs/what-is-mpc.html <br />
[2]Development of quadruped walking robots A review <br />
[3]https://optimization.cbe.cornell.edu/index.php?title=Model_predictive_control
