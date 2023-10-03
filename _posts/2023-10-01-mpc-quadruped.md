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

For quadruped robots, the equation of motion or the dynamics model can be derived using the Lagrangian method given the generalized coordinate $q$

$$
D(q)\ddot{q}+C(q,\dot{q})\dot{q}+G(q)=J_M^T\tau-J_F^TF
$$

The choice of the generalized coordinate $q$, which is a vector, can include variables such as joint angles and the pose of the base of the robot (translation and orientation expressed in Euler angles or unit quaternion). On the left-hand side (LHS), $D(q)$ is the inertial matrix, $C(q,\dot{q})$ represents the Coriolis and centrifugal forces, and $G(q)$ represents the effect of the gravitational forces. On the right-hand side (RHS), $\tau$ and $F$ represent the joint torques and the ground reaction force repectively, while $J_M$ and $J_F$ are Jacobian matrices projecting $\tau$ and $F$ onto the space of $q$ [4]. For a typical quadruped robot, a single leg has 3 DOF, i.e., two DOF at the hip joint and 1 DOF at the elbow. The pose of the base of the robot can be represented as a vector in $\mathbb{R}^{6\times1}$ if Eular angles are used to present the orientation. Thus, generalized coordinate $q$ of the whole body dynamics has a dimension of $3\times4+6=18$. The high dimensionality of the equation as well as the intrinsic nonlinearity makes the MPC formulation difficult to tackle. As a result, often simplifications are made in terms of the dynamics equation.

The first simplification often made is the assumption that the mass of the leg is neglectable compared to the mass of the base of the robot [5], which is often true for most present-day quadruped robot, e.g., MIT cheetah [6], despite that for biological organisms the leg mass could be important for generating the bending at the spine for more efficient galloping [5]. Nevetheless, neglecting the leg mass allows the formulation to focus solely on the base of the robot. A second common simplification makes the assumption that that contacts made between the leg and the ground are point contacts, and as such only forces but not torques can be generated at these contact points [6]. The description of the base's orientational dynamics can be further simplified if one further assumes small pitch and roll angles [6]. Together, it can be shown as in [6] that the system can be modeled as a linear time-varying system, provided that the yaw angle and foot placement is known ahead of time as a part of planning:

$$
\dot{x}(t)=A(\psi)x(t)+B(r_1,r_2,...,r_n,\psi)u(t)
$$

where $x$ is the state variable of the base's position and orientation, $\psi$ is the yaw angle of the robot, $r_i$ is the ith leg's endpoint position, and $u(t)$ is the input to the system including the ground forces at each leg and the gravitational force. 

Given this simplified model, the MPC problem can be defined as follows:

$$
\begin{alignat}{4}
&\!\min_{u,x} &\qquad& \sum_{t=0}^{H-1} \lVert x_{t+1}-x_{t+1, ref} \rVert _{Q_t} + \lVert u_t \rVert _{R_t} \\
&\text{s.t.} &      & x_{t+1} = A_tx_t + B_tu_t, t \in T=\{0,1,...,H-1\} \\
&            &      & \underbar c_t \leq C_t u_t \leq \bar c_t, t \in T \\
&            &      & D_tu_t=0, t \in T \\ 
&            &      & x_0 = x(0) \\
\end{alignat}
$$

Here, $x_{t, ref}$ is the reference trajectory, and $Q_t$ and $R_t$ are diagonal positive semidefinite matrices for calculating the weighted norm of the error between the state $x(t)$ and the reference trajectory and the regularization on the control signal $u(t)$. The inequality constraint $ \underbar c_t \leq C_t u_t \leq \bar c_t$ represents the constraint that the ground reaction force must lie within the friction cone, and thus is realizable. Finally, matrix $D_t$ is a selection matrix enforcing the swing leg, which is not in contact with the ground, to have 0 ground reaction force. 

The above MPC problem is thus formulated to search for the optimal ground reaction force based on the planned leg placement and the robot yaw dynamics, such that the actual state, i.e., the base's position and orientation, tracks the reference trajectory as close as possible. The solution, i.e., the optimal ground reaction force, is then mapped to the joint torque commands through each leg's Jacobian matrix. 

## Solving MPC
Several methods exist for solving MPC problems. For example, Dynamic Programing (DP) can be used to solve a linear quadratic MPC problem with a linear time invariant (LTI) system [7]. The idea of the method is to recursively optimize over control input and the state variable backward in time such that it is solving a nested optimization problem. Suppose we start at the end of the horizon, i.e., $t=H$, and we optimize over $x(H)$ and the input $u(H-1)$. Then the original optimization problem becomes only a function of $x$ and $u$ up to $t=H-1$, and we can again apply the same approach to optimize over $x(H-1)$ and $u(H-2)$. Therefore, by moving recursively back in time, DP is able to solve the linear quadratic MPC with a LTI system. 

However, for the time variant system as formulated in the above section, it is possible to solve the problem using Quadratic Programming (QP)



Reference: <br />
[1]https://www.mathworks.com/help/mpc/gs/what-is-mpc.html <br />
[2]Biswal, P., & Mohanty, P. K. (2021). Development of quadruped walking robots: A review. Ain Shams Engineering Journal, 12(2), 2017-2031. <br />
[3]https://optimization.cbe.cornell.edu/index.php?title=Model_predictive_control <br />
[4]De Santos, P. G., Garcia, E., & Estremera, J. (2006). Quadrupedal locomotion: an introduction to the control of four-legged robots (Vol. 1). London: springer. <br />
[5]Parra Ricaurte, E. A., Pareja, J., Dominguez, S., & Rossi, C. L. A. U. D. I. O. (2022). Comparison of leg dynamic models for quadrupedal robots with compliant backbone. Scientific Reports, 12(1), 14579. <br />
[6]Di Carlo, J., Wensing, P. M., Katz, B., Bledt, G., & Kim, S. (2018, October). Dynamic locomotion in the mit cheetah 3 through convex model-predictive control. In 2018 IEEE/RSJ international conference on intelligent robots and systems (IROS) (pp. 1-9). IEEE. <br />
[7]Garcia, C. E., Prett, D. M., & Morari, M. (1989). Model predictive control: Theory and practice—A survey. Automatica, 25(3), 335-348. <br />
