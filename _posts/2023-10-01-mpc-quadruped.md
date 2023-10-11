# Model Predictive Control for Quadruped Robots

## Overview
Model Predictive Control (MPC) is a powerful control technique that optimizes the input to the plant or system being controlled, based on an explicit model of the system and some desired outcome or trajectory[1]. At each iteration, MPC simulates the behavior of the system over a finite horizon $H$ given the control input $u_{0:H-1}$ over the same period. The sequence of the control input is optimized according to a predefined loss function given the desired behavior of the system, e.g., tracking of the desired trajectory. After the optimization is completed, the first control input of the sequence of control input is applied to the system, while the rest is discarded. At the next iteration, the optimization is performed again given the new feedback information. Therefore, MPC uses the predicted behavior of the system to guide the search for the optimal control, and thus the name. This method has been extensively applied in the field of robotics due to its flexibility to incorporate for example constraints on the system state and control variables.

MPC has been applied to control mobile robots, including legged robots such as quadrupeds. Compared to other mobile robots, legged robots especially quadrupeds have several advantages. First, they provide higher maneuverability and traversability over non-flat and other complex terrains [2]. Their bio-inspired gait also proves to be more energy efficient on uneven terrain. Therefore, legged robots are particularly suitable for applications such as off-road payload carrying, rescue missions, etc., as these tasks might require accessing regions hard to reach for conventional wheel robots. However, controlling a legged robot's locomotion can be challenging given the high degree of freedom (DOF) of the robot as well as the unknown contact dynamics of the surface being traversed on. Therefore, controlling a legged robot requires a more advanced control technique and MPC is suitable for the task, due to its flexibility in the problem formulation as well as the constant replanning during the deployment that allows the robot to quickly adapt to the dynamic changing environment. Applying MPC to legged robots such as quadruped has witnessed quite a few successful examples over recent years, and this tutorial aims to provide an entry point into the general framework of MPC in quadrupeds. 

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

In this discrete formulation of the MPC problem, $x_t$ is the state variable, while $u_t$ is the control variable. In the cost function, the $p(x_H)$ term represents the terminal cost, while the $q(x_t, u_t)$ term represents the stage cost at each time point. $x_{t+1} = g(x_t, u_t)$ specifies the system dynamics, and $h(x_t, u_t)\leq0$ specifies the inequality constraints the state and control variables are subject to. $x_0 = x(0)$ specifies the initial state while $x_H \in X$ specifies the desired region $X$ the terminal state $x_H$ needs to be in at $t=H$ [3]. Together, the optimization problem searches for the optimal control over the horizon $H$ such that the state $x_t$ follows the system dynamics while obeying the boundary conditions and the inequality constraints imposed on the state and control variable. 

For quadruped robots, the equation of motion or the dynamics model can be derived using the Lagrangian method given the generalized coordinate $q$

$$
D(q)\ddot{q}+C(q,\dot{q})\dot{q}+G(q)=J_M^T\tau-J_F^TF
$$

The choice of the generalized coordinate $q$, which is a vector, can include variables such as joint angles and the pose of the base of the robot (translation and orientation expressed in Euler angles or unit quaternion). On the left-hand side (LHS), $D(q)$ is the inertial matrix, $C(q,\dot{q})$ represents the Coriolis and centrifugal forces, and $G(q)$ represents the effect of the gravitational forces. On the right-hand side (RHS), $\tau$ and $F$ represent the joint torques and the ground reaction force respectively, while $J_M$ and $J_F$ are Jacobian matrices projecting $\tau$ and $F$ onto the space of $q$ [4]. For a typical quadruped robot, a single leg has 3 DOFs, i.e., two DOFs at the hip joint and 1 DOF at the elbow. The pose of the base of the robot can be represented as a vector in $\mathbb{R}^{6\times1}$ if Euler angles are used to present the orientation. Thus, generalized coordinate $q$ of the whole body dynamics has a dimension of $3\times4+6=18$. The high dimensionality of the equation as well as the intrinsic nonlinearity makes the MPC formulation difficult to tackle. As a result, often simplifications are made in terms of the dynamics equation.

The first simplification often made is the assumption that the mass of the leg is neglectable compared to the mass of the base of the robot [5], which is often true for most present-day quadruped robots, e.g., MIT cheetah [6], despite that for biological organisms the leg mass could be important for generating the bending at the spine for more efficient galloping [5]. Nevertheless, neglecting the leg mass allows the formulation to focus solely on the base of the robot. A second common simplification assumes that that contacts made between the leg and the ground are point contacts, and as such only forces but not torques can be generated at these contact points [6]. The description of the base's orientational dynamics can be further simplified if one assumes small pitch and roll angles [6]. Together, it was shown in [6] that the system can be modeled as a linear time-varying system, provided that the yaw angle and foot placement are known ahead of time as a part of planning:

$$
\dot{x}(t)=A(\psi)x(t)+B(r_1,r_2,...,r_n,\psi)u(t)
$$

where $x$ is the state variable of the base's position and orientation, $\psi$ is the yaw angle of the robot, $r_i$ is the ith leg's endpoint position, and $u(t)$ is the input to the system including the ground forces at each leg and the gravitational force. 

Given this simplified model, the MPC problem can be defined as follows:

$$
\begin{alignat}{3}
&\!\min_{u,x} &\qquad& \sum_{t=0}^{H-1} \lVert x_{t+1}-x_{t+1, ref} \rVert _{Q_t} + \lVert u_t \rVert _{R_t} \\
&\text{s.t.} &      & x_{t+1} = A_tx_t + B_tu_t, t \in T=\{0,1,...,H-1\} \\
&            &      & \underline c_t \leq C_t u_t \leq \bar c_t, t \in T \\
&            &      & D_tu_t=0, t \in T \\ 
&            &      & x_0 = x(0) \\
\end{alignat}
$$

Here, $x_{t, ref}$ is the reference trajectory, and $Q_t$ and $R_t$ are diagonal positive semidefinite matrices for calculating the weighted norm of the error between the state $x(t)$ and the reference trajectory and the regularization on the control signal $u(t)$. The inequality constraint $ \underline c_t \leq C_t u_t \leq \bar c_t$ represents the constraint that the ground reaction force must lie within the friction cone and thus is realizable. Finally, matrix $D_t$ is a selection matrix enforcing the swing leg, which is not in contact with the ground, to have 0 ground reaction force. 

The above MPC problem is thus formulated to search for the optimal ground reaction force based on the planned leg placement and the robot yaw dynamics, such that the actual state, i.e., the base's position and orientation, tracks the reference trajectory as closely as possible. The solution, i.e., the optimal ground reaction force, is then mapped to the joint torque commands through each leg's Jacobian matrix. 

## Solving MPC
Several methods exist for solving MPC problems. For example, Dynamic Programming (DP) can be used to solve a linear quadratic MPC problem with a linear time-invariant (LTI) system [7]. The idea of the method is to recursively optimize over control input and the state variable backward in time such that it is solving a nested optimization problem. Suppose we start at the end of the horizon, i.e., $t=H$, and we optimize over $x(H)$ and the input $u(H-1)$. Then the original optimization problem becomes only a function of $x$ and $u$ up to $t=H-1$, and we can again apply the same approach to optimize over $x(H-1)$ and $u(H-2)$. Therefore, by moving recursively back in time (**Ricatti recursion**), DP can solve the linear quadratic MPC with an LTI system.

However, for the time-variant system as formulated in the above section, it is possible to solve the problem using **Quadratic Programming** (QP) through a technique called **condensing** [7]. Using the state transition equation in discrete time, the state variables can be eliminated from the optimization problem, and given the quadratic form of the objective function, the optimization can be directly performed over the sequence of control input using QP. If the constraints are also convex, the global optimal solution is guaranteed. 

Another approach, in contrast to condensing, is to keep the state variables, and the optimization is performed on both the state variables and the control input. This approach could prove to be more computationally efficient as it can exploit the sparsity structure in the matrices for solving the optimization problem [7]. A more recent study showed that it is possible to use the condensed approach but also exploit the sparsity structure through a change of variables [8].

For example, in [6], using the simplified dynamics model that only focuses on the center of mass of the robot, the authors were able to derive a linear MPC formulation that they could solve with a QP solver. With the predefined desired foot placement, gait, and centroid trajectory, the authors were able to show a wide range of possible behaviors on the Cheetah robot, including trotting, jumping, galloping, climbing stairs, etc. The MPC controller was able to make new predictions at a frequency between 25 and 50 Hz, showing the efficiency of the QP formulation [6].

For nonlinear MPC problems, two common numerical methods include **Sequential Quadratic Programming** (SQP) and **Interior Point** (IP) method. For a general nonlinear optimization problem of the form:

$$
\begin{alignat}{3}
&\!\min_{x} & F(x)\\
&\text{s.t.} &      G(x)=0 \\
&            &      H(x) \leq 0 
\end{alignat}
$$

the local optimal solution $x^*$ has to meet the Karush-Kuhn-Tucker (KKT) condition. To iteratively solve the KKT conditions, one approach is to first linearize the conditions, and doing so will result in a linear complementarity system that can be thought of as the KKT condition for a QP, and the solution can be found globally. This general approach is the SQP method [17]. The IP method sequentially approximates the original problem by replacing the nonsmooth KKT condition with a smooth one. The solution found in each iteration is in the interior of the set defined by the inequality constraints, and thus the name [17]. By sequentially tightening the relaxation, the solution approaches the actual solution of the original nonlinear problem. These numerical methods are commonly used, especially for whole-body MPC formulations. 

## Connection to Decision Making in Robotics
The MPC in quadruped robots is mainly focused on motion control of the robot, and in general, it searches for optimal control and optimal trajectory given the cost function. Thus, MPC can be thought of as making decisions at two levels simultaneously, with one corresponding to the low-level control input, e.g., joint torques, and the other corresponding to the dynamics of the robot, e.g., the trajectory of the robot torso. For example, in [6], the MPC searches for the optimal ground reaction force which is then mapped to the joint torques through leg Jacobian matrices, such that the robot torso or base follows the desired reference trajectory. Thus, at each time step, the MPC makes decisions with regard to the decision variables in the objective function subject to the constraints, and the flexibility of the MPC formulation allows decisions to be made across different hierarchical levels, tailored to the particular application. 

In class, our discussion revolves around reinforcement learning and the learned policy is the equivalent of a controller for the robot. In contrast to MPC, during deployment, the optimal policy determines the optimal action only based on the current state of the robot, while the optimality is obtained through extensive training. For example, the policy derived from value functions, i.e., $V(s_t)$ or $Q(s_t, a_t)$, the value functions essentially summarize the future expected reward without explicitly considering all potential trajectories. MPC, on the other hand, relies on the dynamics model to search for the optimal actions and thus considers the future trajectories at each time step during actual deployment. 


## Variants of MPC formulation
The above formulations rely most importantly on the simplification of the full body dynamics to focus on the center-of-mass dynamics, which allows fast online planning. These methods still require the user to design the gait, contact timing, etc for the robot. On the other hand, it is possible to utilize whole-body dynamics for MPC formulation, which can plan for both contact points and end effector trajectory. 

For example, in [9], the authors formulate the MPC using the whole-body dynamics model of the robot, and the optimization problem is formulated in search of the optimal joint trajectories in the continuous time domain to ensure that the contact and other constraints are properly respected. To solve the infinite programming (IP), the authors used B-spline to parameterize joint trajectories such that the optimization is converted to a semi-infinite programming problem. To deal with the continuous constraints, the authors used Taylor series expansion to approximate these constraints. Finally, the authors used predefined motion and contact sequences that constitute several phases for motion planning, and the optimization is performed for each phase separately. Due to the high computation demand, the optimization was performed offline. The authors were able to demonstrate behaviors such as sitting on a chair, kicking a ball, walking under constraints, etc.

In contrast, in [10], the authors avoided formulating separate optimization formulations over different contact modes by using the **complementary constraints**. This is motivated both by how rigid body contact problems are handled in the computer graphics field and by the fact that the robot the authors focused on was FastRunner, whose design involves many kinematic constraints that make contact mode scheduling intractable. Specifically, the complementary constraints are of the following form:

$$
\phi(q) \geq 0 \\
\lambda \geq 0 \\
\phi(q)^T \lambda = 0
$$

where $q$ is the joint angles, $\phi(q)$ represents the distance from the end effector to the contact surface, and $\lambda$ is the contact force. The condition $\phi(q) \geq 0$ imposes the impenetrability constraint, while the condition $\phi(q)^T \lambda = 0$ imposes that contact force can be non-zero only when contact distance is 0 and vice versa. Using the complementary constraint, the optimization problem is formulated as a Mathematical Program with Complementarity Constraints (MPCC) and solved using Sequential Quadratic Programming (SQP). The authors used this method to find the optimal sequence of contact mode for running in the FastRunner robot. 

A third variant of the formulation considers both the dynamics of the robot as a single rigid body and the kinematics of the legs at the same time [11]. Thus, this approach considers the whole-body **kino-dynamics** model and unlike in [6] where only the robot base trajectory and contact force are optimized, [11] incorporates the joint velocities in the optimization, and thus achieves a single-task formulation MPC, which does not require predefined gait which might depend on heuristics. This formulation is necessary especially since the authors were focusing on a quadruped robot where each leg is equipped with an actuated wheel, and by directly considering the forward kinematics of the legs, the rolling constraints of the wheels become tractable. Together, this formulation allows the authors to optimize under different contact modes through the same set of parameters, and the MPC problem was solved through a differential dynamic programming based algorithm. 

The variants of MPC formulation presented in this section are by no means exhaustive, but they represent approaches that try to capture more realistic and accurate modeling of the robot, which allows more dynamic motions of the robot and at the same time reduces the need for human-engineered gait sequence, contact time, etc. However, as these formulations become more complex, solving them is more computationally demanding especially for real-time applications.  

## Open Research Questions

As mentioned in the above sections, the whole-body dynamics formulation is computationally challenging due to the high dimensionality of the state space and the nonlinearity in the dynamics. It is therefore important to improve the efficiency of computing rigid body dynamics [12]. Recent progress in this field has shown significant improvement in that regard. For example, [13] developed a proximal formulation of the constrained dynamics, which shows computational improvement by more than a factor of 2 compared to existing methods. The new method is also implemented in an open-source software package [14]. These recent improvements might be critical for bringing whole-body dynamics MPC to real-time applications. 

Other open research questions involve closed-loop stability, robustness against disturbance, and model mismatch [12]. Specifically, the MPC needs to be robust against model uncertainty and measurement noise, while maintaining stability against external disturbances. These issues fall under the realm of **Robust MPC** (RMPC) [15], which explicitly models the uncertainty in the modeling and measurement. A recent study [16] used a hierarchical control structure where at the highest level, an RMPC is used for trajectory planning of the reduced-order model, while a low-level nonlinear controller is used to map the trajectory to the full-order order. The authors further employed a deep learning reinforcement learning framework to compute the uncertainties for the high-level RMPC algorithm [16]. Together, the authors showed robust performance over different terrain types, and the underactuated DOFs, e.g., roll and pitch, remained stable.    


Reference: <br />
[1]https://www.mathworks.com/help/mpc/gs/what-is-mpc.html <br />
[2]Biswal, P., & Mohanty, P. K. (2021). Development of quadruped walking robots: A review. Ain Shams Engineering Journal, 12(2), 2017-2031. <br />
[3]https://optimization.cbe.cornell.edu/index.php?title=Model_predictive_control <br />
[4]De Santos, P. G., Garcia, E., & Estremera, J. (2006). Quadrupedal locomotion: an introduction to the control of four-legged robots (Vol. 1). London: springer. <br />
[5]Parra Ricaurte, E. A., Pareja, J., Dominguez, S., & Rossi, C. L. A. U. D. I. O. (2022). Comparison of leg dynamic models for quadrupedal robots with compliant backbone. Scientific Reports, 12(1), 14579. <br />
[6]Di Carlo, J., Wensing, P. M., Katz, B., Bledt, G., & Kim, S. (2018, October). Dynamic locomotion in the mit cheetah 3 through convex model-predictive control. In 2018 IEEE/RSJ international conference on intelligent robots and systems (IROS) (pp. 1-9). IEEE. <br />
[7]Garcia, C. E., Prett, D. M., & Morari, M. (1989). Model predictive control: Theory and practice—A survey. Automatica, 25(3), 335-348. <br />
[8]Jerez, J. L., Kerrigan, E. C., & Constantinides, G. A. (2011, December). A condensed and sparse QP formulation for predictive control. In 2011 50th IEEE Conference on Decision and Control and European Control Conference (pp. 5217-5222). IEEE. <br />
[9]Lengagne, S., Vaillant, J., Yoshida, E., & Kheddar, A. (2013). Generation of whole-body optimal dynamic multi-contact motions. The International Journal of Robotics Research, 32(9-10), 1104-1119. <br />
[10]Posa, M., Cantu, C., & Tedrake, R. (2014). A direct method for trajectory optimization of rigid bodies through contact. The International Journal of Robotics Research, 33(1), 69-81. <br />
[11]Bjelonic, M., Grandia, R., Harley, O., Galliard, C., Zimmermann, S., & Hutter, M. (2021, September). Whole-body mpc and online gait sequence generation for wheeled-legged robots. In 2021 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS) (pp. 8388-8395). IEEE. <br /> 
[12]Katayama, S., Murooka, M., & Tazaki, Y. (2023). Model predictive control of legged and humanoid robots: models and algorithms. Advanced Robotics, 37(5), 298-315. <br />
[13]Carpentier, J., Budhiraja, R., & Mansard, N. (2021, July). Proximal and sparse resolution of constrained dynamic equations. In Robotics: Science and Systems 2021. <br />
[14]Carpentier, J., Saurel, G., Buondonno, G., Mirabel, J., Lamiraux, F., Stasse, O., & Mansard, N. (2019, January). The Pinocchio C++ library: A fast and flexible implementation of rigid body dynamics algorithms and their analytical derivatives. In 2019 IEEE/SICE International Symposium on System Integration (SII) (pp. 614-619). IEEE. <br />
[15]Bemporad, A., & Morari, M. (2007). Robust model predictive control: A survey. In Robustness in identification and control (pp. 207-226). London: Springer London. <br />
[16]Pandala, A., Fawcett, R. T., Rosolia, U., Ames, A. D., & Hamed, K. A. (2022). Robust predictive control for quadrupedal locomotion: Learning to close the gap between reduced-and full-order models. IEEE Robotics and Automation Letters, 7(3), 6622-6629. <br />
[17]Diehl, M., Ferreau, H. J., & Haverbeke, N. (2009). Efficient numerical methods for nonlinear MPC and moving horizon estimation. Nonlinear model predictive control: towards new challenging applications, 391-417. <br />
