# 24677-project-5-solved
**TO GET THIS SOLUTION VISIT:** [24677 Project 5 Solved](https://www.ankitcodinghub.com/product/24677-project-5-solved-2/)


---

📩 **If you need this solution or have special requests:** **Email:** ankitcoding@gmail.com  
📱 **WhatsApp:** +1 419 877 7882  
📄 **Get a quote instantly using this form:** [Ask Homework Questions](https://www.ankitcodinghub.com/services/ask-homework-questions/)

*We deliver fast, professional, and affordable academic help.*

---

<h2>Description</h2>



<div class="kk-star-ratings kksr-auto kksr-align-center kksr-valign-top" data-payload="{&quot;align&quot;:&quot;center&quot;,&quot;id&quot;:&quot;96320&quot;,&quot;slug&quot;:&quot;default&quot;,&quot;valign&quot;:&quot;top&quot;,&quot;ignore&quot;:&quot;&quot;,&quot;reference&quot;:&quot;auto&quot;,&quot;class&quot;:&quot;&quot;,&quot;count&quot;:&quot;1&quot;,&quot;legendonly&quot;:&quot;&quot;,&quot;readonly&quot;:&quot;&quot;,&quot;score&quot;:&quot;5&quot;,&quot;starsonly&quot;:&quot;&quot;,&quot;best&quot;:&quot;5&quot;,&quot;gap&quot;:&quot;4&quot;,&quot;greet&quot;:&quot;Rate this product&quot;,&quot;legend&quot;:&quot;5\/5 - (1 vote)&quot;,&quot;size&quot;:&quot;24&quot;,&quot;title&quot;:&quot;24677 Project 5 Solved&quot;,&quot;width&quot;:&quot;138&quot;,&quot;_legend&quot;:&quot;{score}\/{best} - ({count} {votes})&quot;,&quot;font_factor&quot;:&quot;1.25&quot;}">

<div class="kksr-stars">

<div class="kksr-stars-inactive">
            <div class="kksr-star" data-star="1" style="padding-right: 4px">


<div class="kksr-icon" style="width: 24px; height: 24px;"></div>
        </div>
            <div class="kksr-star" data-star="2" style="padding-right: 4px">


<div class="kksr-icon" style="width: 24px; height: 24px;"></div>
        </div>
            <div class="kksr-star" data-star="3" style="padding-right: 4px">


<div class="kksr-icon" style="width: 24px; height: 24px;"></div>
        </div>
            <div class="kksr-star" data-star="4" style="padding-right: 4px">


<div class="kksr-icon" style="width: 24px; height: 24px;"></div>
        </div>
            <div class="kksr-star" data-star="5" style="padding-right: 4px">


<div class="kksr-icon" style="width: 24px; height: 24px;"></div>
        </div>
    </div>

<div class="kksr-stars-active" style="width: 138px;">
            <div class="kksr-star" style="padding-right: 4px">


<div class="kksr-icon" style="width: 24px; height: 24px;"></div>
        </div>
            <div class="kksr-star" style="padding-right: 4px">


<div class="kksr-icon" style="width: 24px; height: 24px;"></div>
        </div>
            <div class="kksr-star" style="padding-right: 4px">


<div class="kksr-icon" style="width: 24px; height: 24px;"></div>
        </div>
            <div class="kksr-star" style="padding-right: 4px">


<div class="kksr-icon" style="width: 24px; height: 24px;"></div>
        </div>
            <div class="kksr-star" style="padding-right: 4px">


<div class="kksr-icon" style="width: 24px; height: 24px;"></div>
        </div>
    </div>
</div>


<div class="kksr-legend" style="font-size: 19.2px;">
            5/5 - (1 vote)    </div>
    </div>
<div class="page" title="Page 2">
<div class="layoutArea">
<div class="column">
1 Introduction

In the previous projects, we have learned how to apply modern control techniques to an Un- manned Ground Vehicle (UGV). In this project, we will step up the difficulty and investigate how to control an Unmanned Aerial Vehicle (UAV).

Quadrotor drones has risen in popularity in recent year for a variety of uses, such as aerial photography for topology and agriculture, search and rescue missions, and product delivery. Ever-increasing demands on safety and performance have necessitated the development of more and more sophisticate flight control systems. Unlike ground vehicle, these flying vehicles could be inherently unstable due to many sources of uncertainty such as actuator degradation, external disturbances, and complex unmodeled dynamics (blade flapping and asymmetric angular speed of propellers), which often results in a crash.

In this project, we will make use of adaptive control theory to augment a LQR-based MPC controller and test its effectiveness in the event of a single quadrotor motor of a DJITM Mavic 2 Pro experience a 50% pr higher percent lost of thrust during hovering. As before, the experiment will be performed in the Webots simulator.

This project is the last part of the course project series:

P5 (a) Design a baseline LQR controller to fly the quadrotor without any motor failure. (b) Design a MRAC to fly the quadrotor with 50% lost of thrust in one motor.

</div>
</div>
<div class="layoutArea">
<div class="column">
2

</div>
</div>
</div>
<div class="page" title="Page 3">
<div class="layoutArea">
<div class="column">
2 Model

We will use a popular model of the quadrotor as the control model. As shown in Figure 1, the inertial frame is fixed to the ground. The roll axis of the quadrotor is pointing forward w.r.t. to the drone body. The pitch axis is pointing to the left. The yaw axis is pointing upwards. We will present the simplified nonlinear dynamics model and linearized dynamics model in Section 2.1 and 2.2. The model parameters are defined in Table 1.

Figure 1: Coordinate systems

2.1 Nonlinear dynamics

The dynamics of quadrotor helicopters have been studied in detail by many researchers. Assuming the vehicle as a rigid-body and operating at low speed, the dynamics are given by:

</div>
</div>
<div class="layoutArea">
<div class="column">
3

</div>
</div>
</div>
<div class="page" title="Page 4">
<div class="layoutArea">
<div class="column">
x ̈ = (cos(φ) sin(θ) cos(ψ) + sin(φ) sin(ψ))U1 m

y ̈ = (cos(φ) sin(θ) sin(ψ) − sin(φ) cos(ψ))U1 m

z ̈ = −g + cos(φ) cos(ψ)U1 m

φ ̈ = θ ̇ ψ ̇ ( I y − I z ) − J R θ ̇ Ω R + 1 U 2 Ix Ix Ix

θ ̈ = φ ̇ ψ ̇ ( I z − I x ) − J R φ ̇ Ω R + 1 U 3 Iy Iy Iy

ψ ̈ = φ ̇ θ ̇ ( I x − I y ) + 1 U 4 Iz Iz

where x, y, and z are the positions of the center of mass in the inertial frame; φ, θ, and ψ are the Euler angles which describe the orientation of the body-fixed frame with respect to the inertial frame; m,Ix,Iy, and Iz are the mass and moments of inertial of the quadrotor respectively; and JR and ΩR are the moment of inertia and angular velocity(i.e. rotor speed) of the propeller blades. U1 is the collective thrust, U2, U3, and U4 are the roll, pitch and yaw torques generated by the four propellers.

2.2 Linear dynamics

Since the quadrotor typically operates very near the hover position, we can linearize the system by making small angle approximations, neglecting higher order terms and letting U1 = mg + ∆U1, which will result in the linear dynamics:

x ̈ = g θ y ̈ = − g φ

z ̈ = ∆ U 1 m

φ ̈ = I1 U2 x

θ ̈ = I1 U 3 y

ψ ̈ = I1 U 4 z

Although this linear system seems a bit over simplified than the nonlinear dynamics, it still captures the dominant features of the quadrotor, and is sufficient near the hover position. As expected, the roll, pitch and yaw input command moments about their respective axes

</div>
</div>
<div class="layoutArea">
<div class="column">
4

</div>
</div>
</div>
<div class="page" title="Page 5">
<div class="layoutArea">
<div class="column">
and the collective input commands acceleration in the positive z-direction. Accelerations in the x- and y-directions are achieved primarily through vectoring the collective thrust.

For the purposes of the controller design, we will use the linearized dynamics.

2.3 Physical Parameters

All relevant physical parameters are listed here. The definitions of d1x, d2x, d1y and d1y are showd in Figure 2.

Figure 2: Definition of d1x, d2x, d1y and d1y

</div>
</div>
<div class="layoutArea">
<div class="column">
5

</div>
</div>
</div>
<div class="page" title="Page 6">
<div class="layoutArea">
<div class="column">
Name

(φ, θ, ψ) 􏰋φ ̇, θ ̇, ψ ̇􏰌 m

</div>
<div class="column">
Description

</div>
<div class="column">
Table 1: Model parameters. Unit

</div>
<div class="column">
Value

State State 0.4

</div>
</div>
<table>
<tbody>
<tr>
<td>
<div class="layoutArea">
<div class="column">
(X,Y,Z)

</div>
</div>
</td>
<td>
<div class="layoutArea">
<div class="column">
Vehicle’s coordinates in the world frame

</div>
</div>
</td>
<td>
<div class="layoutArea">
<div class="column">
m

</div>
</div>
</td>
<td>
<div class="layoutArea">
<div class="column">
State

</div>
</div>
</td>
</tr>
<tr>
<td>
<div class="layoutArea">
<div class="column">
(x ̇, y ̇, z ̇)

</div>
</div>
</td>
<td>
<div class="layoutArea">
<div class="column">
Vehicle’s velocity along the direction of vehicle frame

</div>
</div>
</td>
<td>
<div class="layoutArea">
<div class="column">
m/s

</div>
</div>
</td>
<td>
<div class="layoutArea">
<div class="column">
State

</div>
</div>
</td>
</tr>
</tbody>
</table>
<div class="layoutArea">
<div class="column">
Body row, pitch, yaw angle Body row, pitch, yaw angle rate Vehicle mass

</div>
<div class="column">
rad rad/s kg

</div>
</div>
<table>
<tbody>
<tr>
<td>
<div class="layoutArea">
<div class="column">
d1x

</div>
</div>
</td>
<td>
<div class="layoutArea">
<div class="column">
Length in x-axis from front rotors to the center of mass

</div>
</div>
</td>
<td>
<div class="layoutArea">
<div class="column">
m

</div>
</div>
</td>
<td>
<div class="layoutArea">
<div class="column">
0.1122

</div>
</div>
</td>
</tr>
<tr>
<td>
<div class="layoutArea">
<div class="column">
d2x

</div>
</div>
</td>
<td>
<div class="layoutArea">
<div class="column">
Length in x-axis from rear rotors to the center of mass

</div>
</div>
</td>
<td>
<div class="layoutArea">
<div class="column">
m

</div>
</div>
</td>
<td>
<div class="layoutArea">
<div class="column">
0.1171

</div>
</div>
</td>
</tr>
<tr>
<td>
<div class="layoutArea">
<div class="column">
d1y

</div>
</div>
</td>
<td>
<div class="layoutArea">
<div class="column">
Length in y-axis from front rotors to the center of mass

</div>
</div>
</td>
<td>
<div class="layoutArea">
<div class="column">
m

</div>
</div>
</td>
<td>
<div class="layoutArea">
<div class="column">
0.1515

</div>
</div>
</td>
</tr>
<tr>
<td>
<div class="layoutArea">
<div class="column">
d2y

</div>
</div>
</td>
<td>
<div class="layoutArea">
<div class="column">
Length in x-axis from rear rotors to the center of mass

</div>
</div>
</td>
<td>
<div class="layoutArea">
<div class="column">
m

</div>
</div>
</td>
<td>
<div class="layoutArea">
<div class="column">
0.1280

</div>
</div>
</td>
</tr>
</tbody>
</table>
<div class="layoutArea">
<div class="column">
Ix Iy Iz ct cτ ∆T

</div>
<div class="column">
Moment of inertia around x-axis Moment of inertia around y-axis Moment of inertia around z-aixs thrust coefficient

torque coefficient Simulation timestep

</div>
<div class="column">
kg mˆ2 kg mˆ2 kg mˆ2 N

kg mˆ2 sec

</div>
<div class="column">
0.000913855 0.00236242 0.00279965 0.00026 5.2e-06

0.01

</div>
</div>
<div class="layoutArea">
<div class="column">
6

</div>
</div>
</div>
<div class="page" title="Page 7">
<div class="layoutArea">
<div class="column">
3 Controllers

In order to access the effectiveness of adaptive controller in the presence of model uncertainty, We will use a LQR controller as the baseline controller for comparison.

3.1 Problem statement

The primary function of the adaptive controller is to accommodate any uncertainties which may arise in the dynamics. We can write the equations of motion in section (2.2) along with these uncertainties as

x ̇p =Apxp +BpΛu (1)

where Bp ∈ Rnp×m is constant and known, Ap ∈ Rnp×np is constant and unknown, xp ∈ Rnp, u ∈ Rm, Λ ∈ Rm×m is an unknown diagonal positive definite matrix. The state xp = [x,y,z,φ,θ,ψ,x ̇,y ̇,z ̇,φ ̇,θ ̇,ψ ̇]. The control input u = [∆U1,U2,U3,U4]. The goal is to track a reference command r(t) ∈ Rm in the presence of the unknown Ap, and Λ. We define the system output as

yp = Cpxp

In the case of the quadrotor, the output states are x, y, z, ψ and m = 4. The output

tracking error is then given by

ey = yp − r

Augmenting (1) with the integrated output tracking error, 􏰍

eyI = eydt, e ̇yI =ey leads to the extended open loop dynamics

x ̇ =Ax+BΛu+Bcr (2) where x = [xTp eTyI ]T is the extended system state vector. The extended open-loop system

</div>
</div>
<div class="layoutArea">
<div class="column">
matrices are given by

A= C 0 B= 0 Bc= −I

</div>
</div>
<div class="layoutArea">
<div class="column">
􏰂Ap0np×m􏰃 􏰂Bp 􏰃 􏰂0np×m􏰃 p m×m m×m m×m

</div>
</div>
<div class="layoutArea">
<div class="column">
and the extended system output (notice that we abuse the notations here: the output y is different from the the y positions of the center of mass in the inertial frame)

y = [Cp 0m×m]x = Cx

</div>
</div>
<div class="layoutArea">
<div class="column">
7

</div>
</div>
</div>
<div class="page" title="Page 8">
<div class="layoutArea">
<div class="column">
3.2 Baseline LQR Controller

A baseline LQR controller

ubl = Kblx

can be designed for the system in (2) in the case where there is no model discrepancy, i.e., Λ = Im×m and Areal is taken at some nominal value A ̄ where actually Areal = A ̄ + A∗ and A∗ contains all the unmodeled terms (A ̄ is the A in section 3.1). The feedback gains Kbl can be selected using standard LQR design techniques.

3.3 Model Reference Adaptive Controller

The reference model used by MRAC is the closed loop system given by (2), again in the case of no uncertainty, along with the control input in Section 3.2

x ̇m =􏰉A ̄+BKbl􏰊xm +Bcr (3) 􏰐 􏰏􏰎 􏰑

Am We can design an adaptive control input

uad =KˆxTxt (4) such that the resulted system (5) can track the reference model in (3)

x ̇ =􏰋A+BΛKˆxT􏰌x+Bcr (5) The adaptive rate of the control gain is given by

Kˆ ̇T =−ΓxeTPB x

where Γ is a diagonal positive definite matrix, e = x − xm is the model tracking error, and P is the unique symmetric positive definite solution of the Lyapunov equation

A Tm P + P A m = − Q

where Q is also symmetric positive definite.

The augmented structure of the adaptive controller implies that in the nominal case,

i.e., the case with no parameter uncertainty, the overall system is equivalent to the baseline control. However, when failures or other uncertainties arise, the adaptive controller works to assist the baseline controller in maintaining stability and performance.

</div>
</div>
<div class="layoutArea">
<div class="column">
With the Lyapunov function candidate given by

T ̃T −1 ̃

</div>
</div>
<div class="layoutArea">
<div class="column">
V=ePe+Tr(θΓ θ)

where θ ̃ = θˆ−θ is the control gain estimation error, it can be shown [7] that the derivative

</div>
</div>
<div class="layoutArea">
<div class="column">
of the Lyapunov function candidate is given by

V ̇ = − e T P e ≤ 0

The system is globally asymptotically stable and the tracking error asymptotically con- verges to 0, that is

lim e(t) = 0 t→∞

</div>
</div>
<div class="layoutArea">
<div class="column">
8

</div>
</div>
</div>
<div class="page" title="Page 9">
<div class="layoutArea">
<div class="column">
4 P5: Problems

Exercise 1. (50 points) (all the files for this exercise are in ex1 folder) Design a discrete- time infinite horizon LQR controller for the quadrotor to track the command when there is no motor failure. You should complete the code in lqr controller.py. Attach the plot gen- erated by completing 60s simulation and report the error generated at the end of simulation in your write-up. If your error is lower than lower bar, you will receive full score. Otherwise your score will be calculated as (upper bar−your error)/(upper bar−lower bar)∗50 where lower bar is 0.3 and upper bar is 0.6. Note that the full score for this exercise is 50.

You are strongly recommended to go through the code base provided.

Some hints that may be useful:

<ul>
<li>The physical parameters are defined in the base controller.py.</li>
<li>Make sure to discretize your continuous state-space system with the provided discrete timestep (self.delT) prior to solving the ARE.</li>
<li>Using LQR requires designing the Q and R matrices. Q penalizes state variables, while R penalizes control input. Try to think about what form your Q and R matrices should take for good performance. To help you started the tuning, you can take a look at the Example code part of the comment in the code base.
<ul>
<li>– &nbsp;For Q, large values will restrict changes to the respective states, while smaller values will allow the states to easily change.</li>
<li>– &nbsp;Similarly, in R, larger values will restrict control input, while smaller values will allow the control input to vary more.</li>
<li>– &nbsp;Empirically, it is reasonable to set the entries in the Q and R to be 1
(max value of the corresponding state/input)2 in order to normalize the penalty weights.
</li>
</ul>
</li>
</ul>
</div>
</div>
<div class="layoutArea">
<div class="column">
9

</div>
</div>
</div>
<div class="page" title="Page 10">
<div class="layoutArea">
<div class="column">
Exercise 2.

1. (30 points) (all the files for this exercise are in ex2 folder) Design a MRAC to fly the quadrotor with 50% lost of thrust in one motor. The motor failure happens after 14s in simulation. You should complete the code in adaptive controller.py. Attach the plot generated by completing 60s simulation and report your percentage lost of thrust. Please refer to Appendix 1 for details on how we simulate motor failure. If your LQR and MRAC controller can stablize the system with 50% lost of thrust in one motor, you will get full score. Otherwise, you will get (your percentage lost of thrust)/50 ∗ 30 points.

To get the correct plot, you first need to copy your lqr controller.py from Exercise 1 to the folder of Exercise 2 (ex2/controllers/ex2 controller/). Run the code with LQR controller and obtain the saved data for LQR controller. Follow the instruction below to run either the LQR or adaptive controller again to obtain the data and the plot. Every time you change the percentage lost of thrust, you need to repeat this process.

In order to change the percentage lost of thrust, go to ex2 controller.py and change the value of the variable lossOfThust (line 24 of ex2 controller.py) to your preferred percentage lost of thrust. If you want to change which controller to use, edit line 27 of ex2 controller.py to either

customController = AdaptiveController(driver, lossOfThust) or

customController = LQRController(driver, lossOfThust)

2. (20 points) Find a percentage lost of thrust such that LQR fails (the drone will fall onto the floor) but MRAC is still able to maintain tracking. Report the percentage lost of thrust and attach the plot.

Tips and Warnings:

• This simulation may be substantially slower than previous project parts if you do not have a computer with a dedicated graphics card.

• Initialize the gain matrix KˆxT to your LQR gain. It will help you stabilize the system.

</div>
</div>
<div class="layoutArea">
<div class="column">
10

</div>
</div>
</div>
<div class="page" title="Page 11">
<div class="layoutArea">
<div class="column">
5 Appendix

5.1 Conversion from control input to rotor speed

For our quadrotor model in webots, the propeller turns the motor angular velocity into a thrust and a (resistant) torque. The resultant thrust T of each motor is given by the product of thrust constants ct and the square of the angular velocity, where as the resultant torque τ is given by the product of torque coefficient cτ and the the square of the angular velocity.

T = ctω2 τ = cτ ω2

The control input U1 is simply the sum of the thrust of all propellers, and the row, pitch torque U2, U3 are the sum of torque generated by the opposite propellers around certain axes. The yaw torque U4 is the sum of the torque generated by all the propellers around the z-aixs.

</div>
</div>
<div class="layoutArea">
<div class="column">
Hence, we can write the conversion matrix H from rotor speed Ω = [ω12

</div>
<div class="column">
ω2 ω32

</div>
<div class="column">
ω42]T to

</div>
</div>
<div class="layoutArea">
<div class="column">
U =[U1

</div>
<div class="column">
U2 U3

</div>
<div class="column">
U4]T as:

</div>
</div>
<div class="layoutArea">
<div class="column">
1111 U=HΩ=ct 0 −L 0 LΩ

</div>
</div>
<div class="layoutArea">
<div class="column">
−L 0 L 0 −cτ cτ −cτ cτ

ct ct ct ct

</div>
</div>
<div class="layoutArea">
<div class="column">
Inversely, to get the desired motor speeds from control input, we use the inverse of H matrix.

U =HΩ→Ω=H−1U. 5.2 Simulating Actuator failure

To simulate around 50% lost of thrust in motor number 1, the control input to be used for the simulation Ufail is given by:

</div>
</div>
<div class="layoutArea">
<div class="column">
0.7 0 0 0 Ufail=H0 1 0 0H−1U

</div>
</div>
<div class="layoutArea">
<div class="column">
 0 0 1 0  0001

</div>
</div>
<div class="layoutArea">
<div class="column">
11

</div>
</div>
</div>
<div class="page" title="Page 12">
<div class="layoutArea">
<div class="column">
6 Reference

[1] Dydek, Zachary T., Anuradha M. Annaswamy, and Eugene Lavretsky. ”Adaptive control of quadrotor UAVs: A design trade study with flight evaluations.” IEEE Transactions on control systems technology 21.4 (2012): 1400-1406.

</div>
</div>
<div class="layoutArea">
<div class="column">
12

</div>
</div>
</div>
