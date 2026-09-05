# State-Space Control

## About the team

We are **Team 6560, the Charging Champions**, a student robotics team based in Irvine, California,
participating in the FIRST Robotics Competition (FRC). Our work brings together mechanical design,
electronics, and programming to build and operate a competition robot. Software connects these
parts: it interprets sensor measurements, controls motors, and coordinates the robot's actions.

## Project overview

This project explores how mathematical models and sensor feedback can help our robot move more
accurately and perform coordinated tasks more reliably. In simple terms, state-space control
describes what a mechanism is doing now, such as its position and velocity, and uses a model of its
motion to calculate how the motors should respond. For example, an arm approaching its target
quickly needs to slow down before it reaches that target.

We want to study these methods in simulation, compare them with our existing controllers, and
evaluate them on physical mechanisms. The broader goal is to connect the mathematics with practical
robot software: estimating motion from imperfect measurements, responding to changing conditions,
and coordinating several mechanisms during a task. The sections below describe our planned
approach; they are a roadmap, rather than a claim that every capability is already implemented.

The project is also an opportunity for students to learn how physics, mathematics, and computer
science work together in a real engineering system.

## Technical approach

This repository describes Team 6560's plan to study state-space control and determine where it can
improve the accuracy, stability, and coordination of an FRC robot. Our goal is not to replace every
Proportional-Integral-Derivative (PID) controller with a more complicated method. Instead, we plan
to model each mechanism, compare model-based control against our existing methods, and use
state-space control only when the additional information produces a measurable improvement.

## Motivation and objective

A complex robot may need to drive, move an arm, rotate a turret, spin a flywheel, and operate an
intake during the same task. These mechanisms do not always finish at predictable times because
their response depends on starting position, battery voltage, friction, game-piece load, and motion
of the rest of the robot. As a result, an autonomous sequence based only on fixed delays can either
continue before a mechanism is stable or wait longer than necessary.

We plan to separate this problem into three layers. State estimation will determine the current
condition of the robot from its sensors. Local controllers will move each mechanism toward a defined
position or velocity. Finally, a command-based coordinator will run independent motions in parallel
and continue the sequence only after the necessary states remain within defined tolerances. With
this structure, state-space control handles the physical behavior of each mechanism, while the
command scheduler still determines the task that the robot performs.

## State-space model

The **state** is the minimum collection of variables required to predict the future behavior of a
system. For a rotating arm, we can represent the state as

```math
\mathbf{x}=
\begin{bmatrix}
\theta \\
\dot{\theta}
\end{bmatrix},
```

where $\theta$ is the arm angle in radians and $\dot{\theta}$ is its angular velocity in radians per
second. The motor voltage is the input $\mathbf{u}$, and the encoder or other sensor readings form
the measured output $\mathbf{y}$.

For software operating at discrete time steps, the system is written as

```math
\mathbf{x}_{k+1}=A\mathbf{x}_k+B\mathbf{u}_k
```

```math
\mathbf{y}_k=C\mathbf{x}_k+D\mathbf{u}_k.
```

The matrix $A$ describes how the state changes naturally during one control-loop period, while $B$
describes how the motor voltage changes that state. The matrix $C$ maps the internal state to the
quantities measured by the sensors, and $D$ represents any immediate effect of the input on the
measurement. For an FRC robot with a typical 20 ms loop period, the discrete matrices predict the
state of the mechanism one loop into the future.

This model requires physical information such as motor characteristics, gear ratio, moving mass,
moment of inertia, and mechanism geometry. We can initially calculate these values from the design,
but theoretical estimates will not represent friction, mechanical compliance, or changes made
during construction. Therefore, we plan to characterize the completed mechanism with WPILib SysId
and replace uncertain estimates with measured system constants before evaluating the controller.

## State feedback and LQR

State feedback calculates motor voltage using both the desired state and the estimated current
state:

```math
\mathbf{u}=-K(\hat{\mathbf{x}}-\mathbf{r})+\mathbf{u}_{ff}.
```

Here, $\hat{\mathbf{x}}$ is the estimated state, $\mathbf{r}$ is the reference state, $K$ is the
feedback-gain matrix, and $\mathbf{u}_{ff}$ is the feedforward voltage predicted by the model. For
instance, an arm that is below its target but already moving upward quickly should receive a
different voltage from an arm at the same angle with zero velocity. State feedback includes this
difference directly because angle and angular velocity are separate parts of the state.

We plan to calculate $K$ with a Linear Quadratic Regulator (LQR). LQR selects the feedback gains by
minimizing the cost

```math
J=\sum_k\left[(\mathbf{x}_k-\mathbf{r}_k)^TQ(\mathbf{x}_k-\mathbf{r}_k)
+\mathbf{u}_k^TR\mathbf{u}_k\right].
```

The matrix $Q$ penalizes error in selected states, while $R$ penalizes the required actuator effort.
Increasing the state penalty generally produces a faster and more aggressive response; increasing
the input penalty generally reduces voltage use and produces a slower response. These matrices do
not remove the need for testing. Instead, they give us a consistent way to state which errors matter
and compare the resulting response with the same voltage and safety limits.

## State estimation

Not every state can be measured accurately. Encoder position is usually reliable over a short time,
but encoder-derived velocity can be noisy, and drivetrain odometry accumulates error as the wheels
slip. An observer combines the predicted state with the difference between the predicted and actual
sensor measurements:

```math
\hat{\mathbf{x}}_{k+1}=A\hat{\mathbf{x}}_k+B\mathbf{u}_k
+L(\mathbf{y}_k-C\hat{\mathbf{x}}_k).
```

The matrix $L$ determines how strongly the estimate responds to new measurements. A Kalman filter
calculates this correction from the expected process uncertainty and sensor noise. Based on this
model, reliable measurements receive more influence than measurements with high uncertainty.

For drivetrain localization, we plan to combine swerve-module odometry and gyro measurements with
field-position measurements from a Limelight observing AprilTags. Odometry provides frequent and
smooth short-term updates, while AprilTags correct the position error that accumulates over time.
The vision measurement must use the timestamp at which the image was captured so that camera
latency does not apply an old position to the robot's current state. Furthermore, distant,
high-ambiguity, or physically inconsistent measurements should be rejected or assigned greater
uncertainty instead of being treated as equally accurate.

## Linear-algebra and control background

To understand and modify these controllers, we need the following mathematical background:

- **Vectors:** representing a collection of states, references, measurements, or actuator inputs.
- **Matrices:** transforming the current state and input into a predicted future state.
- **Matrix multiplication:** verifying how the dimensions and units of each model component relate.
- **Systems of linear equations:** solving for unknown states or model parameters.
- **Eigenvalues:** evaluating whether the modeled response is stable and how quickly it changes.
- **Discretization:** converting continuous differential equations into updates at the robot's loop
  period.
- **Covariance matrices:** describing the uncertainty and correlation of estimation errors.
- **Controllability:** determining whether the available actuators can move every required state.
- **Observability:** determining whether the available sensors can reconstruct every required state.

WPILib can perform much of the matrix calculation, but it cannot determine whether our selected
state, units, model, or sensor assumptions are physically correct. We therefore need to understand
what each matrix represents even when a library calculates the final controller or estimator gains.

## Planned implementation

We plan to develop the project in the following stages:

1. Select one simple mechanism and define its states, inputs, measurements, operating range, and
   safety constraints.
2. Calculate an initial model from the motor, gearing, mass, inertia, and geometry.
3. Simulate PID with feedforward and LQR under the same reference motion, voltage limit, and
   disturbance conditions.
4. Measure rise time, settling time, overshoot, steady-state error, and voltage use rather than
   judging the controllers only by appearance.
5. Characterize the physical mechanism with SysId and update the model with measured constants.
6. Add an observer when a required state is missing or a direct measurement is too noisy.
7. Test the controller under battery-voltage changes, additional load, sensor noise, actuator
   saturation, and reasonable model error.
8. Apply the same process to coupled or multivariable mechanisms only when the simpler experiment
   supports the additional complexity.
9. Coordinate complete robot actions with commands that run independent mechanisms in parallel and
   wait for measured position, velocity, and stability conditions instead of fixed delays.

Based on these tests, we will retain state-space control only where it produces a useful and
repeatable improvement. A simple mechanism may still be controlled more effectively with PID and
feedforward because that solution is easier to tune, diagnose, and repair during a competition.

## Example applications

- An arm or elevator that must control position and velocity while compensating for gravity.
- A turret that tracks a moving target while remaining inside its mechanical rotation limits.
- A flywheel that must reach a stable angular velocity before a game piece is released.
- A swerve drivetrain that follows a trajectory using estimated field position and velocity.
- A multi-joint mechanism in which the motion of one joint changes the behavior of another.

State-space control is most useful when prediction, state estimation, coupled motion, or multiple
actuators affect the result. The purpose of this project is to identify those situations, validate
the models experimentally, and give Team 6560 a repeatable process for implementing model-based
control when it provides a practical advantage.
