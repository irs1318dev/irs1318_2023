## Overview
> "Everything should be made as simple as possible, but not simpler." - Albert Einstein

Welcome to the Issaquah Robotics Society's Robot Programming Guide!

The Issaquah Robotics Society’s Robot code is designed to be a good example of a moderately large software project that students of varying levels of experience with programming can contribute to. This guide is meant to help you learn each component of the robot controlled by software, and act as a reference in robot programming projects.

First, it is crucial to understand how different parts of the robot work in tandem to comprise a functional machine.

## Intro to Actuators
Actuators can be defined as any physical component that converts electrical signal to motion. Think of it in the context of "actualizing your dreams," except you are an electric signal and the dream is for a linear actuator to extend. 

Examples of actuators include motors and pistons. 

Motors are devices that convert electrical power into rotational motion. They can move a variety of parts such as wheels, flywheels (for shooting things), electric fans, elevators, garage doors, linear actuators, and much more.

Pistons are mechanisms powered by pneumatics or hydraulics, and utilize pressure to extend or retract. Pistons have binary states, as they are controlled by pressure of gas or sometimes liquid. So, for example, 

Some piston-looking mechanisms may not actually be pistons, however, as there are mechanisms called linear actuators that look similar but have motorized control systems.

## Intro to Sensors
Sensors are components that sense certian behaviors of objects.

Think of them as components that behave like the 5 human senses except each serves a very specific purpose.

### Types of Sensors
#### Limit Switches
Limit switches are simple switches that are used to sense when two things are physically touching.  They are simple electronic devices that complete a circuit (or break a circuit) when the switch is pressed, and break a circuit (or complete it) when released.  In WPILib, you would use a DigitalInput, which returns a true or false based on whether the limit switch is pressed or not.

#### Encoders
Encoders are used to measure the amount that an axle has rotated.  There are different types of encoders (optical, magnetic).  We typically use a quadrature encoder, which can detect the amount of rotation and the direction in which the axle has rotated.  Each encoder has a rating for how many "pulses" or ticks it receives in a complete rotation of the axle.  Using some simple math based on the sizes of the wheels (and gears), you can calculate how far something has travelled.  In WPILib, you would typically use an Encoder object, which returns the number of ticks/pulses, the distance (based on the distance per pulse), or the velocity (if you trust the timer on the robot).  In some scenarios, such as when using a TalonSRX, TalonFX, or SparkMAX motor controller, the encoder plugs into the motor controller and is instead used as a part of controlling the motor.  In other scenarios, such as some absolute encoders, the sensor is actually an Analog sensor.  For such encoders, WPILib would use an AnalogInput, which returns a double (rational number) value between 0V (0 degrees) and 5V (360 degrees).

#### Through-Beam Sensors
Through-Beam Sensors are simple infrared sensors and lights that are used to sense whether there is anything between the light and sensor.  They are often used in the real world at the bottom of a garage door to detect if anything is under the garage door so it doesn’t get crushed.  This can be used on a robot to sense whether something is in a given location.  We often use them on robots to detect whether the game piece has been successfully picked up.  In WPILib, you would use an AnalogInput, which returns a double value (rational number) which indicates how many volts were detected by the infrared sensor.  This value will differ based on the through-beam sensor, so you can tell through experimentation whether it is tripped or not for a given value range.

#### Distance Sensors
There are various types of distance sensors, which can use either sound or light to sense how far away the robot is from something else.  In WPILib, you would use an AnalogInput, which would return a double value (rational number) which indicates how many volts were detected by the sensor.  This value will differ based on the sensor and its placement, so you can tell through experimentation what the values mean.  It is also possible to use a more complex sensor that would need to have code written for it to use I2C or another protocol to let the RoboRIO communicate with the sensor.

## Intro to Mechanisms
A mechanism is a piece of code (generally a file) that controls a specific part of the robot. 

Each mechanism should control and manage
1. Reading of sensors
2. Control of acuators

For example the intake mechanism could handle the intake of game pieces by using user input, handled by another class, to determine weather to move the acuators to clamp the game object or move them out to release.

## Structure of a mechanism (stub)
Start: Mechanism implements IMechanism

@Inject 

public ThisMechanism(IDriver driver, IRobotProvider provider)

This begins the class.

@Override
readSensors() 
 
This reads the sensors and prepares information you will need in the current update cycle.

@Override
update()
 
During this time, control any actuators.

@Override
stop() 

This is where you must command everything to stop. If you don't, it won't stop on disable and may become a safety hazard.

