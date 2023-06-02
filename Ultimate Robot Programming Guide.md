## Overview
> "Everything should be made as simple as possible, but not simpler." - Albert Einstein

Welcome to the Issaquah Robotics Society's Robot Programming Guide!

The Issaquah Robotics Society’s Robot code is designed to be a good example of a moderately large software project that students of varying levels of experience with programming can contribute to. This guide is meant to help you learn each component of the robot controlled by software, and act as a reference in robot programming projects.

First, it is crucial to understand how different parts of the robot work in tandem to comprise a functional machine.

## Actuators
Actuators can be defined as any physical component that converts electrical signal to motion. Think of it in the context of "actuating your dreams," except you are an electric signal and the dream is for a motor to spin.

Examples of actuators include motors and pistons. 

Motors are devices that convert electrical power into rotational motion. They can move a variety of parts such as wheels, flywheels (for shooting things), electric fans, elevators, garage doors, linear actuators, and much more.

Pistons 

## Structure of a mechanism
Start: Mechanism implements IMechanism

@Inject 

public ThisMechanism(IDriver driver, IRobotProvider provider)

This begins the class.

@Override
readSensors() 
 
This reads the sensors and prepares information you will need in the current update cycle.

@Override
update()
 
This is where you do stuff.

@Override
stop() 

This is where you tell stuff to stop. If you don't, it won't stop on disable.