Planned chapter guide:

Overview
Intro to Actuators
Intro to Sensors
Intro to Mechanisms
Intro to Robot Structure
Intro to Robot Code Design
Introduce Driver, Mechanisms
Intro to Mechanism Code
Structure
Using Actuators
Fauxbot Exercise 1 (Forklift)
Code organization
Tuning, Hardware, Electronics Constants
Intro to Driver
ButtonMap, Operations
Using Sensors
Fauxbot Exercise 2 (Garage Door)
Logging
Intro to PID, Positional PID
Fauxbot Exercise 3 (Elevator)
Velocity PID
Advanced Motor Controllers
Talon SRX, Talon FX, Spark MAX
Fauxbot Exercise 4 (Shooter)
Intro to Macros
Fauxbot Exercise 5 (Printer)
Advanced Macros
Fauxbot Exercise 5b (Printer Macro)
Intro to Autonomous
Intro to Path Planner
Intro to Trapezoidal Motion Profiling
Motion Magic


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

## Motors
#### [REMOVE THIS Electric motors are typically used to provide movement for the robot. They provide a rotational force that is dependent on the current setting on them and the amount of voltage that is available. Motors are useful when a certain amount of motion is needed or when there are motions that need to happen at different speeds (as opposed to all-or-nothing). Motors are used in places such as drive trains, elevators, and intakes. In WPILib, they are controlled using a double value (rational number) between -1.0 and 1.0. Since 2018, we have typically used the Talon SRX which can incorporate the abilities of a motor, an encoder, a top/bottom limit switch, and a PID controller to allow for advanced control. In 2020, we started using brushless motors, including the Falcon with its built-in TalonFX motor controller as well as the NEO with its corresponding Spark MAX motor controller.]

Movement for the robot typically involves the usage of electric motors. They provide a rotational force that is dependent on the current setting and voltage available. They are useful because they have variable speeds that can be changed with programming. They are used in mechanisms such as drivetrains, elevators, and intakes. We currently use the motor controller for NEO, the Spark MAX. 

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

Mechanisms handle the interactions with the actuators (e.g. motors, pneumatic solenoids) and sensors (e.g. encoders, limit switches) of each part of the robot, controlling them based on the operations from the Driver. A mechanism is a class that implements the IMechanism interface with a name based on the name of that portion of the robot (e.g. DriveTrain, Intake) combined with "Mechanism", such as ThingMechanism. It should be placed within the mechanisms folder with the other mechanisms and managers.

For example, an intake mechanism may intake some cargo, and control numerous motors and recieve information from sensors at the same time. When given the command, it will start running the intake motors to intake cargo. At the same time, it will constantly check whether or not a through-beam in the intake has been broken to determine if cargo has been recieved, and then stop the motors. These would all be written into a singular mechanism as functions, and called from the ButtonMap.

Define mechanism class and member variables

```java
@Singleton
public class ThingMechanism implements IMechanism
{
  // driver
  private final IDriver driver;

  // sensors and actuators
  private final ISomeSensor nameOfSensor;
  private final ISomeActuator nameOfAcutator;

  // logger
  private final ILogger logger;

  // sensor values
  private boolean someSetting;

  // mechanism state
  private boolean someState;
```

At the top of the class, you should have the driver (``private IDriver driver;``), followed by a list of the definitions of your different actuators and sensors (``private final ISomeActuator nameOfActuator;``) and (``private final ISomeSensor nameOfSensor;``). These will be initialized in the constructor. After the driver and set of actuators and sensors are defined, you will also need to define the logger (``private ILogger logger;``), anything that will be read from the sensors (``private boolean someSetting;``) and any state that needs to be kept for the operation of the mechanism (``private boolean someState;``).

##### Write mechanism constructor
```java
  @Inject
  public ThingMechanism(IDriver driver, IRobotProvider provider, LoggingManager logger)
  {
    this.driver = driver;

    this.nameOfSensor = provider.GetSomeSensor(ElectronicsConstants.THING_NAMEOFSENSOR_PWM_CHANNEL);
    this.nameOfActuator = provider.GetSomeActuator(ElectronicsConstants.THING_NAMEOFACTUATOR_PWM_CHANNEL);

    this.logger = logger;

    this.someSetting = false;
    this.someState = false;
  }
```

After defining all of the class's variables, you will define a constructor named like "``public ThingMechanism(IDriver driver, IRobotProvider provider, LoggingManager logger)``". Since 2017 we’ve made use of Google’s Guice to control dependency injection, which is the reason why the special ``@Inject`` markup is required. You will first set the driver to the value that is provided to the constructor by Guice. You will then set the value for each actuator and sensor you defined earlier by calling the corresponding function on the IRobotProvider that is also passed into the constructor. These functions will take some number of arguments based on how the actuators/sensors are physically plugged together in the robot (such as CAN Ids, DIO channel, Analog channel, PCM channel, or PWM channel). 

These arguments should be placed as constants in the ElectronicsConstants file with names such as ``THING_NAMEOFACTUATOR_PWM_CHANNEL`` (The ``ElectronicsConstants`` file will be covered later). We don’t necessarily know in advance how the robot plugs together, so they can be initialized with a value of -1 until we do. After initializing the sensors and actuators, you should set the logger as provided and the settings and states to their default values.

##### Write mechanism readSensors function:

```java
  @Override
  public void readSensors()
  {
    this.someSetting = this.nameOfSensor.get();

    this.logger.logBoolean(LoggingKey.ThingSomeSetting, this.someSetting);
  }
The ``readSensors()`` function reads from the relevant sensors for that mechanism, stores the results in class member variables, and then logs the results to the logger. Most simple sensor types have a simple ``get()`` function or similar to read the current value from that sensor. An entry in the ``LoggingKey`` enum will need to be added to correspond to each setting that we want to log.
```

##### Write mechanism update function:
```java
  @Override
  public void update()
  {
    boolean shouldThingAction = this.driver.getDigital(DigitalOperation.ThingAction);

    double thingActionAmount = 0.0;
    if (shouldThingAction)
    {
      thingActionAmount = TuningConstants.THING_ACTION_AMOUNT;
    }

    this.nameOfActuator.set(thingActionAmount);
  }
```

The ``update()`` function examines the inputs that we retrieve from the ``IDriver``, and then calculates the various outputs to use applies them to the outputs for the relevant actuators. For some mechanisms, the logic will be very simple - reading an operation and applying it to an actuator (extend, retract, like a piston). Other mechanisms will involve some internal state and information from the most recent readings from the sensors, and possibly some math in order to determine what the actuator should do. Note that there will often be a "degree" to which something should be done that we don't know in advance (like how linear actuators can extend certain specific amounts). If we are intaking a ball, we may want to carefully choose the correct strength to run the intake motor at. Because we don't know this value in advance and will discover it experimentally, we should put such values into the ``TuningConstants`` file as a constant with a guess for the value. This value should be low as to prevent any accidents.

##### Write mechanism stop function:

```java
  @Override
  public void stop()
  {
    this.nameOfActuator.set(0.0);
  }
```

The stop function tells each of the actuators to stop moving. This typically means setting any ``Motor`` to 0.0 and any ``DoubleSolenoid`` to ``kOff``. It is called when the robot is being disabled, and it is very important to stop everything to ensure that the robot is safe and doesn't make any uncontrolled movements.

##### Write any getter functions:

```java
  public boolean getSomeSetting()
  {
    return this.someSetting;
  }
```

When there are sensors being read, often we will want to incorporate the data that they return into the running of tasks as a part of macros and autonomous routines. In order to support that, we must add getter functions so that the tasks can access the values that were read from the sensors. These functions just simply return the value that was read during the ``readSensors`` function. Typically we can skip writing these until a task is being written needs this information.

## Using Actuators

All accuators must extend from 2 base classes

1. IMotor for Motor Controllers
2. IDoubleSolenoid for Double Solenoid like Pistons

Note: Motor Controllers are modules that move motors based on code. They are physically attached to motors because motors are too dumb to understand code. We will never be programing the physical motors, instead we will always work with motor controllers that intern move the motors based on the code.

You can also use Special classes like ITalonFX or ISparkMax to use API's provided by these special motor classes.

### Set Up Process

An actuator is initialized by creating a variable IMotor, ITalonFX , ISparkMax, ITalonSRX, etc.

```private final ITalonSRX lowerLeftArmLinearActuator;```

In the constructor you need to initalize the CAN IDs of the motors (you can think of CAN as a method to identify each motor on the physical robot, as each motor is assigned a CAN ID during the installation process. This document will go into more detail about CAN later). 

```this.lowerLeftArmLinearActuator = provider.getTalonSRX(1);```

This is an example for TalonSRX but it will be almost similar for other motor controllers too, except you will use ```provider.<Controller Name>():```

With this you should have your actuator ready to program on

### Basic Actuator Movement 


