package frc.robot.mechanisms;

import frc.robot.*;
import frc.robot.common.*;
import frc.robot.common.robotprovider.*;
import frc.robot.driver.*;
import frc.robot.driver.common.IDriver;

import com.google.inject.Inject;
import com.google.inject.Singleton;

@Singleton
public class ArmMechanism implements IMechanism
{
    //----------------- General variables -----------------

    private static final int defaultPidSlotId = 0;

    private final IDriver driver;
    private final ILogger logger;
    private final ITimer timer;
    private final PowerManager powerManager;

    private double prevTime;

    //----------------- Cone flipper Variables -----------------

    // private final IDoubleSolenoid rightConeFlipper;
    // private final IDoubleSolenoid leftConeFlipper;

    // private double leftFlipperTransitionTime;
    // private double rightFlipperTransitionTime;
    // private enum ConeFlipperState
    // {
    //     Extended,
    //     Extending,
    //     ExtendingWait,
    //     Retracted,
    //     Retracting,
    // };

    // private ConeFlipperState curLeftFlipperState;
    // private ConeFlipperState curRightFlipperState;

    //----------------- Main Arm Variables -----------------

    private final ITalonSRX lowerLeftArm;
    private final ITalonSRX lowerRightArm;
    private final ITalonSRX upperArm;

    // Positions are in ticks, Velocities are in ticks per 100ms
    private double lowerLeftArmPosition;
    private double lowerRightArmPosition;
    private double upperArmPosition;
    private double lowerLeftArmVelocity;
    private double lowerRightArmVelocity;
    private double upperArmVelocity;

    // actual positions calculated using forward kinematics
    private double xPosition;
    private double zPosition;

    private boolean inSimpleMode;

    private double desiredLowerLeftArmPosition;
    private double desiredLowerRightArmPosition;
    private double desiredUpperArmPosition;

    private double armRetractionStartTime;

    //----------------- Intake Variables -----------------

    private final ITalonSRX intakeMotor;
    private final IDoubleSolenoid intakeExtender;
 
    private enum IntakeState
    {
        Retracted,
        Extended
    };
 
    private IntakeState currentIntakeState;

    @Inject
    public ArmMechanism(
        IDriver driver,
        LoggingManager logger,
        IRobotProvider provider,
        PowerManager powerManager,
        ITimer timer)
    {
        this.driver = driver;
        this.logger = logger;
        this.timer = timer;
        this.powerManager = powerManager;

        //------------------------- Main Arm Initializiation -------------------------

        this.lowerLeftArm = provider.getTalonSRX(ElectronicsConstants.ARM_LOWER_LEFT_CAN_ID);
        this.lowerRightArm = provider.getTalonSRX(ElectronicsConstants.ARM_LOWER_RIGHT_CAN_ID);
        this.upperArm = provider.getTalonSRX(ElectronicsConstants.ARM_UPPER_CAN_ID);

        this.inSimpleMode = TuningConstants.ARM_USE_SIMPLE_MODE;

        this.lowerLeftArm.setMotionMagicPIDF(
            TuningConstants.ARM_LOWER_LEFT_POSITION_MM_PID_KP,
            TuningConstants.ARM_LOWER_LEFT_POSITION_MM_PID_KI,
            TuningConstants.ARM_LOWER_LEFT_POSITION_MM_PID_KD,
            TuningConstants.ARM_LOWER_LEFT_POSITION_MM_PID_KF,
            TuningConstants.ARM_LOWER_LEFT_POSITION_MM_CRUISE_VELOCITY,
            TuningConstants.ARM_LOWER_LEFT_POSITION_MM_ACCELERATION,
            ArmMechanism.defaultPidSlotId);
        this.lowerRightArm.setMotionMagicPIDF(
            TuningConstants.ARM_LOWER_RIGHT_POSITION_MM_PID_KP,
            TuningConstants.ARM_LOWER_RIGHT_POSITION_MM_PID_KI,
            TuningConstants.ARM_LOWER_RIGHT_POSITION_MM_PID_KD,
            TuningConstants.ARM_LOWER_RIGHT_POSITION_MM_PID_KF,
            TuningConstants.ARM_LOWER_RIGHT_POSITION_MM_CRUISE_VELOCITY,
            TuningConstants.ARM_LOWER_RIGHT_POSITION_MM_ACCELERATION,
            ArmMechanism.defaultPidSlotId);

        this.upperArm.setMotionMagicPIDF(
            TuningConstants.ARM_UPPER_POSITION_MM_PID_KP,
            TuningConstants.ARM_UPPER_POSITION_MM_PID_KI,
            TuningConstants.ARM_UPPER_POSITION_MM_PID_KD,
            TuningConstants.ARM_UPPER_POSITION_MM_PID_KF,
            TuningConstants.ARM_UPPER_POSITION_MM_CRUISE_VELOCITY,
            TuningConstants.ARM_UPPER_POSITION_MM_ACCELERATION,
            ArmMechanism.defaultPidSlotId);

        this.lowerLeftArmVelocity = 0.0;
        this.lowerRightArmVelocity = 0.0;
        this.upperArmVelocity = 0.0;
        this.lowerLeftArmPosition = HardwareConstants.ARM_EXTENTION_LENGTH * HardwareConstants.ARM_STRING_ENCODER_TICKS_PER_INCH; // Fully Extended
        this.lowerRightArmPosition = HardwareConstants.ARM_EXTENTION_LENGTH * HardwareConstants.ARM_STRING_ENCODER_TICKS_PER_INCH; // Fully Extended
        this.upperArmPosition = 0.0; // Fully Retracted
        this.xPosition = TuningConstants.ARM_FULLY_RETRACTED_X_POSITION;
        this.zPosition = TuningConstants.ARM_FULLY_RETRACTED_Z_POSITION;

        this.desiredLowerLeftArmPosition = this.lowerLeftArmPosition;
        this.desiredLowerRightArmPosition = this.lowerRightArmPosition;
        this.desiredUpperArmPosition = this.upperArmPosition;

        this.lowerLeftArm.setSensorType(TalonXFeedbackDevice.QuadEncoder);
        this.lowerRightArm.setSensorType(TalonXFeedbackDevice.QuadEncoder);
        this.upperArm.setSensorType(TalonXFeedbackDevice.QuadEncoder);

        this.lowerLeftArm.setInvertSensor(TuningConstants.ARM_LOWER_LEFT_INVERT_SENSOR);
        this.lowerRightArm.setInvertSensor(TuningConstants.ARM_LOWER_RIGHT_INVERT_SENSOR);
        this.upperArm.setInvertSensor(TuningConstants.ARM_UPPER_INVERT_SENSOR);

        this.lowerLeftArm.setPosition(this.lowerLeftArmPosition);
        this.lowerRightArm.setPosition(this.lowerRightArmPosition);
        this.upperArm.setPosition(this.upperArmPosition);
        if (this.inSimpleMode)
        {
            this.lowerLeftArm.setControlMode(TalonXControlMode.PercentOutput);
            this.lowerRightArm.setControlMode(TalonXControlMode.PercentOutput);
            this.upperArm.setControlMode(TalonXControlMode.PercentOutput);
        }
        else
        {
            this.lowerLeftArm.setControlMode(TalonXControlMode.MotionMagicPosition);
            this.lowerRightArm.setControlMode(TalonXControlMode.MotionMagicPosition);
            this.upperArm.setControlMode(TalonXControlMode.MotionMagicPosition);
        }

        this.lowerLeftArm.setInvertOutput(TuningConstants.ARM_LOWER_LEFT_INVERT_OUTPUT);
        this.lowerRightArm.setInvertOutput(TuningConstants.ARM_LOWER_RIGHT_INVERT_OUTPUT);
        this.upperArm.setInvertOutput(TuningConstants.ARM_UPPER_INVERT_OUTPUT);

        this.lowerLeftArm.setNeutralMode(MotorNeutralMode.Brake);
        this.lowerRightArm.setNeutralMode(MotorNeutralMode.Brake);
        this.upperArm.setNeutralMode(MotorNeutralMode.Brake);

        //------------------------- Side Stick Initialization ------------------------------
        // this.leftConeFlipper = provider.getDoubleSolenoid(
        //     ElectronicsConstants.PNEUMATICS_MODULE_A,
        //     ElectronicsConstants.PNEUMATICS_MODULE_TYPE_A,
        //     ElectronicsConstants.LEFT_SIDE_STICK_PISTON_FORWARD,
        //     ElectronicsConstants.LEFT_SIDE_STICK_PISTON_BACKWARD);

        // this.rightConeFlipper = provider.getDoubleSolenoid(
        //     ElectronicsConstants.PNEUMATICS_MODULE_A,
        //     ElectronicsConstants.PNEUMATICS_MODULE_TYPE_A,
        //     ElectronicsConstants.RIGHT_SIDE_STICK_PISTON_FORWARD,
        //     ElectronicsConstants.RIGHT_SIDE_STICK_PISTON_BACKWARD);

        // this.leftFlipperTransitionTime = 0.0;
        // this.rightFlipperTransitionTime = 0.0;

        // this.curRightFlipperState = ConeFlipperState.Retracted;
        // this.curLeftFlipperState = ConeFlipperState.Retracted;

        //-------------------------- Intake Initialization ----------------------------------

        this.intakeMotor = provider.getTalonSRX(ElectronicsConstants.INTAKE_MOTOR_CAN_ID);
        this.intakeMotor.setControlMode(TalonXControlMode.PercentOutput);
        this.intakeMotor.setInvertOutput(TuningConstants.ARM_INTAKE_MOTOR_INVERT_OUTPUT);
        this.intakeMotor.setNeutralMode(MotorNeutralMode.Brake);

        this.intakeExtender =
            provider.getDoubleSolenoid(
                ElectronicsConstants.PNEUMATICS_MODULE_A,
                ElectronicsConstants.PNEUMATICS_MODULE_TYPE_A,
                ElectronicsConstants.ARM_INTAKE_PISTON_FORWARD,
                ElectronicsConstants.ARM_INTAKE_PISTON_REVERSE);

        this.currentIntakeState = IntakeState.Retracted;
    }


    @Override
    public void readSensors()
    {
        this.lowerLeftArmPosition = this.lowerLeftArm.getPosition();
        this.lowerRightArmPosition = this.lowerRightArm.getPosition();
        this.upperArmPosition = this.upperArm.getPosition();
        this.lowerLeftArmVelocity = this.lowerLeftArm.getVelocity();
        this.lowerRightArmVelocity = this.lowerRightArm.getVelocity();
        this.upperArmVelocity = this.upperArm.getVelocity();

        this.logger.logNumber(LoggingKey.ArmLowerLeftPosition, this.lowerLeftArmPosition);
        this.logger.logNumber(LoggingKey.ArmLowerRightPosition, this.lowerRightArmPosition);
        this.logger.logNumber(LoggingKey.ArmUpperPosition, this.upperArmPosition);
        this.logger.logNumber(LoggingKey.ArmLowerLeftVelocity, this.lowerLeftArmVelocity);
        this.logger.logNumber(LoggingKey.ArmLowerRightVelocity, this.lowerRightArmVelocity);
        this.logger.logNumber(LoggingKey.ArmUpperVelocity, this.upperArmVelocity);

        DoubleTuple offsets = ArmMechanism.calculateFK(
            (this.lowerLeftArmPosition + this.lowerRightArmPosition) / 2.0,
            this.upperArmPosition);

        if (offsets != null)
        {
            this.logger.logNumber(LoggingKey.ArmFKXPosition, offsets.first);
            this.logger.logNumber(LoggingKey.ArmFKZPosition, offsets.second);
        }        
    }

    @Override
    public void update()
    {
        double currTime = this.timer.get();

        //----------------------------------- Main Arm Control Mode -----------------------------------
        if (this.driver.getDigital(DigitalOperation.ArmEnableSimpleMode))
        {
            this.inSimpleMode = true;

            this.lowerLeftArm.setControlMode(TalonXControlMode.PercentOutput);
            this.lowerRightArm.setControlMode(TalonXControlMode.PercentOutput);
            this.upperArm.setControlMode(TalonXControlMode.PercentOutput);
        }
        else if (this.driver.getDigital(DigitalOperation.ArmDisableSimpleMode))
        {
            this.inSimpleMode = false;

            this.lowerLeftArm.setControlMode(TalonXControlMode.MotionMagicPosition);
            this.lowerRightArm.setControlMode(TalonXControlMode.MotionMagicPosition);
            this.upperArm.setControlMode(TalonXControlMode.MotionMagicPosition);

            this.desiredLowerLeftArmPosition = this.lowerLeftArmPosition;
            this.desiredLowerRightArmPosition = this.lowerRightArmPosition;
            this.desiredUpperArmPosition = this.upperArmPosition;
        }

        //----------------------------------- Flippers -----------------------------------
        // boolean extendRightFlipper = this.driver.getDigital(DigitalOperation.ExtendRightFlipper);
        // boolean extendLeftFlipper = this.driver.getDigital(DigitalOperation.ExtendLeftFlipper);
        // switch (this.curRightFlipperState)
        // {
        //     case Retracted:
        //         if (extendRightFlipper)
        //         {
        //             this.curRightFlipperState = ConeFlipperState.ExtendingWait;
        //         }

        //         break;

        //     case ExtendingWait:
        //         if (!extendRightFlipper)
        //         {
        //             this.curRightFlipperState = ConeFlipperState.Retracted;
        //         }
        //         else if (this.curLeftFlipperState == ConeFlipperState.Retracted &&
        //             ((!this.inSimpleMode &&
        //                 this.upperArmPosition <= TuningConstants.ARM_UPPER_NEAR_FULL_RETRACTED_LENGTH * HardwareConstants.ARM_STRING_ENCODER_TICKS_PER_INCH &&  // some value slightly larger than 0.0 inches
        //                 this.lowerLeftArmPosition >= TuningConstants.ARM_LOWER_NEAR_FULL_EXTENSION_LENGTH * HardwareConstants.ARM_STRING_ENCODER_TICKS_PER_INCH) ||  // some value slightly smaller than fully extended
        //             (this.inSimpleMode && currTime - this.armRetractionStartTime > TuningConstants.ARM_RETRACTION_MAX_TIME)))
        //         {
        //             this.curRightFlipperState = ConeFlipperState.Extending;
        //             this.rightFlipperTransitionTime = currTime;
        //         }

        //         break;

        //     case Extending:
        //         if (!extendRightFlipper)
        //         {
        //             this.curRightFlipperState = ConeFlipperState.Retracting;
        //             this.rightFlipperTransitionTime = currTime;
        //         }
        //         else if (currTime - this.rightFlipperTransitionTime > TuningConstants.ARM_FLIPPER_EXTEND_WAIT_DURATION)
        //         {
        //             this.curRightFlipperState = ConeFlipperState.Extended;
        //         }

        //         break;

        //     case Extended:
        //         if (!extendRightFlipper)
        //         {
        //             this.curRightFlipperState = ConeFlipperState.Retracting;
        //             this.rightFlipperTransitionTime = currTime;
        //         }

        //         break;

        //     case Retracting:
        //         if (extendRightFlipper)
        //         {
        //             this.curRightFlipperState = ConeFlipperState.Extending;
        //             this.rightFlipperTransitionTime = currTime;
        //         }
        //         else if (currTime - this.rightFlipperTransitionTime > TuningConstants.ARM_FLIPPER_RETRACT_WAIT_DURATION)
        //         {
        //             this.curRightFlipperState = ConeFlipperState.Retracted;
        //         }

        //         break;
        // }

        // switch (this.curLeftFlipperState)
        // {
        //     case Retracted:
        //         if (extendLeftFlipper)
        //         {
        //             this.curLeftFlipperState = ConeFlipperState.ExtendingWait;
        //             this.armRetractionStartTime = currTime;
        //         }

        //         break;

        //     case ExtendingWait:
        //         if (!extendLeftFlipper)
        //         {
        //             this.curLeftFlipperState = ConeFlipperState.Retracted;
        //         }
        //         else if (this.curLeftFlipperState == ConeFlipperState.Retracted &&
        //             ((!this.inSimpleMode &&
        //                 this.upperArmPosition <= TuningConstants.ARM_UPPER_NEAR_FULL_RETRACTED_LENGTH * HardwareConstants.ARM_STRING_ENCODER_TICKS_PER_INCH &&  // some value slightly larger than 0.0 inches
        //                 this.lowerLeftArmPosition >= TuningConstants.ARM_LOWER_NEAR_FULL_EXTENSION_LENGTH * HardwareConstants.ARM_STRING_ENCODER_TICKS_PER_INCH) ||  // some value slightly smaller than fully extended
        //             (this.inSimpleMode && currTime - this.armRetractionStartTime > TuningConstants.ARM_RETRACTION_MAX_TIME)))
        //         {
        //             this.curLeftFlipperState = ConeFlipperState.Extending;
        //             this.leftFlipperTransitionTime = currTime;
        //         }

        //         break;

        //     case Extending:
        //         if (!extendLeftFlipper)
        //         {
        //             this.curLeftFlipperState = ConeFlipperState.Retracting;
        //             this.leftFlipperTransitionTime = currTime;
        //         }
        //         else if (currTime - this.leftFlipperTransitionTime > TuningConstants.ARM_FLIPPER_EXTEND_WAIT_DURATION)
        //         {
        //             this.curLeftFlipperState = ConeFlipperState.Extended;
        //         }

        //         break;

        //     case Extended:
        //         if (!extendLeftFlipper)
        //         {
        //             this.curLeftFlipperState = ConeFlipperState.Retracting;
        //             this.leftFlipperTransitionTime = currTime;
        //         }

        //         break;

        //     case Retracting:
        //         if (extendLeftFlipper)
        //         {
        //             this.curLeftFlipperState = ConeFlipperState.Extending;
        //             this.leftFlipperTransitionTime = currTime;
        //         }
        //         else if (currTime - this.leftFlipperTransitionTime > TuningConstants.ARM_FLIPPER_RETRACT_WAIT_DURATION)
        //         {
        //             this.curLeftFlipperState = ConeFlipperState.Retracted;
        //         }

        //         break;
        // }
        
        // this.logger.logString(LoggingKey.ArmRightFlipperState, this.curRightFlipperState.toString());
        // switch (this.curRightFlipperState)
        // {
        //     case Extended:
        //     case Extending:
        //         this.rightConeFlipper.set(DoubleSolenoidValue.Forward);
        //         break;

        //     default:
        //     case ExtendingWait:
        //     case Retracted:
        //     case Retracting:
        //         this.rightConeFlipper.set(DoubleSolenoidValue.Reverse);
        //         break;
        // }

        // this.logger.logString(LoggingKey.ArmLeftFlipperState, this.curLeftFlipperState.toString());
        // switch (this.curLeftFlipperState)
        // {
        //     case Extended:
        //     case Extending:
        //         this.leftConeFlipper.set(DoubleSolenoidValue.Forward);
        //         break;

        //     default:
        //     case ExtendingWait:
        //     case Retracted:
        //     case Retracting:
        //         this.leftConeFlipper.set(DoubleSolenoidValue.Reverse);
        //         break;
        // }

        //----------------------------------- Intake Update -----------------------------------

        // control intake rollers
        double intakePower = TuningConstants.ZERO;
        if (this.driver.getDigital(DigitalOperation.IntakeIn))
        {
            intakePower = TuningConstants.ARM_INTAKE_POWER;
        }
        else if (this.driver.getDigital(DigitalOperation.IntakeOut))
        {
            intakePower = -TuningConstants.ARM_INTAKE_POWER;
        }

        this.intakeMotor.set(intakePower);
        this.logger.logNumber(LoggingKey.ArmIntakePower, intakePower);

        // intake state transitions
        if (this.driver.getDigital(DigitalOperation.IntakeExtend))
        {
            this.currentIntakeState = IntakeState.Extended;
        }
        else if (this.driver.getDigital(DigitalOperation.IntakeRetract))
        {
            this.currentIntakeState = IntakeState.Retracted;
        }

        this.logger.logBoolean(LoggingKey.ArmIntakeExtended, this.currentIntakeState == IntakeState.Extended);
        switch (this.currentIntakeState)
        {
            case Extended:
                this.intakeExtender.set(DoubleSolenoidValue.Forward);
                break;
            default:
            case Retracted:
                this.intakeExtender.set(DoubleSolenoidValue.Reverse);
                break;
        }

        //----------------------------------- Main Arm -----------------------------------
        // if (this.curLeftFlipperState != ConeFlipperState.Retracted || this.curRightFlipperState != ConeFlipperState.Retracted)
        // {
        //     if (this.inSimpleMode)
        //     {
        //         this.lowerLeftArm.set(TuningConstants.ARM_MAX_FORWARD_SIMPLE_VELOCITY);
        //         this.lowerRightArm.set(TuningConstants.ARM_MAX_FORWARD_SIMPLE_VELOCITY);
        //         this.upperArm.set(TuningConstants.ARM_MAX_REVERSE_SIMPLE_VELOCITY);
        //     }
        //     else
        //     {
        //         this.desiredLowerLeftArmPosition = TuningConstants.ARM_LOWER_FULL_EXTENTION_LENGTH * HardwareConstants.ARM_STRING_ENCODER_TICKS_PER_INCH;
        //         this.desiredLowerRightArmPosition = TuningConstants.ARM_LOWER_FULL_EXTENTION_LENGTH * HardwareConstants.ARM_STRING_ENCODER_TICKS_PER_INCH;
        //         this.desiredUpperArmPosition = HardwareConstants.ARM_UPPER_FULL_RETRACTED_LENGTH * HardwareConstants.ARM_STRING_ENCODER_TICKS_PER_INCH;

        //         this.lowerLeftArm.set(this.desiredLowerLeftArmPosition);
        //         this.lowerRightArm.set(this.desiredLowerRightArmPosition);
        //         this.upperArm.set(this.desiredUpperArmPosition);
        //     }
        // }
        // else
        {
            if (this.inSimpleMode)
            {
                // controlled by joysticks
                double lowerArmPower = this.driver.getAnalog(AnalogOperation.ArmSimpleForceLower);
                this.lowerLeftArm.set(lowerArmPower);
                this.lowerRightArm.set(lowerArmPower);
                this.upperArm.set(this.driver.getAnalog(AnalogOperation.ArmSimpleForceUpper));
            }
            else
            {
                 if (this.driver.getAnalog(AnalogOperation.ArmIKXPosition) >= 0.0 && this.driver.getAnalog(AnalogOperation.ArmIKZPosition) >= 0.0)
                 {
                    // controlled by macro
                    DoubleTuple ikResult = ArmMechanism.calculateIK(this.driver.getAnalog(AnalogOperation.ArmIKXPosition), this.driver.getAnalog(AnalogOperation.ArmIKZPosition));
                    if (ikResult != null)
                    {
                        this.desiredLowerLeftArmPosition = ikResult.second;
                        this.desiredLowerRightArmPosition = ikResult.second;
                        this.desiredUpperArmPosition = ikResult.first;
                    }
                }
                else if (this.driver.getAnalog(AnalogOperation.ArmMMUpperPosition) >= 0.0 && this.driver.getAnalog(AnalogOperation.ArmMMLowerPosition) >= 0.0)
                {
                    // controlled by macro
                    this.desiredLowerLeftArmPosition = this.driver.getAnalog(AnalogOperation.ArmMMLowerPosition);
                    this.desiredLowerRightArmPosition = this.driver.getAnalog(AnalogOperation.ArmMMLowerPosition);
                    this.desiredUpperArmPosition = this.driver.getAnalog(AnalogOperation.ArmMMUpperPosition);
                }
                else if (this.driver.getAnalog(AnalogOperation.ArmLowerPositionAdjustment) != 0.0 && this.driver.getAnalog(AnalogOperation.ArmUpperPositionAdjustment) != 0.0)
                {
                    // controlled by joysticks
                    double elapsedTime = currTime - this.prevTime;
                    this.desiredLowerLeftArmPosition += this.driver.getAnalog(AnalogOperation.ArmLowerPositionAdjustment) * TuningConstants.ARM_LOWER_EXTENSION_ADJUSTMENT_VELOCITY * HardwareConstants.ARM_STRING_ENCODER_TICKS_PER_INCH * elapsedTime;
                    this.desiredLowerRightArmPosition += this.driver.getAnalog(AnalogOperation.ArmLowerPositionAdjustment) * TuningConstants.ARM_LOWER_EXTENSION_ADJUSTMENT_VELOCITY * HardwareConstants.ARM_STRING_ENCODER_TICKS_PER_INCH * elapsedTime;
                    this.desiredUpperArmPosition += this.driver.getAnalog(AnalogOperation.ArmUpperPositionAdjustment) * TuningConstants.ARM_UPPER_EXTENSION_ADJUSTMENT_VELOCITY * HardwareConstants.ARM_STRING_ENCODER_TICKS_PER_INCH * elapsedTime;
                }

                this.lowerLeftArm.set(this.desiredLowerLeftArmPosition);
                this.lowerRightArm.set(this.desiredLowerRightArmPosition);
                this.upperArm.set(this.desiredUpperArmPosition);
            }
        }        

        this.logger.logNumber(LoggingKey.ArmLowerLeftDesiredPosition, this.desiredLowerLeftArmPosition);
        this.logger.logNumber(LoggingKey.ArmLowerRightDesiredPosition, this.desiredLowerRightArmPosition);
        this.logger.logNumber(LoggingKey.ArmUpperDesiredPosition, this.desiredUpperArmPosition);

        this.prevTime = currTime;
    }

    @Override
    public void stop()
    {
        this.lowerLeftArm.stop();
        this.lowerRightArm.stop();
        this.upperArm.stop();
        this.intakeMotor.stop();
        this.intakeExtender.set(DoubleSolenoidValue.Off);
        //this.leftConeFlipper.set(DoubleSolenoidValue.Off);
        //this.rightConeFlipper.set(DoubleSolenoidValue.Off);
    }

    public double getMMLowerPosition()
    {
        return this.lowerLeftArmPosition;
    }

    public double getMMUpperPosition()
    {
        return this.upperArmPosition;
    }

    public double getFKXPosition()
    {
        return this.xPosition;
    }

    public double getFKZPosition()
    {
        return this.zPosition;
    }

    /**
     * Calculate the desired actuator values based on a desired position for the end-effector,
     * using Inverse Kinematics
     * @param x offset (in inches)
     * @param z offset (in inches)
     * @return pair of lower, upper linear actuator extensions (in ticks)
     */
    static DoubleTuple calculateIK(double x, double z)
    {
        if (x > TuningConstants.ARM_MAX_IKX_EXTENSION_LENGTH ||
            z > TuningConstants.ARM_MAX_IKZ_EXTENSION_HEIGHT)
        {
            return null;
        }

        // special-case fully-retracted zone to the fully retracted position
        if (x <= TuningConstants.ARM_FULLY_RETRACTED_X_POSITION &&
            z <= TuningConstants.ARM_FULLY_RETRACTED_Z_POSITION)
        {
            return new DoubleTuple(
                HardwareConstants.ARM_EXTENTION_LENGTH * HardwareConstants.ARM_STRING_ENCODER_TICKS_PER_INCH,
                TuningConstants.ZERO);
        }

        DoubleTuple ikAngles = ArmMechanism.calculateIKAnglesFromPosition(x, z);
        if (ikAngles == null)
        {
            return null;
        }

        return ArmMechanism.calculateIKExtensionsFromAngles(ikAngles.first, ikAngles.second);
    }

    /**
     * Calculate the current end-effector position (x, z offsets) given the current linear actuator extensions,
     * using Forward Kinematics
     * @param lowerLAExtension lower linear actuator extension (in ticks)
     * @param upperLAExtension upper linear actuator extension (in ticks)
     * @return x, z offsets where the end-effector is located (in inches)
     */
    static DoubleTuple calculateFK(double lowerLAExtension, double upperLAExtension)
    {
        // Outside the valid lengths, we don't really know - special case these!
        if (lowerLAExtension > HardwareConstants.ARM_EXTENTION_LENGTH * HardwareConstants.ARM_STRING_ENCODER_TICKS_PER_INCH ||
            upperLAExtension > HardwareConstants.ARM_EXTENTION_LENGTH * HardwareConstants.ARM_STRING_ENCODER_TICKS_PER_INCH ||
            lowerLAExtension < 0.0 ||
            upperLAExtension < 0.0)
        {
            return null;
        }

        DoubleTuple fkAngles = ArmMechanism.calculateFKAnglesFromExtensions(lowerLAExtension, upperLAExtension);
        if (fkAngles == null)
        {
            return null;
        }

        return ArmMechanism.calculateFKPositionsFromAngles(fkAngles.first, fkAngles.second);
    }

    /**
     * Calculate the angles for the lower and upper arms based on goal offsets for the end effector
     * using inverse kinematics 
     * @param x goal offset (in inches)
     * @param z goal offset (in inches)
     * @return pair of lower, upper angles (theta1, theta2) (in degrees) or null if the position is unreachable
     */
    static DoubleTuple calculateIKAnglesFromPosition(double x, double z)
    {
        final double L1 = HardwareConstants.ARM_LOWER_ARM_LENGTH;
        final double L2 = HardwareConstants.ARM_UPPER_ARM_LENGTH;

        double R = Math.sqrt(x * x + z * z);
        Double theta2 = Helpers.calculateLawOfCosinesAngleOrNull(L1, L2, R);
        if (theta2 == null)
        {
            // must be impossible to reach this position with our current dimensions...
            return null;
        }

        Double alpha = Helpers.calculateLawOfCosinesAngleOrNull(L1, R, L2);
        if (alpha == null)
        {
            // must be impossible to reach this position with our current dimensions...
            return null;
        }

        double beta = Helpers.atan2d(z, x);
        double theta1 = alpha + beta;

        // in order lower, upper
        return new DoubleTuple(theta1, theta2);
    }

    /**
     * Calculate the offsets of the end effector based on the angles for the lower and upper arms
     * using forward kinematics
     * @param lowerAngle theta1 (in degrees)
     * @param upperAngle theta2 (in degrees)
     * @return pair of horizontal, vertical offsets (x, z) (in inches)
     */
    static DoubleTuple calculateFKPositionsFromAngles(double lowerAngle, double upperAngle)
    {
        final double L1 = HardwareConstants.ARM_LOWER_ARM_LENGTH;
        final double L2 = HardwareConstants.ARM_UPPER_ARM_LENGTH;

        double xPosition = L1 * Helpers.cosd(lowerAngle) + L2 * Helpers.cosd(lowerAngle + 180.0 - upperAngle);
        double zPosition = L1 * Helpers.sind(lowerAngle) + L2 * Helpers.sind(lowerAngle + 180.0 - upperAngle);
        return new DoubleTuple(xPosition, zPosition);
    }

    /**
     * Calculate desired extension of the linear actuator based on the angles of the arms
     * @param theta1 lower arm angle (in degrees)
     * @param theta2 upper arm angle (in degrees)
     * @return pair of lower, upper linear actuator extensions (in ticks) or null if the position is unreachable
     */
    static DoubleTuple calculateIKExtensionsFromAngles(double theta1, double theta2)
    {
        // Lower Linear Actuator Angle & Length Calculations
        final double lambda = HardwareConstants.ARM_LOWER_ARM_LINEAR_ACTUATOR_LEFT_ANGLE_OFFSET;
        final double rho = HardwareConstants.ARM_LOWER_ARM_LINEAR_ACTUATOR_RIGHT_ANGLE_OFFSET;
        final double L10 = HardwareConstants.ARM_LOWER_ARM_TOP_PIN_OF_LINEAR_ACTUATOR_TO_PIN_ON_LOWER_ARM;
        final double L11 = HardwareConstants.ARM_LOWER_ARM_BOTTOM_PIN_OF_LINEAR_ACTUATOR_TO_PIN_ON_LOWER_ARM;

        double E1 = theta1 + rho - lambda;
        double L9 = Helpers.calculateLawOfCosinesDistance(L10, L11, E1);

        // Upper Linear Actuator Angle & Length Calculations
        final double phi = HardwareConstants.ARM_UPPER_ARM_PHI_ANGLE;
        final double L1 = HardwareConstants.ARM_UPPER_ARM_FOUR_BAR_FOLLOWER_PIN_DISTANCE;
        final double L2 = HardwareConstants.ARM_UPPER_ARM_FOUR_BAR_COUPLER_PIN_DISTANCE;
        final double L3 = HardwareConstants.ARM_UPPER_ARM_FOUR_BAR_DRIVER_PIN_DISTANCE;
        final double L4 = HardwareConstants.ARM_UPPER_ARM_FOUR_BAR_GROUND_PIN_DISTANCE;
        final double psi = HardwareConstants.ARM_UPPER_ARM_PSI_ANGLE;
        final double sigma = HardwareConstants.ARM_UPPER_ARM_SIGMA_ANGLE;
        final double L7 = HardwareConstants.ARM_UPPER_ARM_L7;
        final double L8 = HardwareConstants.ARM_UPPER_ARM_L8;

        double A1 = theta2 + phi;
        double L5 = Helpers.calculateLawOfCosinesDistance(L1, L4, A1);

        Double A4 = Helpers.calculateLawOfCosinesAngleOrNull(L3, L5, L2);
        if (A4 == null)
        {
            // must be impossible to reach this position with our current dimensions...
            return null;
        }

        Double A3 = Helpers.calculateLawOfCosinesAngleOrNull(L4, L5, L1);
        if (A3 == null)
        {
            // must be impossible to reach this position with our current dimensions...
            return null;
        }

        double B1 = 180.0 - A3 - A4 - phi - psi + sigma;
        double L6 = Helpers.calculateLawOfCosinesDistance(L7, L8, B1);

        double lowerLAExtension = L9 - HardwareConstants.ARM_LINEAR_ACTUATOR_RETRACTED_LENGTH;
        if (lowerLAExtension < -0.00001 || lowerLAExtension > HardwareConstants.ARM_EXTENTION_LENGTH)
        {
            // be forgiving of values that are just very slightly off...
            if (lowerLAExtension >= -0.00001)
            {
                lowerLAExtension = 0.0;
            }
            else if (lowerLAExtension <= HardwareConstants.ARM_EXTENTION_LENGTH + 0.00001)
            {
                lowerLAExtension = HardwareConstants.ARM_EXTENTION_LENGTH;
            }
            else
            {
                // must be impossible to reach this position with our current dimensions...
                return null;
            }
        }

        double upperLAExtension = L6 - HardwareConstants.ARM_LINEAR_ACTUATOR_RETRACTED_LENGTH;
        if (upperLAExtension < -0.00001 || upperLAExtension > HardwareConstants.ARM_EXTENTION_LENGTH)
        {
            // be forgiving of values that are just very slightly off...
            if (upperLAExtension >= -0.00001)
            {
                upperLAExtension = 0.0;
            }
            else if (upperLAExtension <= HardwareConstants.ARM_EXTENTION_LENGTH + 0.00001)
            {
                upperLAExtension = HardwareConstants.ARM_EXTENTION_LENGTH;
            }
            else
            {
                // must be impossible to reach this position with our current dimensions...
                return null;
            }
        }

        // in order lower, upper
        return new DoubleTuple(
            lowerLAExtension * HardwareConstants.ARM_STRING_ENCODER_TICKS_PER_INCH,
            upperLAExtension * HardwareConstants.ARM_STRING_ENCODER_TICKS_PER_INCH);
    }

    /**
     * Calculate the angles of the lower and upper arms given the extension of the linear actuators
     * @param lowerLAExtension amount the lower LA has been extended (in ticks)
     * @param upperLAExtension amount the upper LA has been extended (in ticks)
     * @return lower, upper arm angles (theta1, theta2)
     */
    static DoubleTuple calculateFKAnglesFromExtensions(double lowerLAExtension, double upperLAExtension)
    {
        double L9 = lowerLAExtension / HardwareConstants.ARM_STRING_ENCODER_TICKS_PER_INCH + HardwareConstants.ARM_LINEAR_ACTUATOR_RETRACTED_LENGTH;
        double L6 = upperLAExtension / HardwareConstants.ARM_STRING_ENCODER_TICKS_PER_INCH + HardwareConstants.ARM_LINEAR_ACTUATOR_RETRACTED_LENGTH;

        // Lower LA Distance FK
        final double lambda = HardwareConstants.ARM_LOWER_ARM_LINEAR_ACTUATOR_LEFT_ANGLE_OFFSET;
        final double rho = HardwareConstants.ARM_LOWER_ARM_LINEAR_ACTUATOR_RIGHT_ANGLE_OFFSET;
        final double L10 = HardwareConstants.ARM_LOWER_ARM_TOP_PIN_OF_LINEAR_ACTUATOR_TO_PIN_ON_LOWER_ARM;
        final double L11 = HardwareConstants.ARM_LOWER_ARM_BOTTOM_PIN_OF_LINEAR_ACTUATOR_TO_PIN_ON_LOWER_ARM;
        
        double E1 = Helpers.calculateLawOfCosinesAngle(L10, L11, L9);
        double theta1 = E1 - rho + lambda;

        // Upper LA Distance FK
        final double phi = HardwareConstants.ARM_UPPER_ARM_PHI_ANGLE;
        final double L1 = HardwareConstants.ARM_UPPER_ARM_FOUR_BAR_FOLLOWER_PIN_DISTANCE;
        final double L2 = HardwareConstants.ARM_UPPER_ARM_FOUR_BAR_COUPLER_PIN_DISTANCE;
        final double L3 = HardwareConstants.ARM_UPPER_ARM_FOUR_BAR_DRIVER_PIN_DISTANCE;
        final double L4 = HardwareConstants.ARM_UPPER_ARM_FOUR_BAR_GROUND_PIN_DISTANCE;
        final double psi = HardwareConstants.ARM_UPPER_ARM_PSI_ANGLE;
        final double sigma = HardwareConstants.ARM_UPPER_ARM_SIGMA_ANGLE;
        final double L7 = HardwareConstants.ARM_UPPER_ARM_L7;
        final double L8 = HardwareConstants.ARM_UPPER_ARM_L8;

        double B1Prime = Helpers.calculateLawOfCosinesAngle(L7, L8, L6);
        double A1Prime = 180 - phi - psi + sigma - B1Prime;
        double L5Prime = Helpers.calculateLawOfCosinesDistance(L4, L3, A1Prime);
        double A3Prime = Helpers.calculateLawOfCosinesAngle(L4, L5Prime, L3);
        double A4Prime = Helpers.calculateLawOfCosinesAngle(L1, L5Prime, L2);
        double theta2 = A4Prime + (A3Prime - phi);

        // in order lower, upper
        return new DoubleTuple(theta1, theta2);
    }
}