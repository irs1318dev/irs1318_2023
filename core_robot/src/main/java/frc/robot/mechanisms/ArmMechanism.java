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

    private boolean inSimpleMode;

    private double desiredLowerLeftArmPosition;
    private double desiredLowerRightArmPosition;
    private double desiredUpperArmPosition;

    private double armRetractionStartTime;

    //----------------- Intake Variables -----------------

    // private final ITalonSRX intakeMotor;
    // private final IDoubleSolenoid intakeExtender;

    // private enum IntakeState
    // {
    //     Retracted,
    //     Extended
    // };

    // private IntakeState currentIntakeState;

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
        this.lowerLeftArmPosition = TuningConstants.ARM_LOWER_FULL_EXTENTION_LENGTH * TuningConstants.ARM_STRING_ENCODER_TICKS_PER_INCH; // Fully Extended
        this.lowerRightArmPosition = TuningConstants.ARM_LOWER_FULL_EXTENTION_LENGTH * TuningConstants.ARM_STRING_ENCODER_TICKS_PER_INCH; // Fully Extended
        this.upperArmPosition = TuningConstants.ARM_UPPER_FULL_RETRACTED_LENGTH * TuningConstants.ARM_STRING_ENCODER_TICKS_PER_INCH; // Fully Retracted

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

        // this.intakeMotor = provider.getTalonSRX(ElectronicsConstants.INTAKE_MOTOR_CAN_ID);
        // this.intakeMotor.setControlMode(TalonXControlMode.PercentOutput);
        // this.intakeMotor.setInvertOutput(TuningConstants.ARM_INTAKE_MOTOR_INVERT_OUTPUT);
        // this.intakeMotor.setNeutralMode(MotorNeutralMode.Brake);

        // this.intakeExtender =
        //     provider.getDoubleSolenoid(
        //         ElectronicsConstants.PNEUMATICS_MODULE_A,
        //         ElectronicsConstants.PNEUMATICS_MODULE_TYPE_A,
        //         ElectronicsConstants.CARGO_INTAKE_PISTON_FORWARD,
        //         ElectronicsConstants.CARGO_INTAKE_PISTON_REVERSE);

        // this.currentIntakeState = IntakeState.Retracted;
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
        //                 this.upperArmPosition <= TuningConstants.ARM_UPPER_NEAR_FULL_RETRACTED_LENGTH * TuningConstants.ARM_STRING_ENCODER_TICKS_PER_INCH &&  // some value slightly larger than 0.0 inches
        //                 this.lowerLeftArmPosition >= TuningConstants.ARM_LOWER_NEAR_FULL_EXTENSION_LENGTH * TuningConstants.ARM_STRING_ENCODER_TICKS_PER_INCH) ||  // some value slightly smaller than fully extended
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
        //                 this.upperArmPosition <= TuningConstants.ARM_UPPER_NEAR_FULL_RETRACTED_LENGTH * TuningConstants.ARM_STRING_ENCODER_TICKS_PER_INCH &&  // some value slightly larger than 0.0 inches
        //                 this.lowerLeftArmPosition >= TuningConstants.ARM_LOWER_NEAR_FULL_EXTENSION_LENGTH * TuningConstants.ARM_STRING_ENCODER_TICKS_PER_INCH) ||  // some value slightly smaller than fully extended
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
        // double intakePower = TuningConstants.ZERO;
        // if (this.driver.getDigital(DigitalOperation.IntakeIn))
        // {
        //     intakePower = TuningConstants.ARM_INTAKE_POWER;
        // }
        // else if (this.driver.getDigital(DigitalOperation.IntakeOut))
        // {
        //     intakePower = -TuningConstants.ARM_INTAKE_POWER;
        // }

        // this.intakeMotor.set(intakePower);
        // this.logger.logNumber(LoggingKey.ArmIntakePower, intakePower);

        // // intake state transitions
        // if (this.driver.getDigital(DigitalOperation.IntakeExtend))
        // {
        //     this.currentIntakeState = IntakeState.Extended;
        // }
        // else if (this.driver.getDigital(DigitalOperation.IntakeRetract))
        // {
        //     this.currentIntakeState = IntakeState.Retracted;
        // }

        // this.logger.logBoolean(LoggingKey.ArmIntakeExtended, this.currentIntakeState == IntakeState.Extended);
        // switch (this.currentIntakeState)
        // {
        //     case Extended:
        //         this.intakeExtender.set(DoubleSolenoidValue.Forward);
        //         break;
        //     default:
        //     case Retracted:
        //         this.intakeExtender.set(DoubleSolenoidValue.Reverse);
        //         break;
        // }

        //----------------------------------- Main Arm -----------------------------------
        // if (this.curLeftFlipperState != ConeFlipperState.Retracted || this.curRightFlipperState != ConeFlipperState.Retracted)
        // {
        //     if (this.inSimpleMode)
        //     {
        //         //this.lowerLeftArm.set(TuningConstants.ARM_MAX_FORWARD_SIMPLE_VELOCITY);
        //         //this.lowerRightArm.set(TuningConstants.ARM_MAX_FORWARD_SIMPLE_VELOCITY);
        //         this.upperArm.set(TuningConstants.ARM_MAX_REVERSE_SIMPLE_VELOCITY);
        //     }
        //     else
        //     {
        //         //this.desiredLowerLeftArmPosition = TuningConstants.ARM_LOWER_FULL_EXTENTION_LENGTH * TuningConstants.ARM_STRING_ENCODER_TICKS_PER_INCH;
        //         //this.desiredLowerRightArmPosition = TuningConstants.ARM_LOWER_FULL_EXTENTION_LENGTH * TuningConstants.ARM_STRING_ENCODER_TICKS_PER_INCH;
        //         this.desiredUpperArmPosition = TuningConstants.ARM_UPPER_FULL_RETRACTED_LENGTH * TuningConstants.ARM_STRING_ENCODER_TICKS_PER_INCH;

        //         //this.lowerLeftArm.set(this.desiredLowerLeftArmPosition);
        //         //this.lowerRightArm.set(this.desiredLowerRightArmPosition);
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
                     ArmPositionSetpoint ikResult = ArmMechanism.calculateIK(this.driver.getAnalog(AnalogOperation.ArmIKXPosition), this.driver.getAnalog(AnalogOperation.ArmIKZPosition));
                    if (ikResult != null)
                    {
                        this.desiredLowerLeftArmPosition = ikResult.lowerPosition;
                        this.desiredLowerRightArmPosition = ikResult.lowerPosition;
                        this.desiredUpperArmPosition = ikResult.upperPosition;
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
                    this.desiredLowerLeftArmPosition += this.driver.getAnalog(AnalogOperation.ArmLowerPositionAdjustment) * TuningConstants.ARM_LOWER_EXTENSION_ADJUSTMENT_VELOCITY * TuningConstants.ARM_STRING_ENCODER_TICKS_PER_INCH * elapsedTime;
                    this.desiredLowerRightArmPosition += this.driver.getAnalog(AnalogOperation.ArmLowerPositionAdjustment) * TuningConstants.ARM_LOWER_EXTENSION_ADJUSTMENT_VELOCITY * TuningConstants.ARM_STRING_ENCODER_TICKS_PER_INCH * elapsedTime;
                    this.desiredUpperArmPosition += this.driver.getAnalog(AnalogOperation.ArmUpperPositionAdjustment) * TuningConstants.ARM_UPPER_EXTENSION_ADJUSTMENT_VELOCITY * TuningConstants.ARM_STRING_ENCODER_TICKS_PER_INCH * elapsedTime;
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

    static ArmAngleSetpoint calculateIKAngles(double x, double z)
    {
        // TODO: limit extension in calling function instead of here:
        // if (x > TuningConstants.ARM_MAX_IKX_EXTENSION_LENGTH || z > TuningConstants.ARM_MAX_IKZ_EXTENSION_HEIGHT)
        // {
        //     return null;
        // }

        // TODO: special-case fully-retracted in calling function instead of here:
        // if (x == 0.0 && z == 0.0)
        // {
        //     // special case fully-retracted
        //     return new ArmAngleSetpoint(0.0, 90.0);
        // }

        final double L1 = HardwareConstants.ARM_LOWER_ARM_LENGTH;
        final double L2 = HardwareConstants.ARM_UPPER_ARM_LENGTH;

        final double L1Squared = L1 * L1;
        final double L2Squared = L2 * L2;

        double rSquared = x * x + z * z;
        double cosineTheta2 = (L1Squared + L2Squared - rSquared) / (2.0 * L1 * L2);
        if (cosineTheta2 > 1.0 || cosineTheta2 < -1.0)
        {
            return null;
        }

        double theta2 = Helpers.acosd(cosineTheta2);
        double theta1 = Helpers.atan2d(z, x) + Helpers.acosd((L1Squared + rSquared - L2Squared) / (2.0 * L1 * Math.sqrt(rSquared)));

        return new ArmAngleSetpoint(theta2, theta1);
    }


    static ArmPositionSetpoint calculateIK(double x, double z)
    {
        if (x >= TuningConstants.ARM_MAX_IKX_EXTENSION_LENGTH && z >= TuningConstants.ARM_MAX_IKZ_EXTENSION_HEIGHT)
        {
            return null;
        }

        double lowerArmAngle = 90; // 90 if Starting straight up (Extended)
        double lowerArmAngleToMove = 0; // From current angle to desired angle
        double upperArmAngle = 0; //0 if retracted *angle is relative to upper arm angle*
        double upperArmAngleToMove = 0; // From current angle to desired angle

        lowerArmAngleToMove = Math.atan(
            (z / x)) +
            Math.atan((HardwareConstants.ARM_UPPER_ARM_LENGTH * Math.sin(upperArmAngleToMove))/
            (HardwareConstants.ARM_LOWER_ARM_LENGTH + (HardwareConstants.ARM_UPPER_ARM_LENGTH * Math.cos(upperArmAngleToMove))));

        upperArmAngleToMove = -Math.acos(
            ((x * x) +
            (z * z) -
            (HardwareConstants.ARM_LOWER_ARM_LENGTH * HardwareConstants.ARM_LOWER_ARM_LENGTH) -
            (HardwareConstants.ARM_UPPER_ARM_LENGTH * HardwareConstants.ARM_UPPER_ARM_LENGTH)) /
            (2 * HardwareConstants.ARM_UPPER_ARM_LENGTH * HardwareConstants.ARM_LOWER_ARM_LENGTH));

        double totalLowerArmAngle = lowerArmAngle +
            HardwareConstants.LOWER_ARM_LINEAR_ACTUATOR_RIGHT_ANGLE_OFFSET +
            HardwareConstants.LOWER_ARM_LINEAR_ACTUATOR_LEFT_ANGLE_OFFSET; // With offsets

        double lowerLinearActuatorDistanceToMove = 8; // Starting value of 8 inch, maybe (placeholder)
        double upperLinearActuatorDistanceToMove = 8; // Length 6 in Equation, Placeholder value
        lowerLinearActuatorDistanceToMove =
            (Math.sqrt(
                Math.pow(HardwareConstants.LOWER_ARM_TOP_PIN_OF_LINEAR_ACTUATOR_TO_PIN_ON_LOWER_ARM, 2) +
                Math.pow(HardwareConstants.LOWER_ARM_BOTTOM_PIN_OF_LINEAR_ACTUATOR_TO_PIN_ON_LOWER_ARM, 2) -
                2 * HardwareConstants.LOWER_ARM_TOP_PIN_OF_LINEAR_ACTUATOR_TO_PIN_ON_LOWER_ARM *
                HardwareConstants.LOWER_ARM_BOTTOM_PIN_OF_LINEAR_ACTUATOR_TO_PIN_ON_LOWER_ARM *
                Math.cos(totalLowerArmAngle)));
        //Upper Linear Actuator Angle & Length Calculations
        double phiAngle = Math.atan2(HardwareConstants.DISTANCE_ONE + HardwareConstants.DISTANCE_TWO, HardwareConstants.DISTANCE_FOUR);
        double angleOne = upperArmAngleToMove + phiAngle;
        double lengthFive = Math.sqrt(Math.pow(HardwareConstants.LENGTH_ONE, 2) + 
        Math.pow(HardwareConstants.LENGTH_FOUR, 2) - 
        2 * HardwareConstants.LENGTH_ONE * HardwareConstants.LENGTH_FOUR * 
        Math.cos(angleOne));
        double angleFour = Math.acos((Math.pow(HardwareConstants.LENGTH_THREE, 2) +
        Math.pow(lengthFive, 2) -
        Math.pow(HardwareConstants.LENGTH_TWO, 2)) / 
        2 * HardwareConstants.LENGTH_THREE * lengthFive);
        double angleThree = Math.acos((Math.pow(HardwareConstants.LENGTH_FOUR, 2) +
        Math.pow(lengthFive, 2) -
        Math.pow(HardwareConstants.LENGTH_ONE, 2)) / 
        2 * HardwareConstants.LENGTH_FOUR * lengthFive);
        double betaOne = 180 - angleThree - angleFour - phiAngle - HardwareConstants.PSI_ANGLE + HardwareConstants.SIGMA_ANGLE;
        double lengthSeven = 5.916; //Math.sqrt(HardwareConstants.DISTANCE_SIX + HardwareConstants.DISTANCE_FIVE);
        double lengthEight = 21.857; //Math.sqrt(HardwareConstants.DISTANCE_TWO + HardwareConstants.DISTANCE_THREE);
        upperLinearActuatorDistanceToMove = Math.sqrt(Math.pow(lengthSeven, 2) + Math.pow(lengthEight, 2) -
        2 * lengthSeven * lengthEight * Math.acos(betaOne));
        
        
        double lowerArmPosition = 0;
        double upperArmPosition = 0;
        lowerArmPosition *= TuningConstants.ARM_STRING_ENCODER_TICKS_PER_INCH * lowerLinearActuatorDistanceToMove;
        lowerArmPosition = (lowerArmPosition + 1) * 1016;
        upperArmPosition *= TuningConstants.ARM_STRING_ENCODER_TICKS_PER_INCH * upperLinearActuatorDistanceToMove;
        upperArmPosition = (upperArmPosition + 1) * 1016;

        return new ArmPositionSetpoint(upperArmPosition, lowerArmPosition);
    }

    static double calculateLawOfCosinesDistance(double adjacent1, double adjacent2, double angle)
    {
        double opposite;
        opposite = Math.sqrt(Math.pow(adjacent1, 2) + Math.pow(adjacent2, 2) - 2.0 * adjacent1 * adjacent2 * Math.cos(angle));
        return opposite;
    }

    static double calculateLawOfCosinesAngle(double adjacent1, double adjacent2, double opposite)
    {
        double angle;
        angle = Math.acos( (Math.pow(adjacent1, 2) + Math.pow(adjacent2, 2) - Math.pow(opposite, 2)) / 2.0 * adjacent1 * adjacent2);
        return angle;
    }
}