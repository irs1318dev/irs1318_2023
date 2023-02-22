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

    private double leftFlipperTransitionTime;
    private double rightFlipperTransitionTime;
    private enum ConeFlipperState
    {
        Extended,
        Extending,
        ExtendingWait,
        Retracted,
        Retracting,
    };

    private ConeFlipperState curLeftFlipperState;
    private ConeFlipperState curRightFlipperState;

    //----------------- Main Arm Variables -----------------

    private final ITalonSRX lowerLeftArmLinearActuator;
    private final ITalonSRX lowerRightArmLinearActuator;
    private final ITalonSRX upperArmLinearActuator;

    // Positions are in ticks, Velocities are in ticks per 100ms
    private double lowerLeftLAPosition;
    private double lowerLeftLAVelocity;
    private double lowerLeftLAError;
    private double lowerRightLAPosition;
    private double lowerRightLAVelocity;
    private double lowerRightLAError;
    private double upperLAPosition;
    private double upperLAVelocity;
    private double upperLAError;

    // actual positions calculated using forward kinematics
    private double xPosition;
    private double zPosition;

    private boolean throughBeamBroken;

    private boolean inSimpleMode;

    private double desiredLowerLeftLAPosition;
    private double desiredLowerRightLAPosition;
    private double desiredUpperLAPosition;

    // converted into desired Lower/Upper positions, but here for when we adjust X/Z position manually
    private double desiredXPosition;
    private double desiredZPosition;

    private double lowerSetpointChangedTime;
    private double upperSetpointChangedTime;
    private boolean lowerLAsStalled;
    private boolean upperLAsStalled;

    private FloatingAverageCalculator lowerLeftLAPowerAverageCalculator;
    private FloatingAverageCalculator lowerRightLAPowerAverageCalculator;
    private FloatingAverageCalculator upperLAsPowerAverageCalculator;

    private double lowerLeftLAPowerAverage;
    private double lowerRightLAPowerAverage;
    private double upperLAsPowerAverage;

    private FloatingAverageCalculator lowerLeftLAVelocityAverageCalculator;
    private FloatingAverageCalculator lowerRightLAVelocityAverageCalculator;
    private FloatingAverageCalculator upperLAVelocityAverageCalculator;

    private double lowerLeftLAVelocityAverage;
    private double lowerRightLAVelocityAverage;
    private double upperLAVelocityAverage;

    //----------------- Intake Variables -----------------

    private final ITalonSRX intakeMotor;
    private final IDoubleSolenoid intakeExtender;

    // private final IAnalogInput intakeThroughBeamSensor;
    

    private enum IntakeState
    {
        Retracted,
        Extended
    };
    
    private double throughBeamValue;
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

        this.lowerLeftArmLinearActuator = provider.getTalonSRX(ElectronicsConstants.ARM_LOWER_LEFT_LA_CAN_ID);
        this.lowerRightArmLinearActuator = provider.getTalonSRX(ElectronicsConstants.ARM_LOWER_RIGHT_LA_CAN_ID);
        this.upperArmLinearActuator = provider.getTalonSRX(ElectronicsConstants.ARM_UPPER_LA_CAN_ID);

        this.inSimpleMode = TuningConstants.ARM_USE_SIMPLE_MODE;

        if (TuningConstants.ARM_USE_MM)
        {
            this.lowerLeftArmLinearActuator.setMotionMagicPIDF(
                TuningConstants.ARM_LOWER_LEFT_POSITION_MM_PID_KP,
                TuningConstants.ARM_LOWER_LEFT_POSITION_MM_PID_KI,
                TuningConstants.ARM_LOWER_LEFT_POSITION_MM_PID_KD,
                TuningConstants.ARM_LOWER_LEFT_POSITION_MM_PID_KF,
                TuningConstants.ARM_LOWER_LEFT_POSITION_MM_CRUISE_VELOCITY,
                TuningConstants.ARM_LOWER_LEFT_POSITION_MM_ACCELERATION,
                ArmMechanism.defaultPidSlotId);
            this.lowerRightArmLinearActuator.setMotionMagicPIDF(
                TuningConstants.ARM_LOWER_RIGHT_POSITION_MM_PID_KP,
                TuningConstants.ARM_LOWER_RIGHT_POSITION_MM_PID_KI,
                TuningConstants.ARM_LOWER_RIGHT_POSITION_MM_PID_KD,
                TuningConstants.ARM_LOWER_RIGHT_POSITION_MM_PID_KF,
                TuningConstants.ARM_LOWER_RIGHT_POSITION_MM_CRUISE_VELOCITY,
                TuningConstants.ARM_LOWER_RIGHT_POSITION_MM_ACCELERATION,
                ArmMechanism.defaultPidSlotId);

            this.upperArmLinearActuator.setMotionMagicPIDF(
                TuningConstants.ARM_UPPER_POSITION_MM_PID_KP,
                TuningConstants.ARM_UPPER_POSITION_MM_PID_KI,
                TuningConstants.ARM_UPPER_POSITION_MM_PID_KD,
                TuningConstants.ARM_UPPER_POSITION_MM_PID_KF,
                TuningConstants.ARM_UPPER_POSITION_MM_CRUISE_VELOCITY,
                TuningConstants.ARM_UPPER_POSITION_MM_ACCELERATION,
                ArmMechanism.defaultPidSlotId);
        }
        else
        {
            this.lowerLeftArmLinearActuator.setPIDF(
                TuningConstants.ARM_LOWER_LEFT_POSITION_PID_KP,
                TuningConstants.ARM_LOWER_LEFT_POSITION_PID_KI,
                TuningConstants.ARM_LOWER_LEFT_POSITION_PID_KD,
                TuningConstants.ARM_LOWER_LEFT_POSITION_PID_KF,
                ArmMechanism.defaultPidSlotId);
            this.lowerRightArmLinearActuator.setPIDF(
                TuningConstants.ARM_LOWER_RIGHT_POSITION_PID_KP,
                TuningConstants.ARM_LOWER_RIGHT_POSITION_PID_KI,
                TuningConstants.ARM_LOWER_RIGHT_POSITION_PID_KD,
                TuningConstants.ARM_LOWER_RIGHT_POSITION_PID_KF,
                ArmMechanism.defaultPidSlotId);

            this.upperArmLinearActuator.setPIDF(
                TuningConstants.ARM_UPPER_POSITION_PID_KP,
                TuningConstants.ARM_UPPER_POSITION_PID_KI,
                TuningConstants.ARM_UPPER_POSITION_PID_KD,
                TuningConstants.ARM_UPPER_POSITION_PID_KF,
                ArmMechanism.defaultPidSlotId);
        }

        this.lowerLeftLAVelocity = 0.0;
        this.lowerRightLAVelocity = 0.0;
        this.upperLAVelocity = 0.0;
        this.lowerLeftLAPosition = HardwareConstants.ARM_EXTENTION_LENGTH * HardwareConstants.ARM_STRING_ENCODER_TICKS_PER_INCH; // Fully Extended
        this.lowerRightLAPosition = HardwareConstants.ARM_EXTENTION_LENGTH * HardwareConstants.ARM_STRING_ENCODER_TICKS_PER_INCH; // Fully Extended
        this.upperLAPosition = 0.0; // Fully Retracted
        this.xPosition = TuningConstants.ARM_FULLY_RETRACTED_X_POSITION;
        this.zPosition = TuningConstants.ARM_FULLY_RETRACTED_Z_POSITION;

        this.desiredLowerLeftLAPosition = this.lowerLeftLAPosition;
        this.desiredLowerRightLAPosition = this.lowerRightLAPosition;
        this.desiredUpperLAPosition = this.upperLAPosition;

        this.lowerLeftArmLinearActuator.setSelectedSlot(ArmMechanism.defaultPidSlotId);
        this.lowerRightArmLinearActuator.setSelectedSlot(ArmMechanism.defaultPidSlotId);
        this.upperArmLinearActuator.setSelectedSlot(ArmMechanism.defaultPidSlotId);

        this.lowerLeftArmLinearActuator.setSensorType(TalonXFeedbackDevice.QuadEncoder);
        this.lowerRightArmLinearActuator.setSensorType(TalonXFeedbackDevice.QuadEncoder);
        this.upperArmLinearActuator.setSensorType(TalonXFeedbackDevice.QuadEncoder);

        this.lowerLeftArmLinearActuator.setInvertOutput(TuningConstants.ARM_LOWER_LEFT_INVERT_OUTPUT);
        this.lowerRightArmLinearActuator.setInvertOutput(TuningConstants.ARM_LOWER_RIGHT_INVERT_OUTPUT);
        this.upperArmLinearActuator.setInvertOutput(TuningConstants.ARM_UPPER_INVERT_OUTPUT);

        this.lowerLeftArmLinearActuator.setInvertSensor(TuningConstants.ARM_LOWER_LEFT_INVERT_SENSOR);
        this.lowerRightArmLinearActuator.setInvertSensor(TuningConstants.ARM_LOWER_RIGHT_INVERT_SENSOR);
        this.upperArmLinearActuator.setInvertSensor(TuningConstants.ARM_UPPER_INVERT_SENSOR);

        this.lowerLeftArmLinearActuator.setNeutralMode(MotorNeutralMode.Brake);
        this.lowerRightArmLinearActuator.setNeutralMode(MotorNeutralMode.Brake);
        this.upperArmLinearActuator.setNeutralMode(MotorNeutralMode.Brake);

        if (this.inSimpleMode)
        {
            this.lowerLeftArmLinearActuator.setControlMode(TalonXControlMode.PercentOutput);
            this.lowerRightArmLinearActuator.setControlMode(TalonXControlMode.PercentOutput);
            this.upperArmLinearActuator.setControlMode(TalonXControlMode.PercentOutput);
        }
        else
        {
            if (TuningConstants.ARM_USE_MM)
            {
                this.lowerLeftArmLinearActuator.setControlMode(TalonXControlMode.MotionMagicPosition);
                this.lowerRightArmLinearActuator.setControlMode(TalonXControlMode.MotionMagicPosition);
                this.upperArmLinearActuator.setControlMode(TalonXControlMode.MotionMagicPosition);
            }
            else
            {
                this.lowerLeftArmLinearActuator.setControlMode(TalonXControlMode.Position);
                this.lowerRightArmLinearActuator.setControlMode(TalonXControlMode.Position);
                this.upperArmLinearActuator.setControlMode(TalonXControlMode.Position);
            }
        }

        this.lowerLeftArmLinearActuator.setPosition(0.0);
        this.lowerRightArmLinearActuator.setPosition(0.0);
        this.upperArmLinearActuator.setPosition(0.0);

        this.lowerLeftLAPowerAverageCalculator = new FloatingAverageCalculator(this.timer, TuningConstants.ARM_POWER_TRACKING_DURATION, TuningConstants.ARM_POWER_SAMPLES_PER_SECOND);
        this.lowerRightLAPowerAverageCalculator = new FloatingAverageCalculator(this.timer, TuningConstants.ARM_POWER_TRACKING_DURATION, TuningConstants.ARM_POWER_SAMPLES_PER_SECOND);
        this.upperLAsPowerAverageCalculator = new FloatingAverageCalculator(this.timer, TuningConstants.ARM_POWER_TRACKING_DURATION, TuningConstants.ARM_POWER_SAMPLES_PER_SECOND);

        this.lowerLeftLAVelocityAverageCalculator = new FloatingAverageCalculator(this.timer, TuningConstants.ARM_VELOCITY_TRACKING_DURATION, TuningConstants.ARM_VELOCITY_SAMPLES_PER_SECOND);
        this.lowerRightLAVelocityAverageCalculator = new FloatingAverageCalculator(this.timer, TuningConstants.ARM_VELOCITY_TRACKING_DURATION, TuningConstants.ARM_VELOCITY_SAMPLES_PER_SECOND);
        this.upperLAVelocityAverageCalculator = new FloatingAverageCalculator(this.timer, TuningConstants.ARM_VELOCITY_TRACKING_DURATION, TuningConstants.ARM_VELOCITY_SAMPLES_PER_SECOND);

        ITalonSRX upperLAFollower = provider.getTalonSRX(ElectronicsConstants.ARM_UPPER_LA_FOLLOWER_CAN_ID);
        upperLAFollower.setNeutralMode(MotorNeutralMode.Brake);
        upperLAFollower.setInvertOutput(TuningConstants.ARM_UPPER_FOLLOWER_INVERT_OUTPUT);
        upperLAFollower.follow(this.upperArmLinearActuator);

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

        this.leftFlipperTransitionTime = 0.0;
        this.rightFlipperTransitionTime = 0.0;

        this.curRightFlipperState = ConeFlipperState.Retracted;
        this.curLeftFlipperState = ConeFlipperState.Retracted;

        //-------------------------- Intake Initialization ----------------------------------

        this.intakeMotor = provider.getTalonSRX(ElectronicsConstants.ARM_INTAKE_MOTOR_CAN_ID);
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

        // this.intakeThroughBeamSensor = provider.getAnalogInput(ElectronicsConstants.INTAKE_LIGHT_FEEDER);
    }


    @Override
    public void readSensors()
    {
        this.lowerLeftLAPosition = this.lowerLeftArmLinearActuator.getPosition() + HardwareConstants.ARM_EXTENTION_LENGTH * HardwareConstants.ARM_STRING_ENCODER_TICKS_PER_INCH;
        this.lowerLeftLAVelocity = this.lowerLeftArmLinearActuator.getVelocity();
        this.lowerLeftLAError = this.lowerLeftArmLinearActuator.getError();
        this.lowerRightLAPosition = this.lowerRightArmLinearActuator.getPosition() + HardwareConstants.ARM_EXTENTION_LENGTH * HardwareConstants.ARM_STRING_ENCODER_TICKS_PER_INCH;
        this.lowerRightLAVelocity = this.lowerRightArmLinearActuator.getVelocity();
        this.lowerRightLAError = this.lowerRightArmLinearActuator.getError();
        this.upperLAPosition = this.upperArmLinearActuator.getPosition();
        this.upperLAVelocity = this.upperArmLinearActuator.getVelocity();
        this.upperLAError = this.upperArmLinearActuator.getError();

        // this.intakeSensorValue = this.intakeThroughBeamSensor.getVoltage();
        // this.throughBeamBroken = this.intakeSensorValue < TuningConstants.FEEDER_LIGHT_CUTOFF_VALUE;

        double lowerLeftLAPower = this.powerManager.getCurrent(ElectronicsConstants.ARM_LOWER_LEFT_LA_PDH_CHANNEL);
        double lowerRightLAPower = this.powerManager.getCurrent(ElectronicsConstants.ARM_LOWER_RIGHT_LA_PDH_CHANNEL);
        double upperLeftLAPower = this.powerManager.getCurrent(ElectronicsConstants.ARM_UPPER_LEFT_LA_PDH_CHANNEL);
        double upperRightLAPower = this.powerManager.getCurrent(ElectronicsConstants.ARM_UPPER_RIGHT_LA_PDH_CHANNEL);
        
        this.lowerLeftLAPowerAverage = this.lowerLeftLAPowerAverageCalculator.update(lowerLeftLAPower);
        this.lowerRightLAPowerAverage = this.lowerRightLAPowerAverageCalculator.update(lowerRightLAPower);
        this.upperLAsPowerAverage = this.upperLAsPowerAverageCalculator.update((upperLeftLAPower + upperRightLAPower) / 2.0);

        this.lowerLeftLAVelocityAverage = this.lowerLeftLAVelocityAverageCalculator.update(this.lowerLeftLAVelocity);
        this.lowerRightLAVelocityAverage = this.lowerRightLAVelocityAverageCalculator.update(this.lowerRightLAVelocity);
        this.upperLAVelocityAverage = this.upperLAVelocityAverageCalculator.update(this.upperLAVelocity);

        this.logger.logNumber(LoggingKey.ArmLowerLeftPosition, this.lowerLeftLAPosition);
        this.logger.logNumber(LoggingKey.ArmLowerLeftVelocity, this.lowerLeftLAVelocity);
        this.logger.logNumber(LoggingKey.ArmLowerLeftVelocityAverage, this.lowerLeftLAVelocityAverage);
        this.logger.logNumber(LoggingKey.ArmLowerLeftError, this.lowerLeftLAError);
        this.logger.logNumber(LoggingKey.ArmLowerLeftPower, this.lowerLeftLAPowerAverage);
        this.logger.logNumber(LoggingKey.ArmLowerRightPosition, this.lowerRightLAPosition);
        this.logger.logNumber(LoggingKey.ArmLowerRightVelocity, this.lowerRightLAVelocity);
        this.logger.logNumber(LoggingKey.ArmLowerRightVelocityAverage, this.lowerRightLAVelocityAverage);
        this.logger.logNumber(LoggingKey.ArmLowerRightError, this.lowerRightLAError);
        this.logger.logNumber(LoggingKey.ArmLowerRightPower, this.lowerRightLAPowerAverage);
        this.logger.logNumber(LoggingKey.ArmUpperPosition, this.upperLAPosition);
        this.logger.logNumber(LoggingKey.ArmUpperVelocity, this.upperLAVelocity);
        this.logger.logNumber(LoggingKey.ArmUpperVelocityAverage, this.upperLAVelocityAverage);
        this.logger.logNumber(LoggingKey.ArmUpperError, this.upperLAError);
        this.logger.logNumber(LoggingKey.ArmUpperPower, this.upperLAsPowerAverage);
        this.logger.logNumber(LoggingKey.ArmIntakeThroughBeamRaw, this.throughBeamValue);
        this.logger.logBoolean(LoggingKey.ArmIntakeSensorBroken, this.throughBeamBroken);

        DoubleTuple offsets = ArmMechanism.calculateFK(
            (this.lowerLeftLAPosition + this.lowerRightLAPosition) / 2.0,
            this.upperLAPosition);

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

            this.lowerLeftArmLinearActuator.setControlMode(TalonXControlMode.PercentOutput);
            this.lowerRightArmLinearActuator.setControlMode(TalonXControlMode.PercentOutput);
            this.upperArmLinearActuator.setControlMode(TalonXControlMode.PercentOutput);
        }
        else if (this.driver.getDigital(DigitalOperation.ArmDisableSimpleMode))
        {
            this.inSimpleMode = false;

            if (TuningConstants.ARM_USE_MM)
            {
                this.lowerLeftArmLinearActuator.setControlMode(TalonXControlMode.MotionMagicPosition);
                this.lowerRightArmLinearActuator.setControlMode(TalonXControlMode.MotionMagicPosition);
                this.upperArmLinearActuator.setControlMode(TalonXControlMode.MotionMagicPosition);
            }
            else
            {
                this.lowerLeftArmLinearActuator.setControlMode(TalonXControlMode.Position);
                this.lowerRightArmLinearActuator.setControlMode(TalonXControlMode.Position);
                this.upperArmLinearActuator.setControlMode(TalonXControlMode.Position);
            }

            this.lowerLeftArmLinearActuator.setSelectedSlot(ArmMechanism.defaultPidSlotId);
            this.lowerRightArmLinearActuator.setSelectedSlot(ArmMechanism.defaultPidSlotId);
            this.upperArmLinearActuator.setSelectedSlot(ArmMechanism.defaultPidSlotId);

            this.desiredLowerLeftLAPosition = this.lowerLeftLAPosition;
            this.desiredLowerRightLAPosition = this.lowerRightLAPosition;
            this.desiredUpperLAPosition = this.upperLAPosition;

            this.lowerSetpointChangedTime = currTime;
            this.upperSetpointChangedTime = currTime;

            this.lowerLAsStalled = false;
            this.upperLAsStalled = false;
        }

        if (this.driver.getDigital(DigitalOperation.ArmForceReset))
        {
            this.lowerLeftArmLinearActuator.setPosition(0.0);
            this.lowerRightArmLinearActuator.setPosition(0.0);
            this.upperArmLinearActuator.setPosition(0.0);

            this.lowerLeftLAPosition = HardwareConstants.ARM_EXTENTION_LENGTH * HardwareConstants.ARM_STRING_ENCODER_TICKS_PER_INCH; // Fully Extended
            this.lowerRightLAPosition = HardwareConstants.ARM_EXTENTION_LENGTH * HardwareConstants.ARM_STRING_ENCODER_TICKS_PER_INCH; // Fully Extended
            this.upperLAPosition = 0.0; // Fully Retracted

            this.desiredLowerLeftLAPosition = this.lowerLeftLAPosition;
            this.desiredLowerRightLAPosition = this.lowerRightLAPosition;
            this.desiredUpperLAPosition = this.upperLAPosition;

            this.lowerSetpointChangedTime = currTime;
            this.upperSetpointChangedTime = currTime;

            this.lowerLAsStalled = false;
            this.upperLAsStalled = false;
        }

        //----------------------------------- Flippers -----------------------------------
        boolean extendRightFlipper = this.driver.getDigital(DigitalOperation.ExtendRightConeFlipper);
        boolean extendLeftFlipper = this.driver.getDigital(DigitalOperation.ExtendLeftConeFlipper);
        switch (this.curRightFlipperState)
        {
            case Retracted:
                if (extendRightFlipper)
                {
                    this.curRightFlipperState = ConeFlipperState.ExtendingWait;
                }

                break;

            case ExtendingWait:
                if (!extendRightFlipper)
                {
                    this.curRightFlipperState = ConeFlipperState.Retracted;
                }
                else if (this.curLeftFlipperState == ConeFlipperState.Retracted &&
                    (this.inSimpleMode ||
                        (this.upperLAPosition <= TuningConstants.ARM_NEAR_FULL_RETRACTED_LENGTH * HardwareConstants.ARM_STRING_ENCODER_TICKS_PER_INCH &&
                        this.lowerLeftLAPosition >= TuningConstants.ARM_NEAR_FULL_EXTENSION_LENGTH * HardwareConstants.ARM_STRING_ENCODER_TICKS_PER_INCH)))
                {
                    this.curRightFlipperState = ConeFlipperState.Extending;
                    this.rightFlipperTransitionTime = currTime;
                }

                break;

            case Extending:
                if (!extendRightFlipper)
                {
                    this.curRightFlipperState = ConeFlipperState.Retracting;
                    this.rightFlipperTransitionTime = currTime;
                }
                else if (currTime - this.rightFlipperTransitionTime > TuningConstants.ARM_FLIPPER_EXTEND_WAIT_DURATION)
                {
                    this.curRightFlipperState = ConeFlipperState.Extended;
                }

                break;

            case Extended:
                if (!extendRightFlipper)
                {
                    this.curRightFlipperState = ConeFlipperState.Retracting;
                    this.rightFlipperTransitionTime = currTime;
                }

                break;

            case Retracting:
                if (extendRightFlipper)
                {
                    this.curRightFlipperState = ConeFlipperState.Extending;
                    this.rightFlipperTransitionTime = currTime;
                }
                else if (currTime - this.rightFlipperTransitionTime > TuningConstants.ARM_FLIPPER_RETRACT_WAIT_DURATION)
                {
                    this.curRightFlipperState = ConeFlipperState.Retracted;
                }

                break;
        }

        switch (this.curLeftFlipperState)
        {
            case Retracted:
                if (extendLeftFlipper)
                {
                    this.curLeftFlipperState = ConeFlipperState.ExtendingWait;
                }

                break;

            case ExtendingWait:
                if (!extendLeftFlipper)
                {
                    this.curLeftFlipperState = ConeFlipperState.Retracted;
                }
                else if (this.curLeftFlipperState == ConeFlipperState.Retracted &&
                    (this.inSimpleMode ||
                        (this.upperLAPosition <= TuningConstants.ARM_NEAR_FULL_RETRACTED_LENGTH * HardwareConstants.ARM_STRING_ENCODER_TICKS_PER_INCH &&
                        this.lowerLeftLAPosition >= TuningConstants.ARM_NEAR_FULL_EXTENSION_LENGTH * HardwareConstants.ARM_STRING_ENCODER_TICKS_PER_INCH)))
                {
                    this.curLeftFlipperState = ConeFlipperState.Extending;
                    this.leftFlipperTransitionTime = currTime;
                }

                break;

            case Extending:
                if (!extendLeftFlipper)
                {
                    this.curLeftFlipperState = ConeFlipperState.Retracting;
                    this.leftFlipperTransitionTime = currTime;
                }
                else if (currTime - this.leftFlipperTransitionTime > TuningConstants.ARM_FLIPPER_EXTEND_WAIT_DURATION)
                {
                    this.curLeftFlipperState = ConeFlipperState.Extended;
                }

                break;

            case Extended:
                if (!extendLeftFlipper)
                {
                    this.curLeftFlipperState = ConeFlipperState.Retracting;
                    this.leftFlipperTransitionTime = currTime;
                }

                break;

            case Retracting:
                if (extendLeftFlipper)
                {
                    this.curLeftFlipperState = ConeFlipperState.Extending;
                    this.leftFlipperTransitionTime = currTime;
                }
                else if (currTime - this.leftFlipperTransitionTime > TuningConstants.ARM_FLIPPER_RETRACT_WAIT_DURATION)
                {
                    this.curLeftFlipperState = ConeFlipperState.Retracted;
                }

                break;
        }

        this.logger.logString(LoggingKey.ArmRightFlipperState, this.curRightFlipperState.toString());
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

        this.logger.logString(LoggingKey.ArmLeftFlipperState, this.curLeftFlipperState.toString());
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
        if (this.driver.getDigital(DigitalOperation.IntakeRelease))
        {
            this.currentIntakeState = IntakeState.Extended;
        }
        else if (this.driver.getDigital(DigitalOperation.IntakeGrab))
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
        if (this.inSimpleMode)
        {
            // controlled by joysticks
            double lowerArmPower = this.driver.getAnalog(AnalogOperation.ArmLowerPositionAdjustment);
            double upperArmPower = this.driver.getAnalog(AnalogOperation.ArmUpperPositionAdjustment);
            this.lowerLeftArmLinearActuator.set(lowerArmPower);
            this.lowerRightArmLinearActuator.set(lowerArmPower);
            this.upperArmLinearActuator.set(upperArmPower);

            this.lowerSetpointChangedTime = currTime;
            this.upperSetpointChangedTime = currTime;

            this.lowerLAsStalled = false;
            this.upperLAsStalled = false;
        }
        else if (this.curLeftFlipperState == ConeFlipperState.Retracted || this.curRightFlipperState == ConeFlipperState.Retracted)
        {
            double elapsedTime = currTime - this.prevTime;

            // first choice
            double ikX = TuningConstants.MAGIC_NULL_VALUE;
            double ikZ = TuningConstants.MAGIC_NULL_VALUE;

            // second choice
            double ikXAdjustment = 0.0;
            double ikZAdjustment = 0.0;

            if (TuningConstants.ARM_USE_IK)
            {
                ikX = this.driver.getAnalog(AnalogOperation.ArmIKXPosition);
                ikZ = this.driver.getAnalog(AnalogOperation.ArmIKZPosition);

                ikXAdjustment = this.driver.getAnalog(AnalogOperation.ArmIKXAdjustment) * TuningConstants.ARM_X_POSITION_ADJUSTMENT_VELOCITY * elapsedTime;
                ikZAdjustment = this.driver.getAnalog(AnalogOperation.ArmIKZAdjustment) * TuningConstants.ARM_Z_POSITION_ADJUSTMENT_VELOCITY * elapsedTime;
            }

            // third choice
            double newDesiredLowerPosition = this.driver.getAnalog(AnalogOperation.ArmMMLowerPosition);
            double newDesiredUpperPosition = this.driver.getAnalog(AnalogOperation.ArmMMUpperPosition);

            // fourth choice
            double lowerPositionAdjustment = this.driver.getAnalog(AnalogOperation.ArmLowerPositionAdjustment) * TuningConstants.ARM_LOWER_EXTENSION_ADJUSTMENT_VELOCITY * HardwareConstants.ARM_STRING_ENCODER_TICKS_PER_INCH * elapsedTime;
            double upperPositionAdjustment = this.driver.getAnalog(AnalogOperation.ArmUpperPositionAdjustment) * TuningConstants.ARM_UPPER_EXTENSION_ADJUSTMENT_VELOCITY * HardwareConstants.ARM_STRING_ENCODER_TICKS_PER_INCH * elapsedTime;

            if (ikX != TuningConstants.MAGIC_NULL_VALUE &&
                ikZ != TuningConstants.MAGIC_NULL_VALUE)
            {
                // controlled by macro
                DoubleTuple ikResult = ArmMechanism.calculateIK(ikX, ikZ);
                if (ikResult != null)
                {
                    boolean updateDesiredIKPosition = false;
                    if (Helpers.RoughEquals(this.desiredLowerLeftLAPosition, ikResult.first, 0.1) ||
                        Helpers.RoughEquals(this.desiredLowerRightLAPosition, ikResult.first, 0.1))
                    {
                        this.lowerSetpointChangedTime = currTime;
                        this.lowerLAsStalled = false;

                        this.desiredLowerLeftLAPosition = ikResult.first;
                        this.desiredLowerRightLAPosition = ikResult.first;
                        updateDesiredIKPosition = true;
                    }

                    if (Helpers.RoughEquals(this.desiredUpperLAPosition, ikResult.second, 0.1))
                    {
                        this.upperSetpointChangedTime = currTime;
                        this.upperLAsStalled = false;

                        this.desiredUpperLAPosition = ikResult.second;
                        updateDesiredIKPosition = true;
                    }

                    if (updateDesiredIKPosition)
                    {
                        this.desiredXPosition = ikX;
                        this.desiredZPosition = ikZ;
                    }
                }
            }
            else if (ikXAdjustment != 0.0 || ikZAdjustment != 0.0)
            {
                // controlled by joysticks
                double newDesiredXPosition = this.desiredXPosition + ikXAdjustment;
                double newDesiredZPosition = this.desiredZPosition + ikZAdjustment;

                DoubleTuple ikResult = ArmMechanism.calculateIK(ikX, ikZ);
                if (ikResult != null)
                {
                    boolean updateDesiredIKPosition = false;
                    if (Helpers.RoughEquals(this.desiredLowerLeftLAPosition, ikResult.first, 0.1) ||
                        Helpers.RoughEquals(this.desiredLowerRightLAPosition, ikResult.first, 0.1))
                    {
                        this.lowerSetpointChangedTime = currTime;
                        this.lowerLAsStalled = false;

                        this.desiredLowerLeftLAPosition = ikResult.first;
                        this.desiredLowerRightLAPosition = ikResult.first;
                        updateDesiredIKPosition = true;
                    }

                    if (Helpers.RoughEquals(this.desiredUpperLAPosition, ikResult.second, 0.1))
                    {
                        this.upperSetpointChangedTime = currTime;
                        this.upperLAsStalled = false;

                        this.desiredUpperLAPosition = ikResult.second;
                        updateDesiredIKPosition = true;
                    }

                    if (updateDesiredIKPosition)
                    {
                        this.desiredXPosition = newDesiredXPosition;
                        this.desiredZPosition = newDesiredZPosition;
                    }
                }
            }
            else if (newDesiredLowerPosition != TuningConstants.MAGIC_NULL_VALUE ||
                newDesiredUpperPosition != TuningConstants.MAGIC_NULL_VALUE)
            {
                // controlled by macro
                boolean updateDesiredIKPosition = false;
                if (newDesiredLowerPosition != TuningConstants.MAGIC_NULL_VALUE &&
                    (!Helpers.RoughEquals(this.desiredLowerLeftLAPosition, newDesiredLowerPosition, 0.1) ||
                        !Helpers.RoughEquals(this.desiredLowerRightLAPosition, newDesiredLowerPosition, 0.1)))
                {
                    this.lowerSetpointChangedTime = currTime;
                    this.lowerLAsStalled = false;

                    this.desiredLowerLeftLAPosition = newDesiredLowerPosition;
                    this.desiredLowerRightLAPosition = newDesiredLowerPosition;
                    updateDesiredIKPosition = true;
                }

                if (newDesiredUpperPosition != TuningConstants.MAGIC_NULL_VALUE &&
                    !Helpers.RoughEquals(this.desiredUpperLAPosition, newDesiredUpperPosition, 0.1))
                {
                    this.upperSetpointChangedTime = currTime;
                    this.upperLAsStalled = false;

                    this.desiredUpperLAPosition = newDesiredUpperPosition;
                    updateDesiredIKPosition = true;
                }

                if (updateDesiredIKPosition)
                {
                    DoubleTuple fkResult = ArmMechanism.calculateFK(
                        (this.desiredLowerLeftLAPosition + this.desiredLowerRightLAPosition) / 2.0,
                        this.desiredUpperLAPosition);
                    if (fkResult != null)
                    {
                        this.desiredXPosition = fkResult.first;
                        this.desiredZPosition = fkResult.second;
                    }
                }
            }
            else if (lowerPositionAdjustment != 0.0 || upperPositionAdjustment != 0.0)
            {
                // controlled by joysticks
                this.desiredLowerLeftLAPosition += lowerPositionAdjustment;
                this.desiredLowerRightLAPosition += lowerPositionAdjustment;
                this.desiredUpperLAPosition += upperPositionAdjustment;

                this.desiredLowerLeftLAPosition = Helpers.EnforceRange(this.desiredLowerLeftLAPosition, 0.0, TuningConstants.ARM_LOWER_MAX_EXTENSION_LENGTH * HardwareConstants.ARM_STRING_ENCODER_TICKS_PER_INCH);
                this.desiredLowerRightLAPosition = Helpers.EnforceRange(this.desiredLowerRightLAPosition, 0.0, TuningConstants.ARM_LOWER_MAX_EXTENSION_LENGTH * HardwareConstants.ARM_STRING_ENCODER_TICKS_PER_INCH);
                this.desiredUpperLAPosition = Helpers.EnforceRange(this.desiredUpperLAPosition, 0.0, TuningConstants.ARM_UPPER_MAX_EXTENSION_LENGTH * HardwareConstants.ARM_STRING_ENCODER_TICKS_PER_INCH);

                boolean updateDesiredIKPosition = false;
                if (lowerPositionAdjustment != 0.0)
                {
                    this.lowerSetpointChangedTime = currTime;
                    this.lowerLAsStalled = false;
                    updateDesiredIKPosition = true;
                }

                if (upperPositionAdjustment != 0.0)
                {
                    this.upperSetpointChangedTime = currTime;
                    this.upperLAsStalled = false;
                    updateDesiredIKPosition = true;
                }

                if (updateDesiredIKPosition)
                {
                    DoubleTuple fkResult = ArmMechanism.calculateFK(
                        (this.desiredLowerLeftLAPosition + this.desiredLowerRightLAPosition) / 2.0,
                        this.desiredUpperLAPosition);
                    if (fkResult != null)
                    {
                        this.desiredXPosition = fkResult.first;
                        this.desiredZPosition = fkResult.second;
                    }
                }
            }
        }

        if (TuningConstants.ARM_STALL_PROTECTION_ENABLED)
        {
            if (currTime > this.lowerSetpointChangedTime + TuningConstants.ARM_VELOCITY_TRACKING_DURATION &&
                this.lowerLeftLAPowerAverage >= TuningConstants.ARM_STALLED_POWER_THRESHOLD &&
                Math.abs(this.lowerLeftLAVelocityAverage) <= TuningConstants.ARM_STALLED_VELOCITY_THRESHOLD)
            {
                this.lowerLAsStalled = false;
            }

            if (currTime > this.lowerSetpointChangedTime + TuningConstants.ARM_VELOCITY_TRACKING_DURATION &&
                this.lowerRightLAPowerAverage >= TuningConstants.ARM_STALLED_POWER_THRESHOLD &&
                Math.abs(this.lowerRightLAVelocityAverage) <= TuningConstants.ARM_STALLED_VELOCITY_THRESHOLD)
            {
                this.lowerLAsStalled = false;
            }

            if (currTime > this.upperSetpointChangedTime + TuningConstants.ARM_VELOCITY_TRACKING_DURATION &&
                this.upperLAsPowerAverage >= TuningConstants.ARM_STALLED_POWER_THRESHOLD &&
                Math.abs(this.upperLAVelocityAverage) <= TuningConstants.ARM_STALLED_VELOCITY_THRESHOLD)
            {
                this.upperLAsStalled = true;
            }
        }

        if (!this.inSimpleMode)
        {
            if (this.lowerLAsStalled)
            {
                this.lowerLeftArmLinearActuator.stop();
                this.lowerRightArmLinearActuator.stop();
            }
            else
            {
                this.lowerLeftArmLinearActuator.set(this.desiredLowerLeftLAPosition - HardwareConstants.ARM_EXTENTION_LENGTH * HardwareConstants.ARM_STRING_ENCODER_TICKS_PER_INCH);
                this.lowerRightArmLinearActuator.set(this.desiredLowerRightLAPosition - HardwareConstants.ARM_EXTENTION_LENGTH * HardwareConstants.ARM_STRING_ENCODER_TICKS_PER_INCH);
            }

            if (this.upperLAsStalled)
            {
                this.upperArmLinearActuator.stop();
            }
            else
            {
                this.upperArmLinearActuator.set(this.desiredUpperLAPosition);
            }
        }

        this.logger.logBoolean(LoggingKey.ArmLowerStalled, this.lowerLAsStalled);
        this.logger.logBoolean(LoggingKey.ArmUpperStalled, this.upperLAsStalled);

        this.logger.logNumber(LoggingKey.ArmLowerLeftDesiredPosition, this.desiredLowerLeftLAPosition);
        this.logger.logNumber(LoggingKey.ArmLowerRightDesiredPosition, this.desiredLowerRightLAPosition);
        this.logger.logNumber(LoggingKey.ArmUpperDesiredPosition, this.desiredUpperLAPosition);

        this.prevTime = currTime;
    }

    @Override
    public void stop()
    {
        this.lowerLeftArmLinearActuator.stop();
        this.lowerRightArmLinearActuator.stop();
        this.upperArmLinearActuator.stop();
        this.intakeMotor.stop();
        this.intakeExtender.set(DoubleSolenoidValue.Off);
        // this.leftConeFlipper.set(DoubleSolenoidValue.Off);
        // this.rightConeFlipper.set(DoubleSolenoidValue.Off);

        this.leftFlipperTransitionTime = 0.0;
        this.rightFlipperTransitionTime = 0.0;

        this.lowerSetpointChangedTime = 0.0;
        this.upperSetpointChangedTime = 0.0;
        this.lowerLAsStalled = false;
        this.upperLAsStalled = false;

        // power averaging
        this.lowerLeftLAPowerAverageCalculator.reset();
        this.lowerRightLAPowerAverageCalculator.reset();
        this.upperLAsPowerAverageCalculator.reset();
        this.lowerLeftLAPowerAverage = 0.0;
        this.lowerRightLAPowerAverage = 0.0;
        this.upperLAsPowerAverage = 0.0;

        // velocity averaging
        this.lowerLeftLAPowerAverageCalculator.reset();
        this.lowerRightLAPowerAverageCalculator.reset();
        this.upperLAsPowerAverageCalculator.reset();
        this.lowerLeftLAVelocityAverage = 0.0;
        this.lowerRightLAVelocityAverage = 0.0;
        this.upperLAVelocityAverage = 0.0;
    }

    public boolean isThroughBeamBroken()
    {
        return this.throughBeamBroken;
    }

    public double getMMLowerPosition()
    {
        return (this.lowerLeftLAPosition + this.lowerRightLAPosition) / 2.0;
    }

    public double getMMUpperPosition()
    {
        return this.upperLAPosition;
    }

    public double getFKXPosition()
    {
        return this.xPosition;
    }

    public double getFKZPosition()
    {
        return this.zPosition;
    }

    public double getLowerLeftLAPowerAverage()
    {
        return this.lowerLeftLAPowerAverage;
    }

    public double getLowerRightLAPowerAverage()
    {
        return this.lowerRightLAPowerAverage;
    }

    public boolean getLowerLAsStalled()
    {
        return this.lowerLAsStalled;
    }

    public boolean getUpperLAsStalled()
    {
        return this.upperLAsStalled;
    }

    public double getUpperLAsPowerAverage()
    {
        return this.upperLAsPowerAverage;
    }

    public boolean getInSimpleMode()
    {
        return this.inSimpleMode;
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
        // block moving it outside of the 
        if (x < TuningConstants.ARM_MIN_IKX_EXTENSION_LENGTH ||
            x > TuningConstants.ARM_MAX_IKX_EXTENSION_LENGTH ||
            z < TuningConstants.ARM_MIN_IKZ_EXTENSION_HEIGHT ||
            z > TuningConstants.ARM_MAX_IKZ_EXTENSION_HEIGHT)
        {
            return null;
        }

        // block the possibility of the end-effector clipping through the robot frame
        if (x < TuningConstants.ARM_X_IK_IN_TRESHOLD &&
            z < TuningConstants.ARM_Z_IK_IN_TRESHOLD)
        {
            return null;
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
        final double D11 = HardwareConstants.ARM_LOWER_ARM_LENGTH;
        final double D12 = HardwareConstants.ARM_UPPER_ARM_LENGTH;

        double R0 = Math.sqrt(x * x + z * z);
        Double theta2 = Helpers.calculateLawOfCosinesAngleOrNull(D11, D12, R0);
        if (theta2 == null)
        {
            // must be impossible to reach this position with our current dimensions...
            return null;
        }

        Double alpha = Helpers.calculateLawOfCosinesAngleOrNull(D11, R0, D12);
        if (alpha == null)
        {
            // must be impossible to reach this position with our current dimensions...
            return null;
        }

        double beta = Helpers.atan2d(z, x);
        double theta1 = alpha + beta;

        if (theta1 < HardwareConstants.ARM_LOWER_JOINT_CONSTRAINT_MIN ||
            theta1 > HardwareConstants.ARM_LOWER_JOINT_CONSTRAINT_MAX ||
            theta2 < HardwareConstants.ARM_UPPER_JOINT_CONSTRAINT_MIN ||
            theta2 > HardwareConstants.ARM_UPPER_JOINT_CONSTRAINT_MAX)
        {
            return null;
        }

        // in order lower, upper
        return new DoubleTuple(theta1, theta2);
    }

    /**
     * Calculate the offsets of the end effector based on the angles for the lower and upper arms
     * using forward kinematics
     * @param theta1 lower angle (in degrees)
     * @param theta2 upper angle (in degrees)
     * @return pair of horizontal, vertical offsets (x, z) (in inches)
     */
    static DoubleTuple calculateFKPositionsFromAngles(double theta1, double theta2)
    {
        final double D11 = HardwareConstants.ARM_LOWER_ARM_LENGTH;
        final double D12 = HardwareConstants.ARM_UPPER_ARM_LENGTH;

        double xPosition = D11 * Helpers.cosd(theta1) + D12 * Helpers.cosd(theta1 - 180.0 + theta2);
        double zPosition = D11 * Helpers.sind(theta1) + D12 * Helpers.sind(theta1 - 180.0 + theta2);
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

        double A1A2 = theta2 + phi;
        double R1 = Helpers.calculateLawOfCosinesDistance(L1, L4, A1A2);

        Double A5 = Helpers.calculateLawOfCosinesAngleOrNull(L3, R1, L2);
        if (A5 == null)
        {
            // must be impossible to reach this position with our current dimensions...
            return null;
        }

        Double A4 = Helpers.calculateLawOfCosinesAngleOrNull(L4, R1, L1);
        if (A4 == null)
        {
            // must be impossible to reach this position with our current dimensions...
            return null;
        }

        // VERIFY FOUR-BAR CONSTRAINTS:
        // (ensure that R2, A7A8, and A3A6 can be found, and that the lengths and angles make sense together)
        double R2 = Helpers.calculateLawOfCosinesDistance(L3, L4, A4 + A5);
        Double A7A8 = Helpers.calculateLawOfCosinesAngleOrNull(L2, L3, R1);
        Double A3A6 = Helpers.calculateLawOfCosinesAngleOrNull(L1, L2, R2);
        if (A7A8 == null || 
            A3A6 == null ||
            L1 + L2 < R2 ||
            L3 + L4 < R2 ||
            L1 + L4 < R1 ||
            L2 + L3 < R1 ||
            !Helpers.WithinDelta(A1A2 + A3A6 + A7A8 + A4 + A5, 360.0, 0.01))
        {
            return null;
        }

        double B1 = 180.0 - A4 - A5 - phi - psi + sigma;
        double L6 = Helpers.calculateLawOfCosinesDistance(L7, L8, B1);

        double lowerLAExtension = L9 - HardwareConstants.ARM_LINEAR_ACTUATOR_RETRACTED_LENGTH;
        if (lowerLAExtension < 0.0 || lowerLAExtension > HardwareConstants.ARM_EXTENTION_LENGTH)
        {
            // be forgiving of values that are just very slightly off...
            if (lowerLAExtension >= -0.00001)
            {
                lowerLAExtension = 0.0;
            }
            else if (lowerLAExtension > HardwareConstants.ARM_EXTENTION_LENGTH &&
                lowerLAExtension <= HardwareConstants.ARM_EXTENTION_LENGTH + 0.00001)
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
        if (upperLAExtension < 0.0 || upperLAExtension > HardwareConstants.ARM_EXTENTION_LENGTH)
        {
            // be forgiving of values that are just very slightly off...
            if (upperLAExtension >= -0.00001)
            {
                upperLAExtension = 0.0;
            }
            else if (upperLAExtension > HardwareConstants.ARM_EXTENTION_LENGTH &&
                upperLAExtension <= HardwareConstants.ARM_EXTENTION_LENGTH + 0.00001)
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

        double B1 = Helpers.calculateLawOfCosinesAngle(L7, L8, L6);
        double A4A5 = 180 - phi - psi + sigma - B1;
        double R2 = Helpers.calculateLawOfCosinesDistance(L4, L3, A4A5);
        double A1 = Helpers.calculateLawOfCosinesAngle(L4, R2, L3);
        double A2 = Helpers.calculateLawOfCosinesAngle(L1, R2, L2);
        double theta2 = A2 + A1 - phi;

        // in order lower, upper
        return new DoubleTuple(theta1, theta2);
    }
}