package frc.robot.mechanisms;

import frc.robot.*;
import frc.robot.common.*;
import frc.robot.common.robotprovider.*;
import frc.robot.driver.*;
import frc.robot.driver.common.IDriver;
import frc.robot.mechanisms.PowerManager.CurrentLimiting;

import com.google.inject.Inject;
import com.google.inject.Singleton;



@Singleton
public class ArmMechanism implements IMechanism
{
    private static final int defaultPidSlotId = 0;

    //----------------- Main Arm Variables -----------------

    // Positions are in ticks
    private double lowerArmPosition;
    // private double upperArmPosition;
    private double lowerArmVelocity;
    // private double upperArmVelocity;

    private final ITalonSRX lowerArm;
    // private final ITalonSRX upperArm;

    private final IDriver driver;
    private final ILogger logger;
    private final ITimer timer;
    private final PowerManager powerManager;

    //----------------- Side Stick Variables -----------------
    private final IDoubleSolenoid flipper;

    private enum SideArmState
    {
        Retracted,
        Extended
    };

    private SideArmState currentSideArmState;

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

        // assume initial position:
        this.lowerArmPosition = TuningConstants.LOWER_ARM_FULL_EXTENTION_LENGTH * TuningConstants.ARM_STRING_ENCODER_TICKS_PER_INCH; // Fully Extended
        // this.upperArmPosition = TuningConstants.UPPER_ARM_FULL_RETRACTED_LENGTH * TuningConstants.ARM_STRING_ENCODER_TICKS_PER_INCH; // Fully Retracted

        this.lowerArmVelocity = 0;
        // this.upperArmVelocity = 0;

        this.lowerArm = provider.getTalonSRX(ElectronicsConstants.ARM_LOWER_CAN_ID);
        // this.upperArm = provider.getTalonSRX(ElectronicsConstants.ARM_UPPER_CAN_ID);

        // this.lowerArm.setMotionMagicPIDF(
        //     TuningConstants.LOWER_ARM_POSITION_MM_PID_KP,
        //     TuningConstants.LOWER_ARM_POSITION_MM_PID_KI,
        //     TuningConstants.LOWER_ARM_POSITION_MM_PID_KD,
        //     TuningConstants.LOWER_ARM_POSITION_MM_PID_KF,
        //     TuningConstants.LOWER_ARM_POSITION_MM_CRUISE_VELOCITY,
        //     TuningConstants.LOWER_ARM_POSITION_MM_ACCELERATION,
        //     ArmMechanism.defaultPidSlotId);

        // this.upperArm.setMotionMagicPIDF(
        //     TuningConstants.UPPER_ARM_POSITION_MM_PID_KP,
        //     TuningConstants.UPPER_ARM_POSITION_MM_PID_KI,
        //     TuningConstants.UPPER_ARM_POSITION_MM_PID_KD,
        //     TuningConstants.UPPER_ARM_POSITION_MM_PID_KF,
        //     TuningConstants.UPPER_ARM_POSITION_MM_CRUISE_VELOCITY,
        //     TuningConstants.UPPER_ARM_POSITION_MM_ACCELERATION,
        //     ArmMechanism.defaultPidSlotId);

        // TODO: start in Motion-Magic mode, have separate control options for simple mode
        this.lowerArm.setControlMode(TalonXControlMode.PercentOutput);
        // this.upperArm.setControlMode(TalonXControlMode.PercentOutput);

        this.lowerArm.setSensorType(TalonXFeedbackDevice.QuadEncoder);
        // this.upperArm.setSensorType(TalonXFeedbackDevice.QuadEncoder);

        this.lowerArm.setInvertOutput(TuningConstants.LOWER_ARM_INVERT_OUTPUT);
        // this.upperArm.setInvertOutput(TuningConstants.UPPER_ARM_INVERT_OUTPUT);

        this.lowerArm.setInvertSensor(TuningConstants.LOWER_ARM_INVERT_SENSOR);
        // this.upperArm.setInvertSensor(TuningConstants.UPPER_ARM_INVERT_SENSOR);

        this.lowerArm.setPosition(this.lowerArmPosition);
        // this.upperArm.setPosition(upperArmPosition);

        this.lowerArm.setNeutralMode(MotorNeutralMode.Brake);
        //this.upperArm.setNeutralMode(MotorNeutralMode.Brake);

        ITalonSRX lowerArmFollower = provider.getTalonSRX(ElectronicsConstants.ARM_LOWER_FOLLOWER_CAN_ID);
        lowerArmFollower.setInvertOutput(TuningConstants.LOWER_ARM_INVERT_OUTPUT);
        lowerArmFollower.setNeutralMode(MotorNeutralMode.Brake);
        lowerArmFollower.follow(this.lowerArm);

        //------------------------- Side Stick Initialization ------------------------------
    
        this.flipper = provider.getDoubleSolenoid(
            ElectronicsConstants.PNEUMATICS_MODULE_A,
            ElectronicsConstants.PNEUMATICS_MODULE_TYPE_A,
            ElectronicsConstants.SIDE_STICK_PISTON_FORWARD, 
            ElectronicsConstants.SIDE_STICK_PISTON_BACKWARD);

        this.currentSideArmState = SideArmState.Retracted;
    }


    @Override
    public void readSensors()
    {
        this.lowerArmPosition = this.lowerArm.getPosition();
        // this.upperArmPosition = this.upperArm.getPosition();
        this.lowerArmVelocity = this.lowerArm.getVelocity();
        
        this.logger.logNumber(LoggingKey.LowerArmPosition, this.lowerArmPosition);
        // this.logger.logNumber(LoggingKey.UpperArmPosition, this.upperArmPosition);
        this.logger.logNumber(LoggingKey.LowerArmVelocity, this.lowerArmVelocity);
    }

    @Override
    public void update()
    {
        boolean extendFlipper = this.driver.getDigital(DigitalOperation.ExtendFlipper);
        if (extendFlipper)
        {
            // TODO: ensure that the arm is fully retracted _before_ we attempt to extend the cone-flipper
            this.flipper.set(DoubleSolenoidValue.Forward);
        }
        else
        {
            this.flipper.set(DoubleSolenoidValue.Reverse);
        }

        // TODO: ensure that our cone flipper is fully retrated.
        // if the cone-flipper isn't fully retracted then we should ensure that our arm is in the fully-retracted position
        double targetXPos = 0; // X goal
        double targetZPos = 0; // Z goal
        if (targetXPos < TuningConstants.ARM_MAX_LENGTH && targetZPos < TuningConstants.ARM_MAX_HEIGHT)
        {
            double lowerArmAngle = 90; // 90 if Starting straight up (Extended)
            double lowerArmAngleToMove = 0; // From current angle to desired angle
            double upperArmAngle = 0; //0 if retracted *angle is relative to upper arm angle*
            double upperArmAngleToMove = 0; // From current angle to desired angle

            lowerArmAngleToMove = Math.atan(
                (targetZPos / targetXPos)) + 
                Math.atan((HardwareConstants.UPPER_ARM_LENGTH * Math.sin(upperArmAngleToMove))/
                (HardwareConstants.LOWER_ARM_LENGTH + (HardwareConstants.UPPER_ARM_LENGTH * Math.cos(upperArmAngleToMove))));
            
            upperArmAngleToMove = -Math.acos(
                ((targetXPos * targetXPos) + 
                (targetZPos * targetZPos) - 
                (HardwareConstants.LOWER_ARM_LENGTH * HardwareConstants.LOWER_ARM_LENGTH) - 
                (HardwareConstants.UPPER_ARM_LENGTH * HardwareConstants.UPPER_ARM_LENGTH)) /
                (2 * HardwareConstants.UPPER_ARM_LENGTH * HardwareConstants.LOWER_ARM_LENGTH));

            double totalLowerArmAngle = lowerArmAngle + 
                HardwareConstants.LOWER_ARM_LINEAR_ACTUATOR_RIGHT_ANGLE_OFFSET + 
                HardwareConstants.LOWER_ARM_LINEAR_ACTUATOR_LEFT_ANGLE_OFFSET; // With offsets

            double linearActuatorDistanceToMove = 8; //Starting value of 8 inch, maybe (placeholder)

            linearActuatorDistanceToMove = 
                (Math.sqrt(
                    Math.pow(HardwareConstants.LOWER_ARM_TOP_PIN_OF_LINEAR_ACTUATOR_TO_PIN_ON_LOWER_ARM, 2) +
                    Math.pow(HardwareConstants.LOWER_ARM_BOTTOM_PIN_OF_LINEAR_ACTUATOR_TO_PIN_ON_LOWER_ARM, 2) - 
                    2 * HardwareConstants.LOWER_ARM_TOP_PIN_OF_LINEAR_ACTUATOR_TO_PIN_ON_LOWER_ARM * 
                    HardwareConstants.LOWER_ARM_BOTTOM_PIN_OF_LINEAR_ACTUATOR_TO_PIN_ON_LOWER_ARM * 
                    Math.cos(totalLowerArmAngle)));

            this.lowerArmPosition *= TuningConstants.ARM_STRING_ENCODER_TICKS_PER_INCH;

            double lowerArmVelocity = this.driver.getAnalog(AnalogOperation.LowerArmVelocity);
            
            this.lowerArm.set(lowerArmVelocity);

            if (this.driver.getDigital(DigitalOperation.ArmLowerExtend))
            {
                this.lowerArm.set((driver.getAnalog(AnalogOperation.LowerArmPosition) + 1) * 1016);
            }
            else
            {
                this.lowerArm.set(this.lowerArmPosition);
            }
        }
    }

    @Override
    public void stop()
    {
        this.lowerArm.stop();
        // this.upperArm.stop();
    }
}
