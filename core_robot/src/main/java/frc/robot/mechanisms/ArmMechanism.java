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

    //----------------- Side Stick Variables -----------------
    private final IDoubleSolenoid rightSideArm;
    private final IDoubleSolenoid leftSideArm;
    private double leftFlipperTransitionTime;
    private double rightFlipperTransitionTime;
    private enum SideArmState
    {
        Extended,
        Extending,
        ExtendingWait,
        Retracted,
        Retracting,
    };

    private SideArmState curLeftSideArmState;
    private SideArmState curRightSideArmState;
    private boolean mainArmOverride;
    private boolean firstInstanceOfState;

    private static final int defaultPidSlotId = 0;
    
    //----------------- Main Arm Variables -----------------

    // Positions are in ticks
    private double lowerArmPosition;
    private double upperArmPosition;
    private double lowerArmVelocity;
    private double upperArmVelocity;


    private final ITalonSRX lowerArm;
    private final ITalonSRX upperArm;
    private double desiredLowerArmPosition;
    private double desiredUpperArmPosition;
    private boolean inSimpleMode;

    private double mainArmRetractionStartTime;


    private final IDriver driver;
    private final ILogger logger;
    private final ITimer timer;
    private final PowerManager powerManager;

    //---------------- Intake Variables ----------------------
    private final ITalonSRX intakeMotor;

    private final IDoubleSolenoid intakeExtender;

    private enum IntakeState
    {
        Retracted,
        Extended
    };

    //---------------- Timer -----------
    private double prevTime;
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

        // assume initial position:
        this.lowerArmPosition = TuningConstants.LOWER_ARM_FULL_EXTENTION_LENGTH * TuningConstants.ARM_STRING_ENCODER_TICKS_PER_INCH; // Fully Extended
        this.upperArmPosition = TuningConstants.UPPER_ARM_FULL_RETRACTED_LENGTH * TuningConstants.ARM_STRING_ENCODER_TICKS_PER_INCH; // Fully Retracted
        this.desiredLowerArmPosition = 0;
        this.desiredUpperArmPosition = 0;
        this.inSimpleMode = false;
        if(inSimpleMode)
        {
            this.lowerArmVelocity = 0;
            this.upperArmVelocity = 0;
        }

        this.lowerArm = provider.getTalonSRX(ElectronicsConstants.ARM_LOWER_CAN_ID);
        this.upperArm = provider.getTalonSRX(ElectronicsConstants.ARM_UPPER_CAN_ID);

        this.lowerArm.setMotionMagicPIDF(
            TuningConstants.LOWER_ARM_POSITION_MM_PID_KP,
            TuningConstants.LOWER_ARM_POSITION_MM_PID_KI,
            TuningConstants.LOWER_ARM_POSITION_MM_PID_KD,
            TuningConstants.LOWER_ARM_POSITION_MM_PID_KF,
            TuningConstants.LOWER_ARM_POSITION_MM_CRUISE_VELOCITY,
            TuningConstants.LOWER_ARM_POSITION_MM_ACCELERATION,
            ArmMechanism.defaultPidSlotId);

        this.upperArm.setMotionMagicPIDF(
            TuningConstants.UPPER_ARM_POSITION_MM_PID_KP,
            TuningConstants.UPPER_ARM_POSITION_MM_PID_KI,
            TuningConstants.UPPER_ARM_POSITION_MM_PID_KD,
            TuningConstants.UPPER_ARM_POSITION_MM_PID_KF,
            TuningConstants.UPPER_ARM_POSITION_MM_CRUISE_VELOCITY,
            TuningConstants.UPPER_ARM_POSITION_MM_ACCELERATION,
            ArmMechanism.defaultPidSlotId);

        // TODO: start in Motion-Magic mode, have separate control options for simple mode
        this.lowerArm.setControlMode(TalonXControlMode.PercentOutput);
        this.upperArm.setControlMode(TalonXControlMode.PercentOutput);

        this.lowerArm.setSensorType(TalonXFeedbackDevice.QuadEncoder);
        this.upperArm.setSensorType(TalonXFeedbackDevice.QuadEncoder);

        this.lowerArm.setInvertOutput(TuningConstants.LOWER_ARM_INVERT_OUTPUT);
        this.upperArm.setInvertOutput(TuningConstants.UPPER_ARM_INVERT_OUTPUT);

        this.lowerArm.setInvertSensor(TuningConstants.LOWER_ARM_INVERT_SENSOR);
        this.upperArm.setInvertSensor(TuningConstants.UPPER_ARM_INVERT_SENSOR);

        this.lowerArm.setPosition(this.lowerArmPosition);
        this.upperArm.setPosition(upperArmPosition);

        this.lowerArm.setNeutralMode(MotorNeutralMode.Brake);
        this.upperArm.setNeutralMode(MotorNeutralMode.Brake);

        ITalonSRX lowerArmFollower = provider.getTalonSRX(ElectronicsConstants.ARM_LOWER_FOLLOWER_CAN_ID);
        lowerArmFollower.setInvertOutput(TuningConstants.LOWER_ARM_INVERT_OUTPUT);
        lowerArmFollower.setNeutralMode(MotorNeutralMode.Brake);
        lowerArmFollower.follow(this.lowerArm);

        //------------------------- Side Stick Initialization ------------------------------
    
        this.leftSideArm = provider.getDoubleSolenoid(
            ElectronicsConstants.PNEUMATICS_MODULE_A,
            ElectronicsConstants.PNEUMATICS_MODULE_TYPE_A,
            ElectronicsConstants.LEFT_SIDE_STICK_PISTON_FORWARD, 
            ElectronicsConstants.LEFT_SIDE_STICK_PISTON_BACKWARD);
        
        this.rightSideArm = provider.getDoubleSolenoid(
            ElectronicsConstants.PNEUMATICS_MODULE_A,
            ElectronicsConstants.PNEUMATICS_MODULE_TYPE_A,
            ElectronicsConstants.RIGHT_SIDE_STICK_PISTON_FORWARD, 
            ElectronicsConstants.RIGHT_SIDE_STICK_PISTON_BACKWARD);
       
        this.leftFlipperTransitionTime = 0.0;
        this.rightFlipperTransitionTime = 0.0;

        this.curRightSideArmState = SideArmState.Retracted;
        this.curLeftSideArmState = SideArmState.Retracted;
        this.mainArmOverride = false;
        this.firstInstanceOfState = true;

        //-------------------------- Intake Initialization ----------------------------------
        this.intakeMotor = provider.getTalonSRX(ElectronicsConstants.INTAKE_MOTOR_CAN_ID);
        this.intakeMotor.setControlMode(TalonXControlMode.PercentOutput);
        this.intakeMotor.setInvertOutput(HardwareConstants.INTAKE_MOTOR_INVERT_OUTPUT);
        this.intakeMotor.setNeutralMode(MotorNeutralMode.Brake);

        this.intakeExtender =
            provider.getDoubleSolenoid(
                ElectronicsConstants.PNEUMATICS_MODULE_A,
                ElectronicsConstants.PNEUMATICS_MODULE_TYPE_A,
                ElectronicsConstants.CARGO_INTAKE_PISTON_FORWARD,
                ElectronicsConstants.CARGO_INTAKE_PISTON_REVERSE);

        this.currentIntakeState = IntakeState.Retracted;
    }


    @Override
    public void readSensors()
    {
        this.lowerArmPosition = this.lowerArm.getPosition();
        this.upperArmPosition = this.upperArm.getPosition();
        this.lowerArmVelocity = this.lowerArm.getVelocity();
        
        this.logger.logNumber(LoggingKey.LowerArmPosition, this.lowerArmPosition);
        this.logger.logNumber(LoggingKey.UpperArmPosition, this.upperArmPosition);
        this.logger.logNumber(LoggingKey.LowerArmVelocity, this.lowerArmVelocity);
    }

    @Override
    public void update()
    {
        double currTime = this.timer.get();
         
            if (this.driver.getDigital(DigitalOperation.ArmLowerExtend))
            {
                double lowerArmVelocity = this.driver.getAnalog(AnalogOperation.LowerArmVelocity);
                this.lowerArm.set(lowerArmVelocity);  
                this.lowerArm.set((driver.getAnalog(AnalogOperation.LowerArmPosition) + 1) * 1016); 
            }
            else
            {
                this.lowerArm.set(lowerArmPosition);
            }
        


        //------------------------------- Flipper's ----------------------------------------------------------------------

        boolean extendRightFlipper = this.driver.getDigital(DigitalOperation.ExtendRightFlipper);
        boolean extendLeftFlipper = this.driver.getDigital(DigitalOperation.ExtendLeftFlipper);
        switch (this.curRightSideArmState)
        {
            case Retracted:
                if (extendRightFlipper)
                {
                    this.curRightSideArmState = SideArmState.ExtendingWait;
                }

                break;

            case ExtendingWait:
                if (!extendRightFlipper)
                {
                    this.curRightSideArmState = SideArmState.Retracted;
                }
                else if (this.curLeftSideArmState == SideArmState.Retracted &&
                    ((!this.inSimpleMode &&
                        this.upperArmPosition <= TuningConstants.UPPER_ARM_NEAR_FULL_RETRACTED_LENGTH * TuningConstants.ARM_STRING_ENCODER_TICKS_PER_INCH &&  // some value slightly larger than 0.0 inches 
                        this.lowerArmPosition >= TuningConstants.LOWER_ARM_NEAR_FULL_EXTENSION_LENGTH * TuningConstants.ARM_STRING_ENCODER_TICKS_PER_INCH) ||  // some value slightly smaller than fully extended 
                    (this.inSimpleMode && currTime - this.mainArmRetractionStartTime > TuningConstants.ARM_RETRACTION_MAX_TIME)))
                {
                    this.curRightSideArmState = SideArmState.Extending;
                    this.rightFlipperTransitionTime = currTime;
                }

                break;

            case Extending:
                if (!extendRightFlipper)
                {
                    this.curRightSideArmState = SideArmState.Retracting;
                    this.rightFlipperTransitionTime = currTime;
                }
                else if (currTime - this.rightFlipperTransitionTime > 0.5) // TODO: use TuningConstant for extending
                {
                    this.curRightSideArmState = SideArmState.Extended;
                }

                break;

            case Extended:
                if (!extendRightFlipper)
                {
                    this.curRightSideArmState = SideArmState.Retracting;
                    this.rightFlipperTransitionTime = currTime;
                }

                break;

            case Retracting:
                if (extendRightFlipper)
                {
                    this.curRightSideArmState = SideArmState.Extending;
                    this.rightFlipperTransitionTime = currTime;
                }
                else if (currTime - this.rightFlipperTransitionTime > 0.5) // TODO: use tuning constant for retracting
                {
                    this.curRightSideArmState = SideArmState.Retracted;
                }

                break;
        }

        switch (this.curLeftSideArmState)
        {
            case Retracted:
                if (extendLeftFlipper)
                {
                    this.curLeftSideArmState = SideArmState.ExtendingWait;
                    this.mainArmRetractionStartTime = currTime;
                }

                break;

            case ExtendingWait:
                if (!extendLeftFlipper)
                {
                    this.curLeftSideArmState = SideArmState.Retracted;
                }
                else if (this.curLeftSideArmState == SideArmState.Retracted &&
                    ((!this.inSimpleMode &&
                        this.upperArmPosition <= TuningConstants.UPPER_ARM_NEAR_FULL_RETRACTED_LENGTH * TuningConstants.ARM_STRING_ENCODER_TICKS_PER_INCH &&  // some value slightly larger than 0.0 inches 
                        this.lowerArmPosition >= TuningConstants.LOWER_ARM_NEAR_FULL_EXTENSION_LENGTH * TuningConstants.ARM_STRING_ENCODER_TICKS_PER_INCH) ||  // some value slightly smaller than fully extended 
                    (this.inSimpleMode && currTime - this.mainArmRetractionStartTime > TuningConstants.ARM_RETRACTION_MAX_TIME)))
                {
                    this.curLeftSideArmState = SideArmState.Extending;
                    this.leftFlipperTransitionTime = currTime;
                }

                break;

            case Extending:
                if (!extendLeftFlipper)
                {
                    this.curLeftSideArmState = SideArmState.Retracting;
                    this.leftFlipperTransitionTime = currTime;
                }
                else if (currTime - this.leftFlipperTransitionTime > 0.5) // TODO: use TuningConstant for extending
                {
                    this.curLeftSideArmState = SideArmState.Extended;
                }

                break;

            case Extended:
                if (!extendLeftFlipper)
                {
                    this.curLeftSideArmState = SideArmState.Retracting;
                    this.leftFlipperTransitionTime = currTime;
                }

                break;

            case Retracting:
                if (extendLeftFlipper)
                {
                    this.curLeftSideArmState = SideArmState.Extending;
                    this.leftFlipperTransitionTime = currTime;
                }
                else if (currTime - this.leftFlipperTransitionTime > 0.5) // TODO: use tuning constant for retracting
                {
                    this.curLeftSideArmState = SideArmState.Retracted;
                }

                break;
        }

        switch (this.curRightSideArmState)
        {
            case Extended:
            case Extending:
                this.rightSideArm.set(DoubleSolenoidValue.Forward);
                break;

            default:
            case ExtendingWait:
            case Retracted:
            case Retracting:
                this.rightSideArm.set(DoubleSolenoidValue.Reverse);
                break;
        }

        switch (this.curLeftSideArmState)
        {
            case Extended:
            case Extending:
                this.leftSideArm.set(DoubleSolenoidValue.Forward);
                break;

            default:
            case ExtendingWait:
            case Retracted:
            case Retracting:
                this.leftSideArm.set(DoubleSolenoidValue.Reverse);
                break;
        }


        //----------------------------------- Intake Update -----------------------------------------------------------
        // control intake rollers
        double intakePower = TuningConstants.ZERO;
        if (this.driver.getDigital(DigitalOperation.IntakeIn))
        {
            intakePower = TuningConstants.INTAKE_POWER;
        }
        else if (this.driver.getDigital(DigitalOperation.IntakeOut))
        {
            intakePower = -TuningConstants.INTAKE_POWER;
        }
        this.intakeMotor.set(intakePower);
        this.logger.logNumber(LoggingKey.IntakePower, intakePower);

        // intake state transitions
        if (this.driver.getDigital(DigitalOperation.IntakeExtend))
        {
            this.currentIntakeState = IntakeState.Extended;
        }
        else if (this.driver.getDigital(DigitalOperation.IntakeRetract))
        {
            this.currentIntakeState = IntakeState.Retracted;
        }
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

        //--------------------------------------------------- Main Arm -----------------------------------------------
        if (this.curLeftSideArmState != SideArmState.Retracted || this.curRightSideArmState != SideArmState.Retracted)
        {
            if(this.inSimpleMode)
            {
                this.lowerArm.set(TuningConstants.ARM_MAX_SIMPLE_VELOCITY);
            } // Add code for individual modes(percent output or mmposition)
            else 
            {
                this.desiredLowerArmPosition = TuningConstants.LOWER_ARM_FULL_EXTENTION_LENGTH * TuningConstants.ARM_STRING_ENCODER_TICKS_PER_INCH;
                this.desiredUpperArmPosition = TuningConstants.UPPER_ARM_FULL_RETRACTED_LENGTH * TuningConstants.ARM_STRING_ENCODER_TICKS_PER_INCH;

                this.lowerArm.set(this.desiredLowerArmPosition);
                this.upperArm.set(this.desiredUpperArmPosition);
            }
        }
        else
        {
            this.lowerArm.setControlMode(TalonXControlMode.PercentOutput);
            this.upperArm.setControlMode(TalonXControlMode.PercentOutput);
            
            //Main Arm Control
            if (this.driver.getDigital(DigitalOperation.ArmSimpleMode))
            {
                this.inSimpleMode = true;
                this.lowerArm.setControlMode(TalonXControlMode.PercentOutput);
                this.upperArm.setControlMode(TalonXControlMode.PercentOutput); 
            }

            if (this.inSimpleMode)
            {
                this.lowerArm.set(this.driver.getAnalog(AnalogOperation.ArmSimpleForceLower) * 10);
                this.upperArm.set(this.driver.getAnalog(AnalogOperation.ArmSimpleForceUpper));
            }
            else
            {
            if (this.driver.getAnalog(AnalogOperation.ArmIKXPosition) >= 0 && this.driver.getAnalog(AnalogOperation.ArmIKYPosition) >= 0)
            {
                Setpoint IK = this.calculateIK(this.driver.getAnalog(AnalogOperation.ArmIKXPosition), this.driver.getAnalog(AnalogOperation.ArmIKYPosition));
                this.desiredLowerArmPosition = IK.lowerPosition;
                this.desiredUpperArmPosition = IK.upperPosition;
            }
            else if(this.driver.getAnalog(AnalogOperation.ArmMMUpperPosition) < 0 && this.driver.getAnalog(AnalogOperation.ArmMMLowerPosition) < 0)
            {
                this.desiredLowerArmPosition =this.driver.getAnalog(AnalogOperation.ArmMMLowerPosition);
                this.desiredUpperArmPosition = this.driver.getAnalog(AnalogOperation.ArmMMUpperPosition); 
            }
            else if (this.driver.getAnalog(AnalogOperation.ArmLowerPositionAdjustment) != 0.0 && this.driver.getAnalog(AnalogOperation.ArmUpperPositionAdjustment) != 0.0)
            {
            
                double elapsedTime = this.prevTime - currTime;
                this.desiredLowerArmPosition = this.driver.getAnalog(AnalogOperation.ArmLowerPositionAdjustment) * TuningConstants.ARM_STRING_ENCODER_TICKS_PER_INCH * elapsedTime;
            }

                this.lowerArm.set(this.desiredLowerArmPosition);
                this.prevTime = currTime;

            }
        }
        
        // Setpoint IK = this.calculateIK(x, y)
        // IK.lowerPosition;
        // IK.upperPosition;
    }

    @Override
    public void stop()
    {
        this.lowerArm.stop();
        this.leftSideArm.set(DoubleSolenoidValue.Off);
        this.rightSideArm.set(DoubleSolenoidValue.Off);
        // this.upperArm.stop();
    }

    private Setpoint calculateIK(double targetXPos, double targetZPos)
    {
        // TODO: ensure that our cone flipper is fully retrated -- Done
        // if the cone-flipper isn't fully retracted then we should ensure that our arm is in the fully-retracted position

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

            double lowerArmPosition = 0;
            lowerArmPosition *= TuningConstants.ARM_STRING_ENCODER_TICKS_PER_INCH; 
            lowerArmPosition = (lowerArmPosition + 1) * 1016;
            double upperArmPosition = 0;        
        }

        return new Setpoint(upperArmPosition, lowerArmPosition);
    }
 
    /**
     * Basic structure to hold an angle/drive pair
     */
    private class Setpoint
    {
        public final double upperPosition;
        public final double lowerPosition;

        public Setpoint(double upperPosition, double lowerPosition)
        {
            this.upperPosition = upperPosition;
            this.lowerPosition = lowerPosition;
        }
    }
}