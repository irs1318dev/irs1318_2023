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
    //----------------- Main Arm Variables ----------------------------------------
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

    private static final int defaultPidSlotId = 0;

    
    
    //-------------------- Side Stick Variables ------------------------------------
    private enum SideArmState
    {
        Retracted,
        Extended
    };
    private SideArmState currentSideArmState;
    private IDoubleSolenoid SideStick;

    @Inject
    public ArmMechanism(
        IDriver driver,
        LoggingManager logger,
        IRobotProvider provider,
        PowerManager powerManager,
        ITimer timer)
    {
        //------------------------- Main Arm Initializiation -----------------------------------

        this.lowerArmPosition = TuningConstants.LOWER_ARM_FULL_EXTENTION_LENGTH * TuningConstants.ARM_STRING_ENCODER_TICKS_PER_INCH; // Fully Extended
        //this.upperArmPosition = TuningConstants.UPPER_ARM_FULL_RETRACTED_LENGTH * TuningConstants.ARM_STRING_ENCODER_TICKS_PER_INCH; // Fully Retracted

        this.lowerArmVelocity = 0;
        //this.upperArmVelocity = 0;

        

        this.lowerArm = provider.getTalonSRX(ElectronicsConstants.ARM_LOWER_CAN_ID);
        //this.upperArm = provider.getTalonSRX(ElectronicsConstants.ARM_UPPER_CAN_ID);
        /* 
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
        
        this.lowerArm.setControlMode(TalonXControlMode.PercentOutput);
        //this.upperArm.setControlMode(TalonXControlMode.PercentOutput);
        
        this.lowerArm.setSensorType(TalonXFeedbackDevice.QuadEncoder);
        //this.upperArm.setSensorType(TalonXFeedbackDevice.QuadEncoder);
        
        this.lowerArm.setInvertOutput(TuningConstants.LOWER_ARM_INVERT_OUTPUT);
        //this.upperArm.setInvertOutput(TuningConstants.UPPER_ARM_INVERT_OUTPUT);
        
        this.lowerArm.setInvertSensor(TuningConstants.LOWER_ARM_INVERT_SENSOR);
        //this.upperArm.setInvertSensor(TuningConstants.UPPER_ARM_INVERT_SENSOR);

        this.lowerArm.setPosition(lowerArmPosition); // 8 inches of extension 254 ticks per inch mayber 256 chekc specs
        //this.upperArm.setPosition(upperArmPosition);

        this.lowerArm.setNeutralMode(MotorNeutralMode.Brake);
        //this.upperArm.setNeutralMode(MotorNeutralMode.Brake);

        ITalonSRX lowerArmFollower = provider.getTalonSRX(ElectronicsConstants.ARM_LOWER_FOLLOWER_CAN_ID);
        lowerArmFollower.setInvertOutput(TuningConstants.LOWER_ARM_INVERT_OUTPUT);
        lowerArmFollower.setNeutralMode(MotorNeutralMode.Brake);
        lowerArmFollower.follow(this.lowerArm);
/* 
        ITalonSRX upperArmFollower = provider.getTalonSRX(ElectronicsConstants.ARM_UPPER_FOLLOWER_CAN_ID);
        upperArmFollower.setInvertOutput(TuningConstants.UPPER_ARM_INVERT_OUTPUT);
        upperArmFollower.setNeutralMode(MotorNeutralMode.Brake);
        upperArmFollower.follow(this.upperArm);*/

        //------------------------- Side Stick Initialization ------------------------------
    
        this.SideStick = provider.getDoubleSolenoid(
            ElectronicsConstants.PNEUMATICS_MODULE_C,
            ElectronicsConstants.PNEUMATICS_MODULE_TYPE_C,
            ElectronicsConstants.SIDE_STICK_PISTON_FORWARD, 
            ElectronicsConstants.SIDE_STICK_PISTON_BACKWARD);

        this.currentSideArmState = SideArmState.Retracted;
        this.driver = driver;
        this.logger = logger;
        this.timer = timer;
        this.powerManager = powerManager;
    }


    @Override
    public void readSensors()
    {
        this.lowerArmPosition = this.lowerArm.getPosition();
        //this.upperArmPosition = this.upperArm.getPosition();
        this.lowerArmVelocity = this.lowerArm.getVelocity();
        
        this.logger.logNumber(LoggingKey.LowerArmPosition, this.lowerArmPosition);
        //this.logger.logNumber(LoggingKey.UpperArmPosition, this.upperArmPosition);
        this.logger.logNumber(LoggingKey.LowerArmVelocity, this.lowerArmVelocity);
    }

    @Override
    public void update()
    {

        if (this.driver.getDigital(DigitalOperation.SwitchArm))
        {
            this.SideStick.set(DoubleSolenoidValue.Forward);
            lowerArm.set(90 * TuningConstants.ARM_STRING_ENCODER_TICKS_PER_INCH); //LowerArm Straight up
            //upperArmAngle etc... // UpperArm fully retracted
        }
        else {
            this.SideStick.set(DoubleSolenoidValue.Reverse);
            MainArm();
        }
        
    }

    @Override
    public void stop()
    {
        this.lowerArm.stop();
        // this.upperArm.stop();
    }

    public void MainArm()
        {
            //------------------------- Main Arm Update --------------------------------
         double targetXPos = 0; // Place holder value - How far away desired point is after from robot after reading april tag
         double targetZPos = 0;
         if (targetXPos < TuningConstants.MAX_LENGTH && targetZPos < TuningConstants.MAX_HEIGHT)
         {
            double lowerArmAngle = 90; // 90 if Starting straight up (Extended)
            double lowerArmAngleToMove = 0; // From current angle to desired angle
            double upperArmAngle = 0; //0 if retracted *angle is relative to upper arm angle*
            double upperArmAngleToMove = 0; // From current angle to desired angle

            lowerArmAngleToMove = Math.atan(
                (targetZPos/targetXPos)) + 
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
            HardwareConstants.LOWER_ARM_LINEAR_ACTUATOR_LEFT_ANGLE_OFFSET; //With offsets
            
            double linearActuatorDistanceToMove = 8; //Starting value of 8 inch, maybe (placeholder)

            linearActuatorDistanceToMove = (Math.sqrt(
            HardwareConstants.LOWER_ARM_TOP_PIN_OF_LINEAR_ACTUATOR_TO_PIN_ON_LOWER_ARM *
            HardwareConstants.LOWER_ARM_TOP_PIN_OF_LINEAR_ACTUATOR_TO_PIN_ON_LOWER_ARM +
            HardwareConstants.LOWER_ARM_BOTTOM_PIN_OF_LINEAR_ACTUATOR_TO_PIN_ON_LOWER_ARM *
            HardwareConstants.LOWER_ARM_BOTTOM_PIN_OF_LINEAR_ACTUATOR_TO_PIN_ON_LOWER_ARM - 
            2 * HardwareConstants.LOWER_ARM_TOP_PIN_OF_LINEAR_ACTUATOR_TO_PIN_ON_LOWER_ARM * 
            HardwareConstants.LOWER_ARM_BOTTOM_PIN_OF_LINEAR_ACTUATOR_TO_PIN_ON_LOWER_ARM * 
            Math.cos(totalLowerArmAngle)));

            this.lowerArmPosition *= TuningConstants.ARM_STRING_ENCODER_TICKS_PER_INCH;

            double lowerArmVelocity = this.driver.getAnalog(AnalogOperation.LowerArmVelocity);
            
            this.lowerArm.set(lowerArmVelocity);

            if (this.driver.getDigital(DigitalOperation.IntakeLowerExtend))
            {
                this.lowerArm.set((driver.getAnalog(AnalogOperation.LowerArmPosition) + 1) * 1016);
            }
            else
            {
                this.lowerArm.set(this.lowerArmPosition);
            }

            // if (this.driver.getDigital(DigitalOperation.IntakeUpperExtend))
            // {
            //     this.upperArm.set((this.driver.getAnalog(AnalogOperation.UpperArmPosition) + 1) * 1016);
            // }
            // else
            // {
            //     this.upperArm.set(this.upperArmPosition);
            // }
            
         
        }
       // return lowerArmPosition;
    }
}
