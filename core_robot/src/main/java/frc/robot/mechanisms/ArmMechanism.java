package frc.robot.mechanisms;

import frc.robot.*;
import frc.robot.common.*;
import frc.robot.common.robotprovider.*;
import frc.robot.driver.*;
import frc.robot.driver.common.IDriver;
import frc.robot.mechanisms.PowerManager.CurrentLimiting;

import com.google.inject.Inject;
import com.google.inject.Singleton;

public class ArmMechanism implements IMechanism{

    public static final AnalogOperation LOWER_ARM_SETPOINT_OPERATION = AnalogOperation.LowerArmPosition;
    public static final AnalogOperation UPPER_ARM_SETPOINT_OPERATION = AnalogOperation.UpperArmPosition;
    
    //Positions are in ticks
    private double lowerArmPosition;
    private double upperArmPosition;
    private double lowerArmVelocity;
    //private double upperArmVelocity;
     

    private final ITalonSRX lowerArm;
   // private final ITalonSRX upperArm;

    private final double ARM_MAX_VELOCITY;
    private final double ARM_MAX_ACCLERATION;

    
    private final IDriver driver;
    private final ILogger logger;
    private final ITimer timer;
    private final PowerManager powerManager;

    private static final int defaultPidSlotId = 0;
    

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
        
        this.ARM_MAX_VELOCITY = TuningConstants.ARM_MAX_VELOCITY;
        this.ARM_MAX_ACCLERATION = TuningConstants.ARM_MAX_ACCLERATION;

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
                this.ARM_MAX_VELOCITY, 
                this.ARM_MAX_ACCLERATION, 
                defaultPidSlotId);
                
        
        this.upperArm.setMotionMagicPIDF(
                TuningConstants.UPPER_ARM_POSITION_MM_PID_KP,
                TuningConstants.UPPER_ARM_POSITION_MM_PID_KI,
                TuningConstants.UPPER_ARM_POSITION_MM_PID_KD,
                TuningConstants.UPPER_ARM_POSITION_MM_PID_KF, 
                this.ARM_MAX_VELOCITY, 
                this.ARM_MAX_ACCLERATION, 
                defaultPidSlotId);
                */
        
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
    }


    @Override
    public void readSensors()
    {
        this.lowerArmPosition = lowerArm.getPosition();
        //this.upperArmPosition = upperArm.getPosition();
        this.lowerArmVelocity = lowerArm.getVelocity();
        
        this.logger.logNumber(LoggingKey.LowerArmPosition, this.lowerArmPosition);
        //this.logger.logNumber(LoggingKey.UpperArmPosition, this.upperArmPosition);
        this.logger.logNumber(LoggingKey.LowerArmVelocity, this.lowerArmVelocity);        
    }

    @Override
    public void update()
    {
        double lowerArmVelocity = driver.getAnalog(AnalogOperation.LowerArmVelocity);
        this.lowerArm.set(lowerArmVelocity);

        // if(driver.getDigital(DigitalOperation.IntakeLowerExtend))
        // {
        //     lowerArm.set( (driver.getAnalog(AnalogOperation.LowerArmPosition) + 1) * 1016);
        // }
        // else
        // {
        //     lowerArm.set(this.lowerArmPosition);
        // }

        // if(driver.getDigital(DigitalOperation.IntakeUpperExtend))
        // {
        //     upperArm.set( (driver.getAnalog(AnalogOperation.UpperArmPosition) + 1) * 1016);
        // }
        // else
        // {
        //     upperArm.set(this.upperArmPosition);
        // }
       
    }

    @Override
    public void stop()
    {
        lowerArm.set(0);
        //upperArm.set(0);
    }
    
}
