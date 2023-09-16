package frc.robot.mechanisms;

import com.google.inject.Singleton;

import frc.lib.controllers.PIDHandler;
import frc.lib.driver.Driver;
import frc.lib.driver.IDriver;
import frc.lib.helpers.Helpers;
import frc.lib.mechanisms.IMechanism;
import frc.lib.mechanisms.LoggingManager;
import frc.lib.robotprovider.ILogger;
import frc.lib.robotprovider.IRobotProvider;
import frc.lib.robotprovider.ITalonFX;
import frc.lib.robotprovider.ITimer;
import frc.lib.robotprovider.IVictorSPX;
import frc.lib.robotprovider.MotorNeutralMode;
import frc.lib.robotprovider.TalonXFeedbackDevice;
import frc.robot.ElectronicsConstants;
import frc.robot.HardwareConstants;
import frc.robot.LoggingKey;
import frc.robot.TuningConstants;
import frc.robot.driver.AnalogOperation;
import frc.robot.driver.DigitalOperation;

@Singleton
public class SimpleDriveTrainMech implements IMechanism {

    private final IVictorSPX leftMotor;
    private final IVictorSPX rightMotor;
    private IDriver driver;

    @inject
    public SimpleDriveTrainMech(
        IRobotProvider provider
    )
    {
        this.leftMotor = provider.getVictorSPX(0);
        this.rightMotor = provider.getVictorSPX(1);

        this.leftMotor.setNeutralMode(MotorNeutralMode.Brake);
        this.rightMotor.setNeutralMode(MotorNeutralMode.Brake);
    }

    @Override
    public void update()
    {
        double forwardBackward = this.driver.getAnalog(AnalogOperation.DriveTrainMoveForward);
        double leftRight = this.driver.getAnalog(AnalogOperation.DriveTrainMoveRight);

        this.leftMotor.set(forwardBackward);
        this.rightMotor.set(leftRight);

    }

    @Override
    public void readSensors() {
        
    }

    @Override
    public void stop() {
        this.leftMotor.set(0.0);
        this.rightMotor.set(0.0);
    }

    
}
