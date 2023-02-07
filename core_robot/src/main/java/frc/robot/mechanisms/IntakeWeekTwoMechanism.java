package frc.robot.mechanisms;

import frc.robot.*;
import frc.robot.common.*;
import frc.robot.common.robotprovider.*;
import frc.robot.driver.*;
import frc.robot.driver.common.*;

import com.google.inject.Inject;
import com.google.inject.Singleton;

@Singleton
public class IntakeWeekTwoMechanism implements IMechanism
{
    private final IDriver driver;
    private final ILogger logger;

    private final ITalonSRX intakeMotor;

    private final IDoubleSolenoid intakeExtender;

    private enum IntakeState
    {
        Retracted,
        Extended
    };

    private IntakeState currentIntakeState;

    @Inject
    public IntakeWeekTwoMechanism(IDriver driver, LoggingManager logger, ITimer timer, IRobotProvider provider)
    {
        // housekeeping
        this.driver = driver;
        this.logger = logger;
        
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
    }

    @Override
    public void update()
    {
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
    }

    @Override
    public void stop()
    {
        this.intakeMotor.stop();
        this.intakeExtender.set(DoubleSolenoidValue.Off);
    }
}
