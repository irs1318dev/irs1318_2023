package frc.robot.mechanisms;

import static org.mockito.Matchers.eq;
import static org.mockito.Mockito.doReturn;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.verifyNoMoreInteractions;

import org.junit.jupiter.api.Test;
import frc.robot.*;
import frc.robot.common.LoggingManager;
import frc.robot.common.robotprovider.*;
import frc.robot.driver.AnalogOperation;
import frc.robot.driver.DigitalOperation;
import frc.robot.driver.common.Driver;

public class DriveTrainMechanismTest
{
    @Test
    public void testSetPower_Zero()
    {
        LoggingManager logger = mock(LoggingManager.class);
        ITimer timer = mock(ITimer.class);
        TestProvider testProvider = new TestProvider();
        ITalonFX leftMotor = testProvider.getTalonFX(ElectronicsConstants.DRIVETRAIN_LEFT_PRIMARY_CAN_ID);
        ITalonFX rightMotor = testProvider.getTalonFX(ElectronicsConstants.DRIVETRAIN_RIGHT_PRIMARY_CAN_ID);
        ITalonFX leftFollowerMotor = testProvider.getTalonFX(ElectronicsConstants.DRIVETRAIN_LEFT_FOLLOWER_CAN_ID);
        ITalonFX rightFollowerMotor = testProvider.getTalonFX(ElectronicsConstants.DRIVETRAIN_RIGHT_FOLLOWER_CAN_ID);

        Driver driver = mock(Driver.class);

        doReturn(false).when(driver).getDigital(DigitalOperation.DriveTrainDisablePID);
        doReturn(false).when(driver).getDigital(DigitalOperation.DriveTrainEnablePID);
        doReturn(0.0).when(driver).getAnalog(AnalogOperation.DriveTrainLeftPosition);
        doReturn(0.0).when(driver).getAnalog(AnalogOperation.DriveTrainRightPosition);
        doReturn(false).when(driver).getDigital(DigitalOperation.DriveTrainUsePositionalMode);
        doReturn(false).when(driver).getDigital(DigitalOperation.DriveTrainSwapFrontOrientation);
        doReturn(false).when(driver).getDigital(DigitalOperation.DriveTrainSimpleMode);
        doReturn(0.0).when(driver).getAnalog(AnalogOperation.DriveTrainMoveForward);
        doReturn(0.0).when(driver).getAnalog(AnalogOperation.DriveTrainTurn);
        doReturn(0.0).when(leftMotor).getError();
        doReturn(0.0).when(leftMotor).getVelocity();
        doReturn(0).when(leftMotor).getPosition();
        doReturn(0.0).when(rightMotor).getError();
        doReturn(0.0).when(rightMotor).getVelocity();
        doReturn(0).when(rightMotor).getPosition();

        DriveTrainMechanism driveTrainMechanism = new DriveTrainMechanism(logger, testProvider, timer);
        driveTrainMechanism.setDriver(driver);
        driveTrainMechanism.readSensors();
        driveTrainMechanism.update();

        // from constructor:
        verify(leftMotor).setNeutralMode(eq(MotorNeutralMode.Brake));
        verify(leftMotor).setInvertOutput(eq(HardwareConstants.DRIVETRAIN_LEFT_PRIMARY_INVERT_OUTPUT));
        verify(leftMotor).setInvertSensor(eq(HardwareConstants.DRIVETRAIN_LEFT_INVERT_SENSOR));
        verify(leftMotor).setSensorType(TalonXFeedbackDevice.IntegratedSensor);
        verify(leftMotor).setVoltageCompensation(TuningConstants.DRIVETRAIN_VOLTAGE_COMPENSATION_ENABLED, TuningConstants.DRIVETRAIN_VOLTAGE_COMPENSATION);
        verify(leftMotor).setSupplyCurrentLimit(TuningConstants.DRIVETRAIN_SUPPLY_CURRENT_LIMITING_ENABLED, TuningConstants.DRIVETRAIN_SUPPLY_CURRENT_MAX, TuningConstants.DRIVETRAIN_SUPPLY_TRIGGER_CURRENT, TuningConstants.DRIVETRAIN_SUPPLY_TRIGGER_DURATION);
        verify(leftMotor).setFeedbackFramePeriod(5);
        verify(leftMotor).setPIDFFramePeriod(5);
        verify(leftMotor).configureVelocityMeasurements(eq(10), eq(32));
        verify(leftMotor).setSelectedSlot(eq(0));
        verify(leftFollowerMotor).setNeutralMode(eq(MotorNeutralMode.Brake));
        verify(leftFollowerMotor).setInvertOutput(eq(HardwareConstants.DRIVETRAIN_LEFT_FOLLOWER1_INVERT_OUTPUT));
        verify(leftFollowerMotor).follow(eq(leftMotor));
        verify(leftFollowerMotor).setVoltageCompensation(TuningConstants.DRIVETRAIN_VOLTAGE_COMPENSATION_ENABLED, TuningConstants.DRIVETRAIN_VOLTAGE_COMPENSATION);
        verify(leftFollowerMotor).setSupplyCurrentLimit(TuningConstants.DRIVETRAIN_SUPPLY_CURRENT_LIMITING_ENABLED, TuningConstants.DRIVETRAIN_SUPPLY_CURRENT_MAX, TuningConstants.DRIVETRAIN_SUPPLY_TRIGGER_CURRENT, TuningConstants.DRIVETRAIN_SUPPLY_TRIGGER_DURATION);
        verify(rightMotor).setNeutralMode(eq(MotorNeutralMode.Brake));
        verify(rightMotor).setInvertOutput(eq(HardwareConstants.DRIVETRAIN_RIGHT_PRIMARY_INVERT_OUTPUT));
        verify(rightMotor).setInvertSensor(eq(HardwareConstants.DRIVETRAIN_RIGHT_INVERT_SENSOR));
        verify(rightMotor).setSensorType(TalonXFeedbackDevice.IntegratedSensor);
        verify(rightMotor).setVoltageCompensation(TuningConstants.DRIVETRAIN_VOLTAGE_COMPENSATION_ENABLED, TuningConstants.DRIVETRAIN_VOLTAGE_COMPENSATION);
        verify(rightMotor).setSupplyCurrentLimit(TuningConstants.DRIVETRAIN_SUPPLY_CURRENT_LIMITING_ENABLED, TuningConstants.DRIVETRAIN_SUPPLY_CURRENT_MAX, TuningConstants.DRIVETRAIN_SUPPLY_TRIGGER_CURRENT, TuningConstants.DRIVETRAIN_SUPPLY_TRIGGER_DURATION);
        verify(rightMotor).setFeedbackFramePeriod(5);
        verify(rightMotor).setPIDFFramePeriod(5);
        verify(rightMotor).configureVelocityMeasurements(eq(10), eq(32));
        verify(rightMotor).setSelectedSlot(eq(0));
        verify(rightFollowerMotor).setNeutralMode(eq(MotorNeutralMode.Brake));
        verify(rightFollowerMotor).setInvertOutput(eq(HardwareConstants.DRIVETRAIN_RIGHT_FOLLOWER1_INVERT_OUTPUT));
        verify(rightFollowerMotor).follow(eq(rightMotor));
        verify(rightFollowerMotor).setVoltageCompensation(TuningConstants.DRIVETRAIN_VOLTAGE_COMPENSATION_ENABLED, TuningConstants.DRIVETRAIN_VOLTAGE_COMPENSATION);
        verify(rightFollowerMotor).setSupplyCurrentLimit(TuningConstants.DRIVETRAIN_SUPPLY_CURRENT_LIMITING_ENABLED, TuningConstants.DRIVETRAIN_SUPPLY_CURRENT_MAX, TuningConstants.DRIVETRAIN_SUPPLY_TRIGGER_CURRENT, TuningConstants.DRIVETRAIN_SUPPLY_TRIGGER_DURATION);

        // from setDriver:
        verify(leftMotor).setPIDF(
            eq(TuningConstants.DRIVETRAIN_VELOCITY_PID_LEFT_KP),
            eq(TuningConstants.DRIVETRAIN_VELOCITY_PID_LEFT_KI),
            eq(TuningConstants.DRIVETRAIN_VELOCITY_PID_LEFT_KD),
            eq(TuningConstants.DRIVETRAIN_VELOCITY_PID_LEFT_KF),
            eq(0));
        verify(rightMotor).setPIDF(
            eq(TuningConstants.DRIVETRAIN_VELOCITY_PID_RIGHT_KP),
            eq(TuningConstants.DRIVETRAIN_VELOCITY_PID_RIGHT_KI),
            eq(TuningConstants.DRIVETRAIN_VELOCITY_PID_RIGHT_KD),
            eq(TuningConstants.DRIVETRAIN_VELOCITY_PID_RIGHT_KF),
            eq(0));
        verify(leftMotor).setControlMode(eq(TalonSRXControlMode.Velocity));
        verify(rightMotor).setControlMode(eq(TalonSRXControlMode.Velocity));

        // from readSensors:
        verify(leftMotor).getError();
        verify(leftMotor).getVelocity();
        verify(leftMotor).getPosition();
        verify(rightMotor).getError();
        verify(rightMotor).getVelocity();
        verify(rightMotor).getPosition();

        // from update:
        verify(leftMotor).set(eq(0.0));
        verify(rightMotor).set(eq(0.0));

        verifyNoMoreInteractions(leftMotor);
        verifyNoMoreInteractions(rightMotor);
        verifyNoMoreInteractions(leftFollowerMotor);
        verifyNoMoreInteractions(rightFollowerMotor);
    }

    @Test
    public void testStop()
    {
        LoggingManager logger = mock(LoggingManager.class);
        ITimer timer = mock(ITimer.class);
        TestProvider testProvider = new TestProvider();
        ITalonFX leftMotor = testProvider.getTalonFX(ElectronicsConstants.DRIVETRAIN_LEFT_PRIMARY_CAN_ID);
        ITalonFX rightMotor = testProvider.getTalonFX(ElectronicsConstants.DRIVETRAIN_RIGHT_PRIMARY_CAN_ID);
        ITalonFX leftFollowerMotor = testProvider.getTalonFX(ElectronicsConstants.DRIVETRAIN_LEFT_FOLLOWER_CAN_ID);
        ITalonFX rightFollowerMotor = testProvider.getTalonFX(ElectronicsConstants.DRIVETRAIN_RIGHT_FOLLOWER_CAN_ID);

        Driver driver = mock(Driver.class);

        doReturn(false).when(driver).getDigital(DigitalOperation.DriveTrainDisablePID);
        doReturn(false).when(driver).getDigital(DigitalOperation.DriveTrainEnablePID);
        doReturn(0.0).when(driver).getAnalog(AnalogOperation.DriveTrainLeftPosition);
        doReturn(0.0).when(driver).getAnalog(AnalogOperation.DriveTrainRightPosition);
        doReturn(false).when(driver).getDigital(DigitalOperation.DriveTrainUsePositionalMode);
        doReturn(false).when(driver).getDigital(DigitalOperation.DriveTrainSwapFrontOrientation);
        doReturn(false).when(driver).getDigital(DigitalOperation.DriveTrainSimpleMode);
        doReturn(0.0).when(driver).getAnalog(AnalogOperation.DriveTrainMoveForward);
        doReturn(0.0).when(driver).getAnalog(AnalogOperation.DriveTrainTurn);
        doReturn(0.0).when(leftMotor).getError();
        doReturn(0.0).when(leftMotor).getVelocity();
        doReturn(0).when(leftMotor).getPosition();
        doReturn(0.0).when(rightMotor).getError();
        doReturn(0.0).when(rightMotor).getVelocity();
        doReturn(0).when(rightMotor).getPosition();

        DriveTrainMechanism driveTrainMechanism = new DriveTrainMechanism(logger, testProvider, timer);
        driveTrainMechanism.setDriver(driver);
        driveTrainMechanism.stop();

        // from constructor:
        verify(leftMotor).setNeutralMode(eq(MotorNeutralMode.Brake));
        verify(leftMotor).setInvertOutput(eq(HardwareConstants.DRIVETRAIN_LEFT_PRIMARY_INVERT_OUTPUT));
        verify(leftMotor).setInvertSensor(eq(HardwareConstants.DRIVETRAIN_LEFT_INVERT_SENSOR));
        verify(leftMotor).setSensorType(TalonXFeedbackDevice.IntegratedSensor);
        verify(leftMotor).setVoltageCompensation(TuningConstants.DRIVETRAIN_VOLTAGE_COMPENSATION_ENABLED, TuningConstants.DRIVETRAIN_VOLTAGE_COMPENSATION);
        verify(leftMotor).setSupplyCurrentLimit(TuningConstants.DRIVETRAIN_SUPPLY_CURRENT_LIMITING_ENABLED, TuningConstants.DRIVETRAIN_SUPPLY_CURRENT_MAX, TuningConstants.DRIVETRAIN_SUPPLY_TRIGGER_CURRENT, TuningConstants.DRIVETRAIN_SUPPLY_TRIGGER_DURATION);
        verify(leftMotor).setFeedbackFramePeriod(5);
        verify(leftMotor).setPIDFFramePeriod(5);
        verify(leftMotor).configureVelocityMeasurements(eq(10), eq(32));
        verify(leftMotor).setSelectedSlot(eq(0));
        verify(leftFollowerMotor).setNeutralMode(eq(MotorNeutralMode.Brake));
        verify(leftFollowerMotor).setInvertOutput(eq(HardwareConstants.DRIVETRAIN_LEFT_FOLLOWER1_INVERT_OUTPUT));
        verify(leftFollowerMotor).follow(eq(leftMotor));
        verify(leftFollowerMotor).setVoltageCompensation(TuningConstants.DRIVETRAIN_VOLTAGE_COMPENSATION_ENABLED, TuningConstants.DRIVETRAIN_VOLTAGE_COMPENSATION);
        verify(leftFollowerMotor).setSupplyCurrentLimit(TuningConstants.DRIVETRAIN_SUPPLY_CURRENT_LIMITING_ENABLED, TuningConstants.DRIVETRAIN_SUPPLY_CURRENT_MAX, TuningConstants.DRIVETRAIN_SUPPLY_TRIGGER_CURRENT, TuningConstants.DRIVETRAIN_SUPPLY_TRIGGER_DURATION);
        verify(rightMotor).setNeutralMode(eq(MotorNeutralMode.Brake));
        verify(rightMotor).setInvertOutput(eq(HardwareConstants.DRIVETRAIN_RIGHT_PRIMARY_INVERT_OUTPUT));
        verify(rightMotor).setInvertSensor(eq(HardwareConstants.DRIVETRAIN_RIGHT_INVERT_SENSOR));
        verify(rightMotor).setSensorType(TalonXFeedbackDevice.IntegratedSensor);
        verify(rightMotor).setVoltageCompensation(TuningConstants.DRIVETRAIN_VOLTAGE_COMPENSATION_ENABLED, TuningConstants.DRIVETRAIN_VOLTAGE_COMPENSATION);
        verify(rightMotor).setSupplyCurrentLimit(TuningConstants.DRIVETRAIN_SUPPLY_CURRENT_LIMITING_ENABLED, TuningConstants.DRIVETRAIN_SUPPLY_CURRENT_MAX, TuningConstants.DRIVETRAIN_SUPPLY_TRIGGER_CURRENT, TuningConstants.DRIVETRAIN_SUPPLY_TRIGGER_DURATION);
        verify(rightMotor).setFeedbackFramePeriod(5);
        verify(rightMotor).setPIDFFramePeriod(5);
        verify(rightMotor).configureVelocityMeasurements(eq(10), eq(32));
        verify(rightMotor).setSelectedSlot(eq(0));
        verify(rightFollowerMotor).setNeutralMode(eq(MotorNeutralMode.Brake));
        verify(rightFollowerMotor).setInvertOutput(eq(HardwareConstants.DRIVETRAIN_RIGHT_FOLLOWER1_INVERT_OUTPUT));
        verify(rightFollowerMotor).follow(eq(rightMotor));
        verify(rightFollowerMotor).setVoltageCompensation(TuningConstants.DRIVETRAIN_VOLTAGE_COMPENSATION_ENABLED, TuningConstants.DRIVETRAIN_VOLTAGE_COMPENSATION);
        verify(rightFollowerMotor).setSupplyCurrentLimit(TuningConstants.DRIVETRAIN_SUPPLY_CURRENT_LIMITING_ENABLED, TuningConstants.DRIVETRAIN_SUPPLY_CURRENT_MAX, TuningConstants.DRIVETRAIN_SUPPLY_TRIGGER_CURRENT, TuningConstants.DRIVETRAIN_SUPPLY_TRIGGER_DURATION);

        // from setDriver:
        verify(leftMotor).setPIDF(
            eq(TuningConstants.DRIVETRAIN_VELOCITY_PID_LEFT_KP),
            eq(TuningConstants.DRIVETRAIN_VELOCITY_PID_LEFT_KI),
            eq(TuningConstants.DRIVETRAIN_VELOCITY_PID_LEFT_KD),
            eq(TuningConstants.DRIVETRAIN_VELOCITY_PID_LEFT_KF),
            eq(0));
        verify(rightMotor).setPIDF(
            eq(TuningConstants.DRIVETRAIN_VELOCITY_PID_RIGHT_KP),
            eq(TuningConstants.DRIVETRAIN_VELOCITY_PID_RIGHT_KI),
            eq(TuningConstants.DRIVETRAIN_VELOCITY_PID_RIGHT_KD),
            eq(TuningConstants.DRIVETRAIN_VELOCITY_PID_RIGHT_KF),
            eq(0));
        verify(leftMotor).setControlMode(eq(TalonSRXControlMode.Velocity));
        verify(rightMotor).setControlMode(eq(TalonSRXControlMode.Velocity));

        // from stop:
        verify(leftMotor).stop();
        verify(rightMotor).stop();
        verify(leftMotor).reset();
        verify(rightMotor).reset();

        verifyNoMoreInteractions(leftMotor);
        verifyNoMoreInteractions(rightMotor);
        verifyNoMoreInteractions(leftFollowerMotor);
        verifyNoMoreInteractions(rightFollowerMotor);
    }
}
