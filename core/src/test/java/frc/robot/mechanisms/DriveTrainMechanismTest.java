package frc.robot.mechanisms;

import static org.mockito.Matchers.eq;
import static org.mockito.Mockito.doReturn;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.verifyNoMoreInteractions;

import org.junit.jupiter.api.Test;
import frc.robot.ElectronicsConstants;
import frc.robot.HardwareConstants;
import frc.robot.TestProvider;
import frc.robot.TuningConstants;
import frc.robot.common.robotprovider.IDashboardLogger;
import frc.robot.common.robotprovider.ITalonSRX;
import frc.robot.common.robotprovider.ITimer;
import frc.robot.common.robotprovider.TalonSRXControlMode;
import frc.robot.common.robotprovider.TalonSRXFeedbackDevice;
import frc.robot.common.robotprovider.TalonSRXNeutralMode;
import frc.robot.driver.Operation;
import frc.robot.driver.common.Driver;
import frc.robot.mechanisms.DriveTrainMechanism;

public class DriveTrainMechanismTest
{
    ////@Test
    public void testSetPower_Zero()
    {
        IDashboardLogger logger = mock(IDashboardLogger.class);
        ITimer timer = mock(ITimer.class);
        TestProvider testProvider = new TestProvider();
        ITalonSRX leftMotor = testProvider.getTalonSRX(ElectronicsConstants.DRIVETRAIN_LEFT_MASTER_CAN_ID);
        ITalonSRX rightMotor = testProvider.getTalonSRX(ElectronicsConstants.DRIVETRAIN_RIGHT_MASTER_CAN_ID);
        ITalonSRX leftFollowerMotor1 = testProvider.getTalonSRX(ElectronicsConstants.DRIVETRAIN_LEFT_FOLLOWER1_CAN_ID);
        ITalonSRX rightFollowerMotor1 = testProvider.getTalonSRX(ElectronicsConstants.DRIVETRAIN_RIGHT_FOLLOWER1_CAN_ID);
        ITalonSRX leftFollowerMotor2 = testProvider.getTalonSRX(ElectronicsConstants.DRIVETRAIN_LEFT_FOLLOWER2_CAN_ID);
        ITalonSRX rightFollowerMotor2 = testProvider.getTalonSRX(ElectronicsConstants.DRIVETRAIN_RIGHT_FOLLOWER2_CAN_ID);

        Driver driver = mock(Driver.class);

        doReturn(false).when(driver).getDigital(Operation.DriveTrainDisablePID);
        doReturn(false).when(driver).getDigital(Operation.DriveTrainEnablePID);
        doReturn(0.0).when(driver).getAnalog(Operation.DriveTrainLeftPosition);
        doReturn(0.0).when(driver).getAnalog(Operation.DriveTrainRightPosition);
        doReturn(false).when(driver).getDigital(Operation.DriveTrainUsePositionalMode);
        doReturn(false).when(driver).getDigital(Operation.DriveTrainSwapFrontOrientation);
        doReturn(false).when(driver).getDigital(Operation.DriveTrainSimpleMode);
        doReturn(0.0).when(driver).getAnalog(Operation.DriveTrainMoveForward);
        doReturn(0.0).when(driver).getAnalog(Operation.DriveTrainTurn);
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
        verify(leftMotor).setNeutralMode(eq(TalonSRXNeutralMode.Brake));
        verify(leftMotor).setInvertOutput(eq(HardwareConstants.DRIVETRAIN_LEFT_MASTER_INVERT_OUTPUT));
        verify(leftMotor).setInvertSensor(eq(HardwareConstants.DRIVETRAIN_LEFT_INVERT_SENSOR));
        verify(leftMotor).setSensorType(TalonSRXFeedbackDevice.QuadEncoder);
        verify(leftMotor).setFeedbackFramePeriod(5);
        verify(leftMotor).setPIDFFramePeriod(5);
        verify(leftMotor).configureVelocityMeasurements();
        verify(leftMotor).setSelectedSlot(eq(0));
        verify(leftFollowerMotor1).setNeutralMode(eq(TalonSRXNeutralMode.Brake));
        verify(leftFollowerMotor1).setInvertOutput(eq(HardwareConstants.DRIVETRAIN_LEFT_FOLLOWER1_INVERT_OUTPUT));
        verify(leftFollowerMotor1).setControlMode(eq(TalonSRXControlMode.Follower));
        verify(leftFollowerMotor1).set(eq((double)ElectronicsConstants.DRIVETRAIN_LEFT_MASTER_CAN_ID));
        verify(leftFollowerMotor2).setNeutralMode(eq(TalonSRXNeutralMode.Brake));
        verify(leftFollowerMotor2).setInvertOutput(eq(HardwareConstants.DRIVETRAIN_LEFT_FOLLOWER2_INVERT_OUTPUT));
        verify(leftFollowerMotor2).setControlMode(eq(TalonSRXControlMode.Follower));
        verify(leftFollowerMotor2).set(eq((double)ElectronicsConstants.DRIVETRAIN_LEFT_MASTER_CAN_ID));
        verify(rightMotor).setNeutralMode(eq(TalonSRXNeutralMode.Brake));
        verify(rightMotor).setInvertOutput(eq(HardwareConstants.DRIVETRAIN_RIGHT_MASTER_INVERT_OUTPUT));
        verify(rightMotor).setInvertSensor(eq(HardwareConstants.DRIVETRAIN_RIGHT_INVERT_SENSOR));
        verify(rightMotor).setSensorType(TalonSRXFeedbackDevice.QuadEncoder);
        verify(rightMotor).setFeedbackFramePeriod(5);
        verify(rightMotor).setPIDFFramePeriod(5);
        verify(rightMotor).configureVelocityMeasurements();
        verify(rightMotor).setSelectedSlot(eq(0));
        verify(rightFollowerMotor1).setControlMode(eq(TalonSRXControlMode.Follower));
        verify(rightFollowerMotor1).setNeutralMode(eq(TalonSRXNeutralMode.Brake));
        verify(rightFollowerMotor1).setInvertOutput(eq(HardwareConstants.DRIVETRAIN_RIGHT_FOLLOWER1_INVERT_OUTPUT));
        verify(rightFollowerMotor1).set(eq((double)ElectronicsConstants.DRIVETRAIN_RIGHT_MASTER_CAN_ID));
        verify(rightFollowerMotor2).setControlMode(eq(TalonSRXControlMode.Follower));
        verify(rightFollowerMotor2).setNeutralMode(eq(TalonSRXNeutralMode.Brake));
        verify(rightFollowerMotor2).setInvertOutput(eq(HardwareConstants.DRIVETRAIN_RIGHT_FOLLOWER2_INVERT_OUTPUT));
        verify(rightFollowerMotor2).set(eq((double)ElectronicsConstants.DRIVETRAIN_RIGHT_MASTER_CAN_ID));

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
        verifyNoMoreInteractions(leftFollowerMotor1);
        verifyNoMoreInteractions(rightFollowerMotor1);
        verifyNoMoreInteractions(leftFollowerMotor2);
        verifyNoMoreInteractions(rightFollowerMotor2);
    }

    ////@Test
    public void testStop()
    {
        IDashboardLogger logger = mock(IDashboardLogger.class);
        ITimer timer = mock(ITimer.class);
        TestProvider testProvider = new TestProvider();
        ITalonSRX leftMotor = testProvider.getTalonSRX(ElectronicsConstants.DRIVETRAIN_LEFT_MASTER_CAN_ID);
        ITalonSRX rightMotor = testProvider.getTalonSRX(ElectronicsConstants.DRIVETRAIN_RIGHT_MASTER_CAN_ID);
        ITalonSRX leftFollowerMotor1 = testProvider.getTalonSRX(ElectronicsConstants.DRIVETRAIN_LEFT_FOLLOWER1_CAN_ID);
        ITalonSRX rightFollowerMotor1 = testProvider.getTalonSRX(ElectronicsConstants.DRIVETRAIN_RIGHT_FOLLOWER1_CAN_ID);
        ITalonSRX leftFollowerMotor2 = testProvider.getTalonSRX(ElectronicsConstants.DRIVETRAIN_LEFT_FOLLOWER2_CAN_ID);
        ITalonSRX rightFollowerMotor2 = testProvider.getTalonSRX(ElectronicsConstants.DRIVETRAIN_RIGHT_FOLLOWER2_CAN_ID);

        Driver driver = mock(Driver.class);

        doReturn(false).when(driver).getDigital(Operation.DriveTrainDisablePID);
        doReturn(false).when(driver).getDigital(Operation.DriveTrainEnablePID);
        doReturn(0.0).when(driver).getAnalog(Operation.DriveTrainLeftPosition);
        doReturn(0.0).when(driver).getAnalog(Operation.DriveTrainRightPosition);
        doReturn(false).when(driver).getDigital(Operation.DriveTrainUsePositionalMode);
        doReturn(false).when(driver).getDigital(Operation.DriveTrainSwapFrontOrientation);
        doReturn(false).when(driver).getDigital(Operation.DriveTrainSimpleMode);
        doReturn(0.0).when(driver).getAnalog(Operation.DriveTrainMoveForward);
        doReturn(0.0).when(driver).getAnalog(Operation.DriveTrainTurn);
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
        verify(leftMotor).setNeutralMode(eq(TalonSRXNeutralMode.Brake));
        verify(leftMotor).setInvertOutput(eq(HardwareConstants.DRIVETRAIN_LEFT_MASTER_INVERT_OUTPUT));
        verify(leftMotor).setInvertSensor(eq(HardwareConstants.DRIVETRAIN_LEFT_INVERT_SENSOR));
        verify(leftMotor).setSensorType(TalonSRXFeedbackDevice.QuadEncoder);
        verify(leftMotor).setFeedbackFramePeriod(5);
        verify(leftMotor).setPIDFFramePeriod(5);
        verify(leftMotor).configureVelocityMeasurements();
        verify(leftMotor).setSelectedSlot(eq(0));
        verify(leftFollowerMotor1).setNeutralMode(eq(TalonSRXNeutralMode.Brake));
        verify(leftFollowerMotor1).setInvertOutput(eq(HardwareConstants.DRIVETRAIN_LEFT_FOLLOWER1_INVERT_OUTPUT));
        verify(leftFollowerMotor1).setControlMode(eq(TalonSRXControlMode.Follower));
        verify(leftFollowerMotor1).set(eq((double)ElectronicsConstants.DRIVETRAIN_LEFT_MASTER_CAN_ID));
        verify(leftFollowerMotor2).setNeutralMode(eq(TalonSRXNeutralMode.Brake));
        verify(leftFollowerMotor2).setInvertOutput(eq(HardwareConstants.DRIVETRAIN_LEFT_FOLLOWER2_INVERT_OUTPUT));
        verify(leftFollowerMotor2).setControlMode(eq(TalonSRXControlMode.Follower));
        verify(leftFollowerMotor2).set(eq((double)ElectronicsConstants.DRIVETRAIN_LEFT_MASTER_CAN_ID));
        verify(rightMotor).setNeutralMode(eq(TalonSRXNeutralMode.Brake));
        verify(rightMotor).setInvertOutput(eq(HardwareConstants.DRIVETRAIN_RIGHT_MASTER_INVERT_OUTPUT));
        verify(rightMotor).setInvertSensor(eq(HardwareConstants.DRIVETRAIN_RIGHT_INVERT_SENSOR));
        verify(rightMotor).setSensorType(TalonSRXFeedbackDevice.QuadEncoder);
        verify(rightMotor).setFeedbackFramePeriod(5);
        verify(rightMotor).setPIDFFramePeriod(5);
        verify(rightMotor).configureVelocityMeasurements();
        verify(rightMotor).setSelectedSlot(eq(0));
        verify(rightFollowerMotor1).setControlMode(eq(TalonSRXControlMode.Follower));
        verify(rightFollowerMotor1).setNeutralMode(eq(TalonSRXNeutralMode.Brake));
        verify(rightFollowerMotor1).setInvertOutput(eq(HardwareConstants.DRIVETRAIN_RIGHT_FOLLOWER1_INVERT_OUTPUT));
        verify(rightFollowerMotor1).set(eq((double)ElectronicsConstants.DRIVETRAIN_RIGHT_MASTER_CAN_ID));
        verify(rightFollowerMotor2).setControlMode(eq(TalonSRXControlMode.Follower));
        verify(rightFollowerMotor2).setNeutralMode(eq(TalonSRXNeutralMode.Brake));
        verify(rightFollowerMotor2).setInvertOutput(eq(HardwareConstants.DRIVETRAIN_RIGHT_FOLLOWER2_INVERT_OUTPUT));
        verify(rightFollowerMotor2).set(eq((double)ElectronicsConstants.DRIVETRAIN_RIGHT_MASTER_CAN_ID));

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
        verifyNoMoreInteractions(leftFollowerMotor1);
        verifyNoMoreInteractions(rightFollowerMotor1);
        verifyNoMoreInteractions(leftFollowerMotor2);
        verifyNoMoreInteractions(rightFollowerMotor2);
    }
}
