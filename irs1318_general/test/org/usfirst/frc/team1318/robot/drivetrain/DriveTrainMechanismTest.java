package org.usfirst.frc.team1318.robot.drivetrain;

import static org.mockito.Matchers.eq;
import static org.mockito.Mockito.doReturn;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.verifyNoMoreInteractions;

import org.junit.Test;
import org.usfirst.frc.team1318.robot.ElectronicsConstants;
import org.usfirst.frc.team1318.robot.TestWpilibProvider;
import org.usfirst.frc.team1318.robot.TuningConstants;
import org.usfirst.frc.team1318.robot.common.IDashboardLogger;
import org.usfirst.frc.team1318.robot.common.wpilib.ITalonSRX;
import org.usfirst.frc.team1318.robot.common.wpilib.ITimer;
import org.usfirst.frc.team1318.robot.common.wpilib.TalonSRXControlMode;
import org.usfirst.frc.team1318.robot.common.wpilib.TalonSRXFeedbackDevice;
import org.usfirst.frc.team1318.robot.common.wpilib.TalonSRXNeutralMode;
import org.usfirst.frc.team1318.robot.driver.Operation;
import org.usfirst.frc.team1318.robot.driver.common.Driver;

public class DriveTrainMechanismTest
{
    @Test
    public void testSetPower_Zero()
    {
        IDashboardLogger logger = mock(IDashboardLogger.class);
        ITimer timer = mock(ITimer.class);
        TestWpilibProvider testProvider = new TestWpilibProvider();
        ITalonSRX leftMotor = testProvider.getTalonSRX(ElectronicsConstants.DRIVETRAIN_LEFT_MOTOR_CAN_ID);
        ITalonSRX rightMotor = testProvider.getTalonSRX(ElectronicsConstants.DRIVETRAIN_RIGHT_MOTOR_CAN_ID);
        ITalonSRX leftFollowerMotor = testProvider.getTalonSRX(ElectronicsConstants.DRIVETRAIN_LEFT_FOLLOWER_CAN_ID);
        ITalonSRX rightFollowerMotor = testProvider.getTalonSRX(ElectronicsConstants.DRIVETRAIN_RIGHT_FOLLOWER_CAN_ID);

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
        verify(leftMotor).setInvertOutput(eq(false));
        verify(leftMotor).setInvertSensor(eq(true));
        verify(leftMotor).setSensorType(TalonSRXFeedbackDevice.QuadEncoder);
        verify(leftMotor).setSelectedSlot(eq(0));
        verify(leftFollowerMotor).setNeutralMode(eq(TalonSRXNeutralMode.Brake));
        verify(leftFollowerMotor).setInvertOutput(eq(false));
        verify(leftFollowerMotor).setControlMode(eq(TalonSRXControlMode.Follower));
        verify(leftFollowerMotor).set(eq((double)ElectronicsConstants.DRIVETRAIN_LEFT_MOTOR_CAN_ID));
        verify(rightMotor).setNeutralMode(eq(TalonSRXNeutralMode.Brake));
        verify(rightMotor).setInvertOutput(eq(true));
        verify(rightMotor).setInvertSensor(eq(true));
        verify(rightMotor).setSensorType(TalonSRXFeedbackDevice.QuadEncoder);
        verify(rightMotor).setSelectedSlot(eq(0));
        verify(rightFollowerMotor).setControlMode(eq(TalonSRXControlMode.Follower));
        verify(rightFollowerMotor).setNeutralMode(eq(TalonSRXNeutralMode.Brake));
        verify(rightFollowerMotor).setInvertOutput(eq(true));
        verify(rightFollowerMotor).set(eq((double)ElectronicsConstants.DRIVETRAIN_RIGHT_MOTOR_CAN_ID));

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
        IDashboardLogger logger = mock(IDashboardLogger.class);
        ITimer timer = mock(ITimer.class);
        TestWpilibProvider testProvider = new TestWpilibProvider();
        ITalonSRX leftMotor = testProvider.getTalonSRX(ElectronicsConstants.DRIVETRAIN_LEFT_MOTOR_CAN_ID);
        ITalonSRX rightMotor = testProvider.getTalonSRX(ElectronicsConstants.DRIVETRAIN_RIGHT_MOTOR_CAN_ID);
        ITalonSRX leftFollowerMotor = testProvider.getTalonSRX(ElectronicsConstants.DRIVETRAIN_LEFT_FOLLOWER_CAN_ID);
        ITalonSRX rightFollowerMotor = testProvider.getTalonSRX(ElectronicsConstants.DRIVETRAIN_RIGHT_FOLLOWER_CAN_ID);

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
        verify(leftMotor).setInvertOutput(eq(false));
        verify(leftMotor).setInvertSensor(eq(true));
        verify(leftMotor).setSensorType(TalonSRXFeedbackDevice.QuadEncoder);
        verify(leftMotor).setSelectedSlot(eq(0));
        verify(leftFollowerMotor).setNeutralMode(eq(TalonSRXNeutralMode.Brake));
        verify(leftFollowerMotor).setInvertOutput(eq(false));
        verify(leftFollowerMotor).setControlMode(eq(TalonSRXControlMode.Follower));
        verify(leftFollowerMotor).set(eq((double)ElectronicsConstants.DRIVETRAIN_LEFT_MOTOR_CAN_ID));
        verify(rightMotor).setNeutralMode(eq(TalonSRXNeutralMode.Brake));
        verify(rightMotor).setInvertOutput(eq(true));
        verify(rightMotor).setInvertSensor(eq(true));
        verify(rightMotor).setSensorType(TalonSRXFeedbackDevice.QuadEncoder);
        verify(rightMotor).setSelectedSlot(eq(0));
        verify(rightFollowerMotor).setControlMode(eq(TalonSRXControlMode.Follower));
        verify(rightFollowerMotor).setNeutralMode(eq(TalonSRXNeutralMode.Brake));
        verify(rightFollowerMotor).setInvertOutput(eq(true));
        verify(rightFollowerMotor).set(eq((double)ElectronicsConstants.DRIVETRAIN_RIGHT_MOTOR_CAN_ID));

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
