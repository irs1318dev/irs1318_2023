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
import org.usfirst.frc.team1318.robot.common.wpilib.CANTalonControlMode;
import org.usfirst.frc.team1318.robot.common.wpilib.ICANTalon;
import org.usfirst.frc.team1318.robot.driver.Operation;
import org.usfirst.frc.team1318.robot.driver.common.Driver;

public class DriveTrainMechanismTest
{
    @Test
    public void testSetPower_Zero()
    {
        IDashboardLogger logger = mock(IDashboardLogger.class);
        TestWpilibProvider testProvider = new TestWpilibProvider();
        ICANTalon leftMotor = testProvider.getCANTalon(ElectronicsConstants.DRIVETRAIN_LEFT_MOTOR_CHANNEL);
        ICANTalon rightMotor = testProvider.getCANTalon(ElectronicsConstants.DRIVETRAIN_RIGHT_MOTOR_CHANNEL);
        ICANTalon leftFollowerMotor = testProvider.getCANTalon(ElectronicsConstants.DRIVETRAIN_LEFT_FOLLOWER_CHANNEL);
        ICANTalon rightFollowerMotor = testProvider.getCANTalon(ElectronicsConstants.DRIVETRAIN_RIGHT_FOLLOWER_CHANNEL);

        Driver driver = mock(Driver.class);

        DriveTrainMechanism driveTrainMechanism = new DriveTrainMechanism(logger, testProvider);
        driveTrainMechanism.setDriver(driver);

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
        doReturn(0.0).when(leftMotor).getSpeed();
        doReturn(0).when(leftMotor).getTicks();
        doReturn(0.0).when(rightMotor).getError();
        doReturn(0.0).when(rightMotor).getSpeed();
        doReturn(0).when(rightMotor).getTicks();

        driveTrainMechanism.update();

        verify(leftMotor).set(eq(0.0));
        verify(rightMotor).set(eq(0.0));
        verify(leftMotor).getError();
        verify(leftMotor).getSpeed();
        verify(leftMotor).getTicks();
        verify(rightMotor).getError();
        verify(rightMotor).getSpeed();
        verify(rightMotor).getTicks();

        verify(leftMotor).enableBrakeMode(eq(false));
        verify(leftMotor).reverseOutput(eq(false));
        verify(leftMotor).reverseSensor(eq(true));
        verify(leftFollowerMotor).enableBrakeMode(eq(false));
        verify(leftFollowerMotor).reverseOutput(eq(false));
        verify(leftFollowerMotor).changeControlMode(eq(CANTalonControlMode.Follower));
        verify(leftFollowerMotor).set(eq((double)ElectronicsConstants.DRIVETRAIN_LEFT_MOTOR_CHANNEL));
        verify(rightMotor).enableBrakeMode(eq(false));
        verify(rightMotor).reverseOutput(eq(true));
        verify(rightMotor).reverseSensor(eq(false));
        verify(rightFollowerMotor).changeControlMode(eq(CANTalonControlMode.Follower));
        verify(rightFollowerMotor).enableBrakeMode(eq(false));
        verify(rightFollowerMotor).reverseOutput(eq(true));
        verify(rightFollowerMotor).set(eq((double)ElectronicsConstants.DRIVETRAIN_RIGHT_MOTOR_CHANNEL));
        verify(leftMotor).setPIDF(
            eq(TuningConstants.DRIVETRAIN_VELOCITY_PID_LEFT_KP),
            eq(TuningConstants.DRIVETRAIN_VELOCITY_PID_LEFT_KI),
            eq(TuningConstants.DRIVETRAIN_VELOCITY_PID_LEFT_KD),
            eq(TuningConstants.DRIVETRAIN_VELOCITY_PID_LEFT_KF));
        verify(rightMotor).setPIDF(
            eq(TuningConstants.DRIVETRAIN_VELOCITY_PID_RIGHT_KP),
            eq(TuningConstants.DRIVETRAIN_VELOCITY_PID_RIGHT_KI),
            eq(TuningConstants.DRIVETRAIN_VELOCITY_PID_RIGHT_KD),
            eq(TuningConstants.DRIVETRAIN_VELOCITY_PID_RIGHT_KF));
        verifyNoMoreInteractions(leftMotor);
        verifyNoMoreInteractions(rightMotor);
        verifyNoMoreInteractions(leftFollowerMotor);
        verifyNoMoreInteractions(rightFollowerMotor);
    }

    @Test
    public void testStop()
    {
        IDashboardLogger logger = mock(IDashboardLogger.class);
        TestWpilibProvider testProvider = new TestWpilibProvider();
        ICANTalon leftMotor = testProvider.getCANTalon(ElectronicsConstants.DRIVETRAIN_LEFT_MOTOR_CHANNEL);
        ICANTalon rightMotor = testProvider.getCANTalon(ElectronicsConstants.DRIVETRAIN_RIGHT_MOTOR_CHANNEL);
        ICANTalon leftFollowerMotor = testProvider.getCANTalon(ElectronicsConstants.DRIVETRAIN_LEFT_FOLLOWER_CHANNEL);
        ICANTalon rightFollowerMotor = testProvider.getCANTalon(ElectronicsConstants.DRIVETRAIN_RIGHT_FOLLOWER_CHANNEL);

        Driver driver = mock(Driver.class);

        DriveTrainMechanism driveTrainMechanism = new DriveTrainMechanism(logger, testProvider);
        driveTrainMechanism.setDriver(driver);
        driveTrainMechanism.stop();

        verify(leftMotor).set(eq(0.0));
        verify(rightMotor).set(eq(0.0));
        verify(rightMotor).reset();
        verify(leftMotor).reset();
        verify(leftMotor).enableBrakeMode(eq(false));
        verify(leftMotor).reverseOutput(eq(false));
        verify(leftMotor).reverseSensor(eq(true));
        verify(leftFollowerMotor).enableBrakeMode(eq(false));
        verify(leftFollowerMotor).reverseOutput(eq(false));
        verify(leftFollowerMotor).changeControlMode(eq(CANTalonControlMode.Follower));
        verify(leftFollowerMotor).set(eq(ElectronicsConstants.DRIVETRAIN_LEFT_MOTOR_CHANNEL));
        verify(rightMotor).enableBrakeMode(eq(false));
        verify(rightMotor).reverseOutput(eq(true));
        verify(rightMotor).reverseSensor(eq(false));
        verify(rightFollowerMotor).changeControlMode(eq(CANTalonControlMode.Follower));
        verify(rightFollowerMotor).enableBrakeMode(eq(false));
        verify(rightFollowerMotor).reverseOutput(eq(true));
        verify(rightFollowerMotor).set(eq(ElectronicsConstants.DRIVETRAIN_RIGHT_MOTOR_CHANNEL));
        verifyNoMoreInteractions(leftMotor);
        verifyNoMoreInteractions(rightMotor);
        verifyNoMoreInteractions(leftFollowerMotor);
        verifyNoMoreInteractions(rightFollowerMotor);
    }
}
