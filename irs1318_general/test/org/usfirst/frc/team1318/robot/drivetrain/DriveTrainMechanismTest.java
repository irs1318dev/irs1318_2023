package org.usfirst.frc.team1318.robot.drivetrain;

import static org.mockito.Matchers.eq;
import static org.mockito.Mockito.doReturn;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.verifyNoMoreInteractions;

import org.junit.Test;
import org.usfirst.frc.team1318.robot.ElectronicsConstants;
import org.usfirst.frc.team1318.robot.TestWpilibProvider;
import org.usfirst.frc.team1318.robot.common.IDashboardLogger;
import org.usfirst.frc.team1318.robot.common.wpilib.IEncoder;
import org.usfirst.frc.team1318.robot.common.wpilib.IMotor;
import org.usfirst.frc.team1318.robot.common.wpilib.ITimer;
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
        IMotor leftMotor = testProvider.getTalon(ElectronicsConstants.DRIVETRAIN_LEFT_TALON_CHANNEL);
        IMotor rightMotor = testProvider.getTalon(ElectronicsConstants.DRIVETRAIN_RIGHT_TALON_CHANNEL);
        IEncoder leftEncoder = testProvider.getEncoder(
            ElectronicsConstants.DRIVETRAIN_LEFT_ENCODER_CHANNEL_A,
            ElectronicsConstants.DRIVETRAIN_LEFT_ENCODER_CHANNEL_B);
        IEncoder rightEncoder = testProvider.getEncoder(
            ElectronicsConstants.DRIVETRAIN_RIGHT_ENCODER_CHANNEL_A,
            ElectronicsConstants.DRIVETRAIN_RIGHT_ENCODER_CHANNEL_B);

        Driver driver = mock(Driver.class);

        DriveTrainMechanism driveTrainMechanism = new DriveTrainMechanism(logger, timer, testProvider);
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
        doReturn(0.0).when(rightEncoder).getDistance();
        doReturn(0.0).when(rightEncoder).getRate();
        doReturn(0).when(rightEncoder).get();
        doReturn(0.0).when(leftEncoder).getDistance();
        doReturn(0.0).when(leftEncoder).getRate();
        doReturn(0).when(leftEncoder).get();
        doReturn(0.0).when(timer).get();

        driveTrainMechanism.update();

        verify(leftMotor).set(eq(0.0));
        verify(rightMotor).set(eq(-0.0));
        verify(leftEncoder).getDistance();
        verify(leftEncoder).getRate();
        verify(leftEncoder).get();
        verify(rightEncoder).getDistance();
        verify(rightEncoder).getRate();
        verify(rightEncoder).get();
        verifyNoMoreInteractions(leftMotor);
        verifyNoMoreInteractions(rightMotor);
        verifyNoMoreInteractions(leftEncoder);
        verifyNoMoreInteractions(rightEncoder);
    }

    @Test
    public void testStop()
    {
        IDashboardLogger logger = mock(IDashboardLogger.class);
        ITimer timer = mock(ITimer.class);
        TestWpilibProvider testProvider = new TestWpilibProvider();
        IMotor leftMotor = testProvider.getTalon(ElectronicsConstants.DRIVETRAIN_LEFT_TALON_CHANNEL);
        IMotor rightMotor = testProvider.getTalon(ElectronicsConstants.DRIVETRAIN_RIGHT_TALON_CHANNEL);
        IEncoder leftEncoder = testProvider.getEncoder(
            ElectronicsConstants.DRIVETRAIN_LEFT_ENCODER_CHANNEL_A,
            ElectronicsConstants.DRIVETRAIN_LEFT_ENCODER_CHANNEL_B);
        IEncoder rightEncoder = testProvider.getEncoder(
            ElectronicsConstants.DRIVETRAIN_RIGHT_ENCODER_CHANNEL_A,
            ElectronicsConstants.DRIVETRAIN_RIGHT_ENCODER_CHANNEL_B);

        Driver driver = mock(Driver.class);

        DriveTrainMechanism driveTrainMechanism = new DriveTrainMechanism(logger, timer, testProvider);
        driveTrainMechanism.setDriver(driver);
        driveTrainMechanism.stop();

        verify(leftMotor).set(eq(0.0));
        verify(rightMotor).set(eq(0.0));
        verify(leftEncoder).reset();
        verify(rightEncoder).reset();
        verifyNoMoreInteractions(leftMotor);
        verifyNoMoreInteractions(rightMotor);
        verifyNoMoreInteractions(leftEncoder);
        verifyNoMoreInteractions(rightEncoder);
    }
}
