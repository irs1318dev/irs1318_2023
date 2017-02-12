package org.usfirst.frc.team1318.robot.drivetrain;

import static org.junit.Assert.assertEquals;
import static org.mockito.Matchers.eq;
import static org.mockito.Mockito.doReturn;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.verifyNoMoreInteractions;

import org.junit.Test;
import org.usfirst.frc.team1318.robot.TuningConstants;
import org.usfirst.frc.team1318.robot.common.IDashboardLogger;
import org.usfirst.frc.team1318.robot.common.wpilibmocks.IEncoder;
import org.usfirst.frc.team1318.robot.common.wpilibmocks.IMotor;

public class DriveTrainComponentTest
{
    @Test
    public void testSetPower_MaxPower()
    {
        IDashboardLogger logger = mock(IDashboardLogger.class);
        IMotor leftMotor = mock(IMotor.class);
        IMotor rightMotor = mock(IMotor.class);
        IEncoder leftEncoder = mock(IEncoder.class);
        IEncoder rightEncoder = mock(IEncoder.class);

        DriveTrainComponent driveTrainComponent = new DriveTrainComponent(logger, leftMotor, rightMotor, leftEncoder, rightEncoder);

        driveTrainComponent.setDriveTrainPower(TuningConstants.DRIVETRAIN_MAX_POWER_LEVEL, TuningConstants.DRIVETRAIN_MAX_POWER_LEVEL);

        verify(leftMotor).set(eq(TuningConstants.DRIVETRAIN_MAX_POWER_LEVEL));
        verify(rightMotor).set(eq(-TuningConstants.DRIVETRAIN_MAX_POWER_LEVEL));
        verifyNoMoreInteractions(leftMotor);
        verifyNoMoreInteractions(rightMotor);
        verifyNoMoreInteractions(leftEncoder);
        verifyNoMoreInteractions(rightEncoder);
    }

    @Test
    public void testSetPower_MinPower()
    {
        IDashboardLogger logger = mock(IDashboardLogger.class);
        IMotor leftMotor = mock(IMotor.class);
        IMotor rightMotor = mock(IMotor.class);
        IEncoder leftEncoder = mock(IEncoder.class);
        IEncoder rightEncoder = mock(IEncoder.class);

        DriveTrainComponent driveTrainComponent = new DriveTrainComponent(logger, leftMotor, rightMotor, leftEncoder, rightEncoder);

        driveTrainComponent.setDriveTrainPower(-TuningConstants.DRIVETRAIN_MAX_POWER_LEVEL, -TuningConstants.DRIVETRAIN_MAX_POWER_LEVEL);

        verify(leftMotor).set(eq(-TuningConstants.DRIVETRAIN_MAX_POWER_LEVEL));
        verify(rightMotor).set(eq(TuningConstants.DRIVETRAIN_MAX_POWER_LEVEL));
        verifyNoMoreInteractions(leftMotor);
        verifyNoMoreInteractions(rightMotor);
        verifyNoMoreInteractions(leftEncoder);
        verifyNoMoreInteractions(rightEncoder);
    }

    @Test
    public void testGetTicks_None()
    {
        IDashboardLogger logger = mock(IDashboardLogger.class);
        IMotor leftMotor = mock(IMotor.class);
        IMotor rightMotor = mock(IMotor.class);
        IEncoder leftEncoder = mock(IEncoder.class);
        IEncoder rightEncoder = mock(IEncoder.class);

        doReturn(0).when(leftEncoder).get();
        doReturn(0).when(rightEncoder).get();

        DriveTrainComponent driveTrainComponent = new DriveTrainComponent(logger, leftMotor, rightMotor, leftEncoder, rightEncoder);

        int leftTicks = driveTrainComponent.getLeftEncoderTicks();
        assertEquals(0, leftTicks);

        int rightTicks = driveTrainComponent.getRightEncoderTicks();
        assertEquals(0, rightTicks);

        verify(leftEncoder).get();
        verify(rightEncoder).get();
        verifyNoMoreInteractions(leftMotor);
        verifyNoMoreInteractions(rightMotor);
        verifyNoMoreInteractions(leftEncoder);
        verifyNoMoreInteractions(rightEncoder);
    }

    @Test
    public void testGetTicks_Some()
    {
        IDashboardLogger logger = mock(IDashboardLogger.class);
        IMotor leftMotor = mock(IMotor.class);
        IMotor rightMotor = mock(IMotor.class);
        IEncoder leftEncoder = mock(IEncoder.class);
        IEncoder rightEncoder = mock(IEncoder.class);

        doReturn(12775).when(leftEncoder).get();
        doReturn(14773).when(rightEncoder).get();

        DriveTrainComponent driveTrainComponent = new DriveTrainComponent(logger, leftMotor, rightMotor, leftEncoder, rightEncoder);

        int leftTicks = driveTrainComponent.getLeftEncoderTicks();
        assertEquals(-12775, leftTicks);

        int rightTicks = driveTrainComponent.getRightEncoderTicks();
        assertEquals(14773, rightTicks);

        verify(leftEncoder).get();
        verify(rightEncoder).get();
        verifyNoMoreInteractions(leftMotor);
        verifyNoMoreInteractions(rightMotor);
        verifyNoMoreInteractions(leftEncoder);
        verifyNoMoreInteractions(rightEncoder);
    }

    @Test
    public void testSetPower_NoPower()
    {
        IDashboardLogger logger = mock(IDashboardLogger.class);
        IMotor leftMotor = mock(IMotor.class);
        IMotor rightMotor = mock(IMotor.class);
        IEncoder leftEncoder = mock(IEncoder.class);
        IEncoder rightEncoder = mock(IEncoder.class);

        DriveTrainComponent driveTrainComponent = new DriveTrainComponent(logger, leftMotor, rightMotor, leftEncoder, rightEncoder);

        driveTrainComponent.setDriveTrainPower(0.0, -0.0);

        verify(leftMotor).set(eq(0.0));
        verify(rightMotor).set(eq(0.0));
        verifyNoMoreInteractions(leftMotor);
        verifyNoMoreInteractions(rightMotor);
        verifyNoMoreInteractions(leftEncoder);
        verifyNoMoreInteractions(rightEncoder);
    }

    @Test
    public void testGetDistance_None()
    {
        IDashboardLogger logger = mock(IDashboardLogger.class);
        IMotor leftMotor = mock(IMotor.class);
        IMotor rightMotor = mock(IMotor.class);
        IEncoder leftEncoder = mock(IEncoder.class);
        IEncoder rightEncoder = mock(IEncoder.class);

        doReturn(0.0).when(leftEncoder).getDistance();
        doReturn(0.0).when(rightEncoder).getDistance();

        DriveTrainComponent driveTrainComponent = new DriveTrainComponent(logger, leftMotor, rightMotor, leftEncoder, rightEncoder);

        double leftDistance = driveTrainComponent.getLeftEncoderDistance();
        assertEquals(0.0, leftDistance, 0.001);

        double rightDistance = driveTrainComponent.getRightEncoderDistance();
        assertEquals(0.0, rightDistance, 0.001);

        verify(leftEncoder).getDistance();
        verify(rightEncoder).getDistance();
        verifyNoMoreInteractions(leftMotor);
        verifyNoMoreInteractions(rightMotor);
        verifyNoMoreInteractions(leftEncoder);
        verifyNoMoreInteractions(rightEncoder);
    }

    @Test
    public void testGetDistance_Some()
    {
        IDashboardLogger logger = mock(IDashboardLogger.class);
        IMotor leftMotor = mock(IMotor.class);
        IMotor rightMotor = mock(IMotor.class);
        IEncoder leftEncoder = mock(IEncoder.class);
        IEncoder rightEncoder = mock(IEncoder.class);

        doReturn(25.5).when(leftEncoder).getDistance();
        doReturn(25.5).when(rightEncoder).getDistance();

        DriveTrainComponent driveTrainComponent = new DriveTrainComponent(logger, leftMotor, rightMotor, leftEncoder, rightEncoder);

        double leftDistance = driveTrainComponent.getLeftEncoderDistance();
        assertEquals(-25.5, leftDistance, 0.001);

        double rightDistance = driveTrainComponent.getRightEncoderDistance();
        assertEquals(25.5, rightDistance, 0.001);

        verify(leftEncoder).getDistance();
        verify(rightEncoder).getDistance();
        verifyNoMoreInteractions(leftMotor);
        verifyNoMoreInteractions(rightMotor);
        verifyNoMoreInteractions(leftEncoder);
        verifyNoMoreInteractions(rightEncoder);
    }

    @Test
    public void testGetVelocity_None()
    {
        IDashboardLogger logger = mock(IDashboardLogger.class);
        IMotor leftMotor = mock(IMotor.class);
        IMotor rightMotor = mock(IMotor.class);
        IEncoder leftEncoder = mock(IEncoder.class);
        IEncoder rightEncoder = mock(IEncoder.class);

        doReturn(0.0).when(leftEncoder).getDistance();
        doReturn(0.0).when(rightEncoder).getDistance();

        DriveTrainComponent driveTrainComponent = new DriveTrainComponent(logger, leftMotor, rightMotor, leftEncoder, rightEncoder);

        double leftVelocity = driveTrainComponent.getLeftEncoderVelocity();
        assertEquals(0.0, leftVelocity, 0.001);

        double rightVelocity = driveTrainComponent.getRightEncoderVelocity();
        assertEquals(0.0, rightVelocity, 0.001);

        verify(leftEncoder).getRate();
        verify(rightEncoder).getRate();
        verifyNoMoreInteractions(leftMotor);
        verifyNoMoreInteractions(rightMotor);
        verifyNoMoreInteractions(leftEncoder);
        verifyNoMoreInteractions(rightEncoder);
    }

    @Test
    public void testGetVelocity_Some()
    {
        IDashboardLogger logger = mock(IDashboardLogger.class);
        IMotor leftMotor = mock(IMotor.class);
        IMotor rightMotor = mock(IMotor.class);
        IEncoder leftEncoder = mock(IEncoder.class);
        IEncoder rightEncoder = mock(IEncoder.class);

        doReturn(21.7).when(leftEncoder).getRate();
        doReturn(21.7).when(rightEncoder).getRate();

        DriveTrainComponent driveTrainComponent = new DriveTrainComponent(logger, leftMotor, rightMotor, leftEncoder, rightEncoder);

        double leftVelocity = driveTrainComponent.getLeftEncoderVelocity();
        assertEquals(-21.7, leftVelocity, 0.001);

        double rightVelocity = driveTrainComponent.getRightEncoderVelocity();
        assertEquals(21.7, rightVelocity, 0.001);

        verify(leftEncoder).getRate();
        verify(rightEncoder).getRate();
        verifyNoMoreInteractions(leftMotor);
        verifyNoMoreInteractions(rightMotor);
        verifyNoMoreInteractions(leftEncoder);
        verifyNoMoreInteractions(rightEncoder);
    }

    @Test
    public void testReset()
    {
        IDashboardLogger logger = mock(IDashboardLogger.class);
        IMotor leftMotor = mock(IMotor.class);
        IMotor rightMotor = mock(IMotor.class);
        IEncoder leftEncoder = mock(IEncoder.class);
        IEncoder rightEncoder = mock(IEncoder.class);

        DriveTrainComponent driveTrainComponent = new DriveTrainComponent(logger, leftMotor, rightMotor, leftEncoder, rightEncoder);
        driveTrainComponent.reset();

        verify(leftEncoder).reset();
        verify(rightEncoder).reset();
        verifyNoMoreInteractions(leftMotor);
        verifyNoMoreInteractions(rightMotor);
        verifyNoMoreInteractions(leftEncoder);
        verifyNoMoreInteractions(rightEncoder);
    }
}
