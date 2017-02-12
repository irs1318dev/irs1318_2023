package org.usfirst.frc.team1318.robot.drivetrain;

import static org.mockito.Matchers.eq;
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
}
