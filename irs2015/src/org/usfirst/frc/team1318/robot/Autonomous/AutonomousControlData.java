package org.usfirst.frc.team1318.robot.Autonomous;

/**
 * Class simply holding data about the current control settings for autonomous mode.  
 * 
 * @author Will
 *
 */
public class AutonomousControlData
{
    private boolean collectorExtend;
    private boolean collectorRetract;
    private boolean collectorCollect;
    private boolean collectorExpel;
    private boolean shooterAngle;
    private int shooterMode;
    private boolean shooterShoot;
    private double driveTrainXVelocity;
    private double driveTrainYVelocity;
    private double driveTrainLeftPosition;
    private double driveTrainRightPosition;
    private boolean driveTrainPositionMode;
    private boolean driveTrainSimpleMode;
    
    /**
     * Initializes a new AutonomousControlData
     */
    public AutonomousControlData()
    {
        this.collectorExtend = false;
        this.collectorRetract = false;
        this.collectorCollect = false;
        this.collectorExpel = false;
        this.shooterAngle = false;
        this.shooterMode = 3;
        this.shooterShoot = false;
        this.driveTrainXVelocity = 0.0;
        this.driveTrainYVelocity = 0.0;
        this.driveTrainSimpleMode = false;
        this.driveTrainLeftPosition = 0.0;
        this.driveTrainRightPosition = 0.0;
        this.driveTrainPositionMode = false;
    }

    public boolean getCollectorExtend()
    {
        return this.collectorExtend;
    }

    public void setCollectorExtend(boolean collectorExtend)
    {
        this.collectorExtend = collectorExtend;
    }

    public boolean getCollectorRetract()
    {
        return this.collectorRetract;
    }

    public void setCollectorRetract(boolean collectorRetract)
    {
        this.collectorRetract = collectorRetract;
    }

    public boolean getCollectorCollect()
    {
        return this.collectorCollect;
    }

    public void setCollectorCollect(boolean collectorCollect)
    {
        this.collectorCollect = collectorCollect;
    }

    public boolean getCollectorExpel()
    {
        return this.collectorExpel;
    }

    public void setCollectorExpel(boolean collectorExpel)
    {
        this.collectorExpel = collectorExpel;
    }

    public boolean getShooterAngle()
    {
        return this.shooterAngle;
    }

    public void setShooterAngle(boolean shooterAngle)
    {
        this.shooterAngle = shooterAngle;
    }

    public int getShooterMode()
    {
        return this.shooterMode;
    }

    public void setShooterMode(int shooterMode)
    {
        this.shooterMode = shooterMode;
    }

    public boolean getShooterShoot()
    {
        return this.shooterShoot;
    }

    public void setShooterShoot(boolean shooterShoot)
    {
        this.shooterShoot = shooterShoot;
    }

    public double getDriveTrainXVelocity()
    {
        return this.driveTrainXVelocity;
    }

    public void setDriveTrainXVelocity(double driveTrainXVelocity)
    {
        this.driveTrainXVelocity = driveTrainXVelocity;
    }

    public double getDriveTrainYVelocity()
    {
        return this.driveTrainYVelocity;
    }

    public void setDriveTrainYVelocity(double driveTrainYVelocity)
    {
        this.driveTrainYVelocity = driveTrainYVelocity;
    }

    public boolean getDriveTrainSimpleMode()
    {
        return this.driveTrainSimpleMode;
    }

    public void setDriveTrainSimpleMode(boolean driveTrainSimpleMode)
    {
        this.driveTrainSimpleMode = driveTrainSimpleMode;
    }

    public double getDriveTrainLeftPosition()
    {
        return this.driveTrainLeftPosition;
    }

    public void setDriveTrainLeftPosition(double driveTrainLeftPosition)
    {
        this.driveTrainLeftPosition = driveTrainLeftPosition;
    }

    public double getDriveTrainRightPosition()
    {
        return this.driveTrainRightPosition;
    }

    public void setDriveTrainRightPosition(double driveTrainRightPosition)
    {
        this.driveTrainRightPosition = driveTrainRightPosition;
    }

    public boolean getDriveTrainPositionMode()
    {
        return this.driveTrainPositionMode;
    }

    public void setDriveTrainPositionMode(boolean driveTrainPositionMode)
    {
        this.driveTrainPositionMode = driveTrainPositionMode;
    }
}
