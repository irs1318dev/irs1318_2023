package org.usfirst.frc.team1318.robot.DriveTrain;

/**
 * Interface describing the drivetrain component, so that it could be mock-implemented for unit tests.
 * 
 * @author Will
 *
 */
public interface IDriveTrainComponent
{
    /**
     * set the power levels to the drive train
     * @param leftPower level to apply to the left motor
     * @param rightPower level to apply to the right motor
     */
    public void setDriveTrainPower(double leftPower, double rightPower);
    
    /**
     * sets the state of the shifters 
     * @param state what the shifter state should be
     */
    public void setShifterState(boolean state);

    /**
     * get the velocity from the left encoder
     * @return a value indicating the velocity
     */
    public double getLeftEncoderVelocity();

    /**
     * get the velocity from the right encoder
     * @return a value indicating the velocity
     */
    public double getRightEncoderVelocity();

    /**
     * get the distance from the left encoder
     * @return a value indicating the distance
     */
    public double getLeftEncoderDistance();

    /**
     * get the distance from the right encoder
     * @return a value indicating the distance
     */
    public double getRightEncoderDistance();
}
