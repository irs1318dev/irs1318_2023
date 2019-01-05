package frc.robot.driver.controltasks;

import frc.robot.HardwareConstants;
import frc.robot.driver.Operation;
import frc.robot.mechanisms.DriveTrainMechanism;

public class DriveDistancePositionTimedTask extends TimedTask
{
    private final double distance;
    private final double velocity;

    private DriveTrainMechanism driveTrain;

    private double startLeftTicks;
    private double startRightTicks;
    private double endLeftTicks;
    private double endRightTicks;

    public DriveDistancePositionTimedTask(double speed, double distance, double duration)
    {
        super(duration);

        this.distance = distance;
        this.velocity = speed;
    }

    @Override
    public void begin()
    {
        super.begin();

        this.driveTrain = this.getInjector().getInstance(DriveTrainMechanism.class);

        this.startLeftTicks = this.driveTrain.getLeftPosition();
        this.startRightTicks = this.driveTrain.getRightPosition();

        this.endLeftTicks = this.startLeftTicks + this.distance / HardwareConstants.DRIVETRAIN_LEFT_PULSE_DISTANCE;
        this.endRightTicks = this.startRightTicks + this.distance / HardwareConstants.DRIVETRAIN_RIGHT_PULSE_DISTANCE;
    }

    @Override
    public void update()
    {
        this.setDigitalOperationState(Operation.DriveTrainUsePositionalMode, false);
        this.setAnalogOperationState(Operation.DriveTrainTurn, 0.0);
        this.setAnalogOperationState(Operation.DriveTrainMoveForward, this.velocity);
    }

    @Override
    public void end()
    {
        super.end();

        this.setDigitalOperationState(Operation.DriveTrainUsePositionalMode, false);
        this.setAnalogOperationState(Operation.DriveTrainTurn, 0.0);
        this.setAnalogOperationState(Operation.DriveTrainMoveForward, 0.0);
    }

    @Override
    public boolean hasCompleted()
    {
        if (super.hasCompleted())
        {
            return true;
        }

        double leftTicks = this.driveTrain.getLeftPosition();
        double rightTicks = this.driveTrain.getRightPosition();

        if (this.distance >= 0.0)
        {
            return leftTicks >= this.endLeftTicks
                ||
                rightTicks >= this.endRightTicks;
        }
        else
        {
            return leftTicks <= this.endLeftTicks
                ||
                rightTicks <= this.endRightTicks;
        }
    }
}
