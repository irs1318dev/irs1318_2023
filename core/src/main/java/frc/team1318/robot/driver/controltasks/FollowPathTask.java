package frc.team1318.robot.driver.controltasks;

import frc.team1318.robot.common.robotprovider.*;
import frc.team1318.robot.driver.Operation;
import frc.team1318.robot.driver.common.IControlTask;
import frc.team1318.robot.mechanisms.DriveTrainMechanism;

public class FollowPathTask extends TimedTask implements IControlTask
{
    private final ITrajectory leftTrajectory;
    private final ITrajectory rightTrajectory;
    private final int trajectoryLength;
    private final double timestep;

    private DriveTrainMechanism driveTrain;

    private double startLeftPosition;
    private double startRightPosition;

    /**
     * Initializes a new FollowPathTask
     * @param pathPlan to follow
     */
    public FollowPathTask(PathPlan pathPlan)
    {
        super(pathPlan.getDuration());

        this.leftTrajectory = pathPlan.getLeft();
        this.rightTrajectory = pathPlan.getRight();

        this.trajectoryLength = leftTrajectory.length();
        this.timestep = pathPlan.getTimestep();
    }

    /**
     * Begin the current task
     */
    @Override
    public void begin()
    {
        super.begin();

        this.driveTrain = this.getInjector().getInstance(DriveTrainMechanism.class);

        this.startLeftPosition = this.driveTrain.getLeftPosition();
        this.startRightPosition = this.driveTrain.getRightPosition();

        this.setDigitalOperationState(Operation.DriveTrainUsePathMode, true);
        this.setAnalogOperationState(Operation.DriveTrainLeftPosition, this.startLeftPosition);
        this.setAnalogOperationState(Operation.DriveTrainRightPosition, this.startRightPosition);
        this.setAnalogOperationState(Operation.DriveTrainLeftVelocity, 0.0);
        this.setAnalogOperationState(Operation.DriveTrainRightVelocity, 0.0);
        this.setAnalogOperationState(Operation.DriveTrainLeftAcceleration, 0.0);
        this.setAnalogOperationState(Operation.DriveTrainRightAcceleration, 0.0);
    }

    /**
     * Run an iteration of the current task and apply any control changes
     */
    @Override
    public void update()
    {
        double elapsedTime = this.timer.get() - this.startTime;
        if (elapsedTime > this.duration)
        {
            elapsedTime = this.duration;
        }

        double currentSegmentIndex = Math.floor(elapsedTime / this.timestep);
        if (currentSegmentIndex >= this.trajectoryLength)
        {
            currentSegmentIndex = this.trajectoryLength - 1;
        }

        ISegment currentLeftSegment = this.leftTrajectory.get((int)currentSegmentIndex);
        ISegment currentRightSegment = this.rightTrajectory.get((int)currentSegmentIndex);

        this.setAnalogOperationState(Operation.DriveTrainLeftPosition, this.startLeftPosition + currentLeftSegment.getPosition());
        this.setAnalogOperationState(Operation.DriveTrainRightPosition, this.startRightPosition + currentRightSegment.getPosition());
        this.setAnalogOperationState(Operation.DriveTrainLeftVelocity, currentLeftSegment.getVelocity());
        this.setAnalogOperationState(Operation.DriveTrainRightVelocity, currentRightSegment.getVelocity());
        this.setAnalogOperationState(Operation.DriveTrainLeftAcceleration, currentLeftSegment.getAcceleration());
        this.setAnalogOperationState(Operation.DriveTrainRightAcceleration, currentRightSegment.getAcceleration());
    }

    /**
     * Cancel the current task and clear control changes
     */
    @Override
    public void stop()
    {
        super.stop();

        this.setDigitalOperationState(Operation.DriveTrainUsePathMode, false);
        this.setAnalogOperationState(Operation.DriveTrainLeftPosition, 0.0);
        this.setAnalogOperationState(Operation.DriveTrainRightPosition, 0.0);
        this.setAnalogOperationState(Operation.DriveTrainLeftVelocity, 0.0);
        this.setAnalogOperationState(Operation.DriveTrainRightVelocity, 0.0);
        this.setAnalogOperationState(Operation.DriveTrainLeftAcceleration, 0.0);
        this.setAnalogOperationState(Operation.DriveTrainRightAcceleration, 0.0);
    }

    /**
     * End the current task and reset control changes appropriately
     */
    @Override
    public void end()
    {
        super.end();

        this.setDigitalOperationState(Operation.DriveTrainUsePathMode, false);
        this.setAnalogOperationState(Operation.DriveTrainLeftPosition, 0.0);
        this.setAnalogOperationState(Operation.DriveTrainRightPosition, 0.0);
        this.setAnalogOperationState(Operation.DriveTrainLeftVelocity, 0.0);
        this.setAnalogOperationState(Operation.DriveTrainRightVelocity, 0.0);
        this.setAnalogOperationState(Operation.DriveTrainLeftAcceleration, 0.0);
        this.setAnalogOperationState(Operation.DriveTrainRightAcceleration, 0.0);
    }
}
