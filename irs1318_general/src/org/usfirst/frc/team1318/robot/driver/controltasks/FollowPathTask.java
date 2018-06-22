package org.usfirst.frc.team1318.robot.driver.controltasks;

import org.usfirst.frc.team1318.robot.TuningConstants;
import org.usfirst.frc.team1318.robot.driver.Operation;
import org.usfirst.frc.team1318.robot.driver.common.IControlTask;
import org.usfirst.frc.team1318.robot.drivetrain.DriveTrainMechanism;
import org.usfirst.frc.team1318.robot.general.PositionManager;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;

public class FollowPathTask extends TimedTask implements IControlTask
{
    private static final double DEFAULT_TIMESTEP = 0.05;
    private static final double MAX_VELOCITY = 68.0;
    private static final double MAX_ACCELERATION = 80.0;
    private static final double MAX_JERK = 2400.0;

    private final Trajectory trajectory;
    private final double timestep;

    private DriveTrainMechanism driveTrain;

    private double startLeftPosition;
    private double startRightPosition;
    private double startAngle;

    /**
     * Initializes a new FollowPathTask
     * @param trajectory to follow
     * @param timestep between the different segments of the trajectory
     * @param duration to perform the task in seconds
     */
    private FollowPathTask(Trajectory trajectory, double timestep, double duration)
    {
        super(duration);

        this.timestep = timestep;
        this.trajectory = trajectory;
    }

    /**
     * Create a task that follows the provided waypoints
     * @param waypoints to follow
     * @return a new task
     */
    public static FollowPathTask Create(Waypoint... waypoints)
    {
        return FollowPathTask.Create(FollowPathTask.DEFAULT_TIMESTEP, waypoints);
    }

    /**
     * Create a task that follows the provided waypoints
     * @param timestep between the different segments of the trajectory
     * @param waypoints to follow
     * @return a new task
     */
    public static FollowPathTask Create(double timestep, Waypoint... waypoints)
    {
        Trajectory.Config config = new Trajectory.Config(
            Trajectory.FitMethod.HERMITE_CUBIC,
            Trajectory.Config.SAMPLES_HIGH,
            timestep,
            FollowPathTask.MAX_VELOCITY,
            FollowPathTask.MAX_ACCELERATION,
            FollowPathTask.MAX_JERK);

        Trajectory trajectory = Pathfinder.generate(waypoints, config);

        double duration = trajectory.segments.length * timestep;
        return new FollowPathTask(trajectory, timestep, duration);
    }

    /**
     * Begin the current task
     */
    @Override
    public void begin()
    {
        super.begin();

        this.driveTrain = this.getInjector().getInstance(DriveTrainMechanism.class);

        PositionManager positionManager = this.getInjector().getInstance(PositionManager.class);

        this.startLeftPosition = this.driveTrain.getLeftPosition();
        this.startRightPosition = this.driveTrain.getRightPosition();
        this.startAngle = positionManager.getNavxAngle();

        this.setDigitalOperationState(Operation.DriveTrainUsePathMode, true);
        this.setAnalogOperationState(Operation.DriveTrainLeftPosition, this.startLeftPosition);
        this.setAnalogOperationState(Operation.DriveTrainRightPosition, this.startRightPosition);
        this.setAnalogOperationState(Operation.DriveTrainLeftVelocity, 0.0);
        this.setAnalogOperationState(Operation.DriveTrainRightVelocity, 0.0);
        this.setAnalogOperationState(Operation.DriveTrainOrientation, this.startAngle);
    }

    /**
     * Run an iteration of the current task and apply any control changes
     */
    @Override
    public void update()
    {
        double elapsedTime = this.timer.get() - this.startTime;

        this.setAnalogOperationState(Operation.DriveTrainLeftPosition, this.startLeftPosition);
        this.setAnalogOperationState(Operation.DriveTrainRightPosition, this.startRightPosition);
        this.setAnalogOperationState(Operation.DriveTrainLeftVelocity, 0.0);
        this.setAnalogOperationState(Operation.DriveTrainRightVelocity, 0.0);
        this.setAnalogOperationState(Operation.DriveTrainOrientation, this.startAngle);
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
        this.setAnalogOperationState(Operation.DriveTrainOrientation, 0.0);
    }

    /**
     * End the current task and reset control changes appropriately
     */
    @Override
    public void end()
    {
        super.end();

        this.setAnalogOperationState(Operation.DriveTrainLeftPosition, 0.0);
        this.setAnalogOperationState(Operation.DriveTrainRightPosition, 0.0);
        this.setAnalogOperationState(Operation.DriveTrainLeftVelocity, 0.0);
        this.setAnalogOperationState(Operation.DriveTrainRightVelocity, 0.0);
        this.setAnalogOperationState(Operation.DriveTrainOrientation, 0.0);

        this.setDigitalOperationState(Operation.DriveTrainUsePathMode, false);
    }

    /**
     * Checks whether this task has completed, or whether it should continue being processed
     * @return true if we should continue onto the next task, otherwise false (to keep processing this task)
     */
    @Override
    public boolean hasCompleted()
    {
        double leftEncoderTicks = this.driveTrain.getLeftPosition();
        double rightEncoderTicks = this.driveTrain.getRightPosition();

        // check how far away we are from the desired end location
        double leftDelta = Math.abs(0.0 - leftEncoderTicks);
        double rightDelta = Math.abs(0.0 - rightEncoderTicks);

        // return that we have completed this task if are within an acceptable distance
        // from the desired end location for both left and right. 
        return (leftDelta < TuningConstants.DRIVETRAIN_POSITIONAL_ACCEPTABLE_DELTA
            && rightDelta < TuningConstants.DRIVETRAIN_POSITIONAL_ACCEPTABLE_DELTA);
    }
}
