package frc.team1318.robot.driver.controltasks;

import frc.team1318.robot.HardwareConstants;
import frc.team1318.robot.driver.Operation;
import frc.team1318.robot.driver.common.IControlTask;
import frc.team1318.robot.mechanisms.DriveTrainMechanism;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Segment;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.modifiers.TankModifier;

public class FollowPathTask extends TimedTask implements IControlTask
{
    private static final double DEFAULT_TIMESTEP = 0.05;
    private static final double MAX_VELOCITY = 68.0;
    private static final double MAX_ACCELERATION = 80.0;
    private static final double MAX_JERK = 2400.0;

    private final Trajectory leftTrajectory;
    private final Trajectory rightTrajectory;
    private final int trajectoryLength;
    private final double timestep;

    private DriveTrainMechanism driveTrain;

    private double startLeftPosition;
    private double startRightPosition;
    //private double startAngle;

    /**
     * Initializes a new FollowPathTask
     * @param trajectory to follow
     * @param timestep between the different segments of the trajectory
     * @param duration to perform the task in seconds
     */
    private FollowPathTask(Trajectory trajectory, double timestep, double duration)
    {
        super(duration);

        TankModifier tankModifier = new TankModifier(trajectory);
        tankModifier.modify(HardwareConstants.DRIVETRAIN_WHEEL_SEPARATION_DISTANCE);

        this.leftTrajectory = tankModifier.getLeftTrajectory();
        this.rightTrajectory = tankModifier.getRightTrajectory();
        this.trajectoryLength = trajectory.length();
        this.timestep = timestep;
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

        double duration = trajectory.length() * timestep;
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

        this.startLeftPosition = this.driveTrain.getLeftPosition();
        this.startRightPosition = this.driveTrain.getRightPosition();

        //PositionManager positionManager = this.getInjector().getInstance(PositionManager.class);
        //this.startAngle = positionManager.getNavxAngle();

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

        Segment currentLeftSegment = this.leftTrajectory.get((int)currentSegmentIndex);
        Segment currentRightSegment = this.rightTrajectory.get((int)currentSegmentIndex);

        this.setAnalogOperationState(Operation.DriveTrainLeftPosition, this.startLeftPosition + currentLeftSegment.position);
        this.setAnalogOperationState(Operation.DriveTrainRightPosition, this.startRightPosition + currentRightSegment.position);
        this.setAnalogOperationState(Operation.DriveTrainLeftVelocity, currentLeftSegment.velocity);
        this.setAnalogOperationState(Operation.DriveTrainRightVelocity, currentRightSegment.velocity);
        this.setAnalogOperationState(Operation.DriveTrainLeftAcceleration, currentLeftSegment.acceleration);
        this.setAnalogOperationState(Operation.DriveTrainRightAcceleration, currentRightSegment.acceleration);
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

        this.setAnalogOperationState(Operation.DriveTrainLeftPosition, 0.0);
        this.setAnalogOperationState(Operation.DriveTrainRightPosition, 0.0);
        this.setAnalogOperationState(Operation.DriveTrainLeftVelocity, 0.0);
        this.setAnalogOperationState(Operation.DriveTrainRightVelocity, 0.0);
        this.setAnalogOperationState(Operation.DriveTrainLeftAcceleration, 0.0);
        this.setAnalogOperationState(Operation.DriveTrainRightAcceleration, 0.0);

        this.setDigitalOperationState(Operation.DriveTrainUsePathMode, false);
    }
}
