package frc.team1318.robot.common.robotprovider;

import frc.team1318.robot.HardwareConstants;
import jaci.pathfinder.*;
import jaci.pathfinder.modifiers.TankModifier;

public class TrajectoryGenerator implements ITrajectoryGenerator
{
    private static final double DEFAULT_TIMESTEP = 0.05;
    private static final double MAX_VELOCITY = 68.0;
    private static final double MAX_ACCELERATION = 80.0;
    private static final double MAX_JERK = 2400.0;

    public TrajectoryGenerator()
    {
    }

    @Override
    public PathPlan generate(TrajectoryWaypoint... waypoints)
    {
        Trajectory.Config config = new Trajectory.Config(
            Trajectory.FitMethod.HERMITE_CUBIC,
            Trajectory.Config.SAMPLES_HIGH,
            TrajectoryGenerator.DEFAULT_TIMESTEP,
            TrajectoryGenerator.MAX_VELOCITY,
            TrajectoryGenerator.MAX_ACCELERATION,
            TrajectoryGenerator.MAX_JERK);

        Trajectory trajectory = Pathfinder.generate(this.translate(waypoints), config);

        double duration = trajectory.length() * TrajectoryGenerator.DEFAULT_TIMESTEP;
        TankModifier tankModifier = new TankModifier(trajectory);
        tankModifier.modify(HardwareConstants.DRIVETRAIN_WHEEL_SEPARATION_DISTANCE);

        return new PathPlan(
            new TrajectoryWrapper(tankModifier.getLeftTrajectory()),
            new TrajectoryWrapper(tankModifier.getRightTrajectory()),
            duration,
            TrajectoryGenerator.DEFAULT_TIMESTEP);
    }

    private Waypoint[] translate(TrajectoryWaypoint[] trajectoryWaypoints)
    {
        Waypoint[] waypoints = new Waypoint[trajectoryWaypoints.length];
        for (int i = 0; i < trajectoryWaypoints.length; i++)
        {
            TrajectoryWaypoint trajectoryWaypoint = trajectoryWaypoints[i];
            waypoints[i] =
                new Waypoint(
                    trajectoryWaypoint.getY(),
                    -trajectoryWaypoint.getX(),
                    -Pathfinder.d2r(trajectoryWaypoint.getAngle()));
        }

        return waypoints;
    }
}