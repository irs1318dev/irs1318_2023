package frc.lib.robotprovider;

import com.pathplanner.lib.*;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.helpers.Helpers;

public class PathPlannerWrapper implements IPathPlanner
{
    public PathPlannerWrapper()
    {
    }

    @Override
    public ITrajectory loadTrajectory(String name, double maxVelocity, double maxAcceleration)
    {
        return this.loadTrajectory(name, maxVelocity, maxAcceleration, false);
    }

    @Override
    public ITrajectory loadTrajectory(String name, double maxVelocity, double maxAcceleration, boolean reversed)
    {
        PathPlannerTrajectory trajectory = PathPlanner.loadPath(name, maxVelocity, maxAcceleration, reversed);
        return new PathPlannerTrajectoryWrapper(trajectory);
    }

    @Override
    public ITrajectory buildTrajectory(
        double maxVelocity,
        double maxAcceleration,
        PathPlannerWaypoint firstWaypoint,
        PathPlannerWaypoint secondWaypoint,
        PathPlannerWaypoint... otherWaypoints)
    {
        return new PathPlannerTrajectoryWrapper(
            PathPlanner.generatePath(
                new PathConstraints(maxVelocity * Helpers.METERS_PER_INCH, maxAcceleration * Helpers.METERS_PER_INCH),
                this.convertWaypoint(firstWaypoint),
                this.convertWaypoint(secondWaypoint),
                this.convertWaypoints(otherWaypoints)));
    }

    private PathPoint convertWaypoint(PathPlannerWaypoint waypoint)
    {
        return new PathPoint(
            new Translation2d(waypoint.x * Helpers.METERS_PER_INCH, waypoint.y * Helpers.METERS_PER_INCH),
            Rotation2d.fromDegrees(waypoint.heading),
            Rotation2d.fromDegrees(waypoint.orientation),
            waypoint.velocityOverride == -1.0 ? -1.0 : waypoint.velocityOverride * Helpers.METERS_PER_INCH); // -1.0 is a special value
    }

    private PathPoint[] convertWaypoints(PathPlannerWaypoint[] otherWaypoints)
    {
        if (otherWaypoints == null)
        {
            return null;
        }

        PathPoint[] otherPoints = new PathPoint[otherWaypoints.length];
        for (int i = 0; i < otherWaypoints.length; i++)
        {
            otherPoints[i] = this.convertWaypoint(otherWaypoints[i]);
        }

        return otherPoints;
    }

    private class PathPlannerTrajectoryWrapper implements ITrajectory
    {
        private final PathPlannerTrajectory wrappedObject;

        PathPlannerTrajectoryWrapper(PathPlannerTrajectory wrappedObject)
        {
            this.wrappedObject = wrappedObject;
        }

        @Override
        public double getDuration()
        {
            return this.wrappedObject.getTotalTimeSeconds();
        }

        @Override
        public TrajectoryState get(double time)
        {
            PathPlannerState state = (PathPlannerState)this.wrappedObject.sample(time);
            Pose2d poseMeters = state.poseMeters;
            return new TrajectoryState(
                poseMeters.getX() * Helpers.INCHES_PER_METER,
                poseMeters.getY() * Helpers.INCHES_PER_METER,
                state.holonomicRotation.getDegrees(),
                poseMeters.getRotation().getCos() * state.velocityMetersPerSecond * Helpers.INCHES_PER_METER,
                poseMeters.getRotation().getSin() * state.velocityMetersPerSecond * Helpers.INCHES_PER_METER,
                state.holonomicAngularVelocityRadPerSec * Helpers.RADIANS_TO_DEGREES);
        }
        
    }
}
