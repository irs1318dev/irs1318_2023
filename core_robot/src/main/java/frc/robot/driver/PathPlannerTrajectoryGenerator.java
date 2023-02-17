package frc.robot.driver;

import frc.robot.TuningConstants;
import frc.robot.common.robotprovider.IPathPlanner;
import frc.robot.common.robotprovider.ITrajectory;
import frc.robot.common.robotprovider.PathPlannerWaypoint;
import frc.robot.driver.common.TrajectoryManager;

public class PathPlannerTrajectoryGenerator
{
    public static void generateTrajectories(TrajectoryManager trajectoryManager, IPathPlanner pathPlanner)
    {
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                new PathPlannerWaypoint(0.0, 0.0),
                new PathPlannerWaypoint(48.0, 0.0)),
            "goForward4ft");

        addTrajectory(trajectoryManager,
        pathPlanner.buildTrajectory(
            TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
            TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
            new PathPlannerWaypoint(0.0, 0.0, 180.0),
            new PathPlannerWaypoint(-4.0, 0.0, 180.0)),
            "goBackwards4inch");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                new PathPlannerWaypoint(0.0, 0.0, 90.0),
                new PathPlannerWaypoint(0.0, 48.0, 90.0)),
            "goLeft4ft");



        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                new PathPlannerWaypoint(0.0, 0.0),
                new PathPlannerWaypoint(84.0, 0.0)),
            "goForward7ft");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                new PathPlannerWaypoint(0.0, 0.0, 0.0, 0.0),
                new PathPlannerWaypoint(-1.0, 0.0, 180.0, 180.0)),
            "turn180Path");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                new PathPlannerWaypoint(0.0, 0.0, 180.0, 0.0),
                new PathPlannerWaypoint(-84.0, 0.0, 180.0, 180.0)),
            "goBack7ftRotate");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                new PathPlannerWaypoint(0.0, 0.0, 180.0, 0.0),
                new PathPlannerWaypoint(-1.0, 0.0, 180.0, 0.0)),
            "goBack6ft");
    }

    private static void addTrajectory(TrajectoryManager trajectoryManager, ITrajectory trajectory, String name)
    {
        try
        {
            trajectoryManager.addTrajectory(name, trajectory);
        }
        catch (Exception ex)
        {
            System.err.println("Encountered exception generating path " + name + ": " + ex.toString());
            if (TuningConstants.THROW_EXCEPTIONS)
            {
                throw ex;
            }
        }
    }
}