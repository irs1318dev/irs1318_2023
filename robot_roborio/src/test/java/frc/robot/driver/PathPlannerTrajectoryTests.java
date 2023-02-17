package frc.robot.driver;

import org.junit.jupiter.api.Test;

import frc.robot.common.robotprovider.PathPlannerWrapper;
import frc.robot.driver.common.TrajectoryManager;

public class PathPlannerTrajectoryTests
{
    @Test
    public void verifyTrajectoryGeneration()
    {
        TrajectoryManager trajectoryManager = new TrajectoryManager();
        PathPlannerTrajectoryGenerator.generateTrajectories(trajectoryManager, new PathPlannerWrapper());
        trajectoryManager.buildAll();
    }
}