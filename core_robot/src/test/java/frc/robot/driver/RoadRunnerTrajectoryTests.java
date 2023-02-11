package frc.robot.driver;

import org.junit.jupiter.api.Test;

import frc.robot.driver.common.TrajectoryManager;

public class RoadRunnerTrajectoryTests
{
    @Test
    public void verifyTrajectoryGeneration()
    {
        TrajectoryManager trajectoryManager = new TrajectoryManager();
        RoadRunnerTrajectoryGenerator.generateTrajectories(trajectoryManager);
        trajectoryManager.buildAll();
    }
}