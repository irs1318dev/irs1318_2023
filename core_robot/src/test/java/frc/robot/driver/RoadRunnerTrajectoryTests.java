package frc.robot.driver;

import org.junit.jupiter.api.Test;

import frc.robot.driver.common.PathManager;

public class RoadRunnerTrajectoryTests
{
    @Test
    public void verifyTrajectoryGeneration()
    {
        PathManager pathManager = new PathManager();
        RoadRunnerTrajectoryGenerator.generateTrajectories(pathManager);
    }
}