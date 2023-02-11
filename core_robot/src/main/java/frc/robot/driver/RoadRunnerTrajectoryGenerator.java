package frc.robot.driver;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.util.Arrays;

import com.acmerobotics.roadrunner.geometry.*;
import com.acmerobotics.roadrunner.trajectory.*;
import com.acmerobotics.roadrunner.trajectory.constraints.*;

import de.siegmar.fastcsv.writer.CsvWriter;
import frc.robot.HardwareConstants;
import frc.robot.TuningConstants;
import frc.robot.common.*;
import frc.robot.common.robotprovider.ITrajectory;
import frc.robot.common.robotprovider.TrajectoryState;
import frc.robot.driver.common.TrajectoryManager;
import frc.robot.driver.common.TrajectoryWrapper;

public class RoadRunnerTrajectoryGenerator
{
    private static final TrajectoryVelocityConstraint velocityConstraint =
        new MinVelocityConstraint(
            Arrays.asList(
                new SwerveVelocityConstraint(
                    TuningConstants.DRIVETRAIN_MAX_MODULE_PATH_VELOCITY,
                    HardwareConstants.DRIVETRAIN_HORIZONTAL_WHEEL_SEPERATION_DISTANCE,
                    HardwareConstants.DRIVETRAIN_VERTICAL_WHEEL_SEPERATION_DISTANCE),
                new AngularVelocityConstraint(TuningConstants.DRIVETRAIN_MAX_PATH_TURN_VELOCITY * Helpers.DEGREES_TO_RADIANS),
                new TranslationalVelocityConstraint(TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY)));

    private static final TrajectoryAccelerationConstraint accelerationConstraint =
            new ProfileAccelerationConstraint(TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION);

    public static void main(String[] args)
    {
        TrajectoryManager pathManager = new TrajectoryManager();
        RoadRunnerTrajectoryGenerator.generateTrajectories(pathManager);
        // ITrajectory trajectory = pathManager.getTrajectory("w2ba-goToPickUpBall2");

        // try (CsvWriter csvWriter = CsvWriter.builder().build(java.nio.file.Path.of("test.csv"), StandardCharsets.UTF_8))
        // {
        //     csvWriter.writeRow("t", "x", "y", "theta", "vx", "vy", "omega");

        //     for (double t = 0.0; t < trajectory.getDuration() + 0.01; t += 0.02)
        //     {
        //         TrajectoryState state = trajectory.get(t);
        //         csvWriter.writeRow(
        //             Double.toString(t),
        //             Double.toString(state.xPosition),
        //             Double.toString(state.yPosition),
        //             Double.toString(state.angle),
        //             Double.toString(state.xVelocity),
        //             Double.toString(state.yVelocity),
        //             Double.toString(state.angleVelocity));
        //     }

        //     csvWriter.close();
        // }
        // catch (IOException e)
        // {
        // }
    }

    public static void generateTrajectories(TrajectoryManager pathManager)
    {
        // ----------------------------------------- Sample paths ----------------------------------------- //
        addPath(
            pathManager,
            startTrajectory()
                .splineTo(new Vector2d(48, 0), 0),
            "goForward4ft");

        addPath(pathManager, 
            startTrajectory(180.0 * Helpers.DEGREES_TO_RADIANS)
                .splineTo(new Vector2d(-4.0, 0), 180.0 * Helpers.DEGREES_TO_RADIANS), 
            "goBackwards4inch");

        addPath(
            pathManager,
            startTrajectory(90.0 * Helpers.DEGREES_TO_RADIANS)
                .splineTo(new Vector2d(0, 48), 90.0 * Helpers.DEGREES_TO_RADIANS),
            "goLeft4ft");



        addPath(
            pathManager,
            startTrajectory()
                .splineTo(new Vector2d(84, 0), 0),
            "goForward7ft");

        addPath(
            pathManager,
            startTrajectory()
                .splineToSplineHeading(new Pose2d(-1, 0, 180.0 * Helpers.DEGREES_TO_RADIANS), 180.0 * Helpers.DEGREES_TO_RADIANS),
            "turn180Path");

        addPath(
            pathManager,
            startTrajectory(180.0 * Helpers.DEGREES_TO_RADIANS)
                .splineToSplineHeading(new Pose2d(-84, 0, 180.0 * Helpers.DEGREES_TO_RADIANS), 180.0 * Helpers.DEGREES_TO_RADIANS),
            "goBack7ftRotate");

        addPath(
            pathManager,
            startTrajectory(180.0 * Helpers.DEGREES_TO_RADIANS)
                .splineTo(new Vector2d(-72, 0), 180.0 * Helpers.DEGREES_TO_RADIANS),
            "goBack6ft");
        
        //---------------------------------------------- 2023 Paths -----------------------------------------------------

        // 180 heading is towards the grind and 0 is towards the charge station

        //Blue Alliance Values
        final double BlueStartGridX = 70.188;
        final double BlueStartOneGridY = -196.595;
        final double BlueStartTwoCubeGridY = 174.19;
        final double BlueStartThreeGridY = 152.375;
        final double BlueStartFourGridY = 130.375;
        final double BlueStartFiveGridY = 108.19;
        final double BlueStartSixGridY = 86.375;
        final double BlueStartSevenGridY = 64.095;
        final double BlueStartEightGridY = 42.19;
        final double BlueStartNineGridY = 20.095;
        final double BlueCloseChargeStationX = 98.5;
        final double BlueFarChargeStationX = 207.001;
        final double BlueChargeStationY = 108.015;
        final double BlueInBetweenPointAfterChargeStationX = 213.5;
        final double BlueGroundPiecesX = 260.455;
        final double BlueGroundOneY = 180.19;
        final double BlueGroundTwoY = 36.19;
        final double BlueGroundThreeY = 132.19;
        final double BlueGroundFourY = 84.19;

        //TANGENTS:
        // +x = 0
        // -x = 180
        // -y = -90
        // +y = 90
        addPath(
            pathManager,
            startTrajectory(BlueStartGridX,  BlueStartNineGridY,  0 * Helpers.DEGREES_TO_RADIANS, 0 * Helpers.DEGREES_TO_RADIANS)
                
                .lineTo(new Vector2d(BlueFarChargeStationX, BlueStartNineGridY)) // Goes forward
                .splineToConstantHeading(new Vector2d(BlueFarChargeStationX, BlueChargeStationY), 90 * Helpers.DEGREES_TO_RADIANS),

                
                //Jamie's Charge Station task
            "BlueNineStartToGuardInBetweenToFarChargeStation");

    }

    private static TrajectoryBuilder startTrajectory()
    {
        return startTrajectory(0.0, 0.0, 0.0, 0.0);
    }

    private static TrajectoryBuilder startTrajectory(double startTangent)
    {
        return startTrajectory(0.0, 0.0, 0.0, startTangent);
    }

    private static TrajectoryBuilder startTrajectory(double startXPos, double startYPos, double startHeading, double startTangent)
    {
        return new TrajectoryBuilder(new Pose2d(startXPos, startYPos, startHeading), startTangent, RoadRunnerTrajectoryGenerator.velocityConstraint, RoadRunnerTrajectoryGenerator.accelerationConstraint);
    }

    private static void addPath(TrajectoryManager pathManager, TrajectoryBuilder trajectoryBuilder, String name)
    {
        try
        {
            pathManager.addTrajectory(name, trajectoryBuilder);
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