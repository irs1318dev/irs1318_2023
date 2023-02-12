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
        TrajectoryManager trajectoryManager = new TrajectoryManager();
        RoadRunnerTrajectoryGenerator.generateTrajectories(trajectoryManager);
        // ITrajectory trajectory = trajectoryManager.getTrajectory("w2ba-goToPickUpBall2");

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

    public static void generateTrajectories(TrajectoryManager trajectoryManager)
    {
        // ----------------------------------------- Sample paths ----------------------------------------- //
        addTrajectory(
            trajectoryManager,
            startTrajectory()
                .splineTo(new Vector2d(48, 0), 0),
            "goForward4ft");

        addTrajectory(trajectoryManager, 
            startTrajectory(180.0 * Helpers.DEGREES_TO_RADIANS)
                .splineTo(new Vector2d(-4.0, 0), 180.0 * Helpers.DEGREES_TO_RADIANS), 
            "goBackwards4inch");

        addTrajectory(
            trajectoryManager,
            startTrajectory(90.0 * Helpers.DEGREES_TO_RADIANS)
                .splineTo(new Vector2d(0, 48), 90.0 * Helpers.DEGREES_TO_RADIANS),
            "goLeft4ft");



        addTrajectory(
            trajectoryManager,
            startTrajectory()
                .splineTo(new Vector2d(84, 0), 0),
            "goForward7ft");

        addTrajectory(
            trajectoryManager,
            startTrajectory()
                .splineToSplineHeading(new Pose2d(-1, 0, 180.0 * Helpers.DEGREES_TO_RADIANS), 180.0 * Helpers.DEGREES_TO_RADIANS),
            "turn180Path");

        addTrajectory(
            trajectoryManager,
            startTrajectory(180.0 * Helpers.DEGREES_TO_RADIANS)
                .splineToSplineHeading(new Pose2d(-84, 0, 180.0 * Helpers.DEGREES_TO_RADIANS), 180.0 * Helpers.DEGREES_TO_RADIANS),
            "goBack7ftRotate");

        addTrajectory(
            trajectoryManager,
            startTrajectory(180.0 * Helpers.DEGREES_TO_RADIANS)
                .splineTo(new Vector2d(-72, 0), 180.0 * Helpers.DEGREES_TO_RADIANS),
            "goBack6ft");
        
        //---------------------------------------------- 2023 Paths ------------------------------------------------------//

        //Y Values
        final double StartOneGridY = 196.595;
        final double StartTwoCubeGridY = 174.19; // April Tag ID's 6 and 3
        final double StartThreeGridY = 152.375;
        final double StartFourGridY = 130.375;
        final double StartFiveGridY = 108.19; // April Tag ID's 7 and 2
        final double StartSixGridY = 86.375;
        final double StartSevenGridY = 64.095;
        final double StartEightGridY = 42.19; // April Tag ID's 8 and 1
        final double StartNineGridY = 20.095;
        final double ChargeStationY = 108.015;
        final double GroundOneY = 180.19;
        final double GroundTwoY = 36.19;
        final double GroundThreeY = 132.19;
        final double GroundFourY = 84.19;
        //X Values
        final double StartGridX = 253.861;
        final double CloseChargeStationX = 201.549;
        final double FarChargeStation = 141.048;
        final double InBetweenPointAfterChargeStationX = 110.549;
        final double GroundPiecesX = 63.594;

        //Changed X Values
        double c_StartGridX;
        double c_CloseChargeStationX;
        double c_FarChargeStation;
        double c_InBetweenPointAfterChargeStationX;
        double c_GroundPiecesX;
        boolean isRed = false;

        double TurnaryOperator = (isRed ? 1 : - 1);
        c_StartGridX = StartGridX * TurnaryOperator;
        c_CloseChargeStationX = CloseChargeStationX * TurnaryOperator;
        c_FarChargeStation = FarChargeStation * TurnaryOperator;
        c_InBetweenPointAfterChargeStationX = InBetweenPointAfterChargeStationX * TurnaryOperator;
        c_GroundPiecesX = GroundPiecesX * TurnaryOperator;


        //Blue Alliance X Values
        final double BlueStartGridX = 70.188;
        final double BlueCloseChargeStationX = 122.5; // 24 inches from the ChargeStation exact entry
        final double BlueFarChargeStationX = 183.001; // -5 inches from the ChargeStation exact entry
        final double BlueInBetweenPointAfterChargeStationX = 213.5;
        final double BlueGroundPiecesX = 260.455;
        //Red Alliance X Values
        final double RedStartGridX = 578.035;
        final double RedCloseChargeStation = 576.223; // 24 inches from the ChargeStation exact entry
        final double RedFarChargeStation = 419.097; // 24 inches from the ChargeStation exact entry
        final double RedGroundPiecesX = 260.455;
        //TANGENTS:
        // +x = 0 Towards the red alliance
        // -x = 180 Towards the blue alliance
        // -y = -90 Towards the Guardrail
        // +y = 90 Towards Loading Zone
        addTrajectory(
            trajectoryManager,
            startTrajectory(BlueStartGridX,  StartEightGridY,  0 * Helpers.DEGREES_TO_RADIANS, 0 * Helpers.DEGREES_TO_RADIANS)
                
                .lineTo(new Vector2d(BlueFarChargeStationX, StartEightGridY)) // Goes forward
                .splineToConstantHeading(new Vector2d(BlueFarChargeStationX, ChargeStationY), 90 * Helpers.DEGREES_TO_RADIANS),
                
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

    private static void addTrajectory(TrajectoryManager trajectoryManager, TrajectoryBuilder trajectoryBuilder, String name)
    {
        try
        {
            trajectoryManager.addTrajectory(name, trajectoryBuilder);
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