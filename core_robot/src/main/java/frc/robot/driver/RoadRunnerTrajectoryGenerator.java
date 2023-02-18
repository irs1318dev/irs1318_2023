package frc.robot.driver;

import java.util.Arrays;

import com.acmerobotics.roadrunner.geometry.*;
import com.acmerobotics.roadrunner.trajectory.*;
import com.acmerobotics.roadrunner.trajectory.constraints.*;

import frc.robot.HardwareConstants;
import frc.robot.TuningConstants;
import frc.robot.common.*;
import frc.robot.driver.common.TrajectoryManager;

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
        trajectoryManager.buildAll();
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

        double ForwardHT = (TuningConstants.isRed ? 180 : 0); //TurnaryOperatorForwardsHeadingOrTangent
        double BackwardHT = (TuningConstants.isRed ? 0 : 180); //TurnaryOperatorBackwardsHeadingOrTangent

        //Vectors
        Vector2d StartOneGrid = new Vector2d(TuningConstants.isRed ? TuningConstants.StartGridX : -TuningConstants.StartGridX  , TuningConstants.StartOneGridY); //1
        Vector2d StartTwoGrid = new Vector2d(TuningConstants.isRed ? TuningConstants.StartGridX : -TuningConstants.StartGridX, TuningConstants.StartTwoGridY); //2
        Vector2d StartThreeGrid = new Vector2d(TuningConstants.isRed ? TuningConstants.StartGridX : -TuningConstants.StartGridX, TuningConstants.StartThreeGridY); //3
        Vector2d StartFourGrid = new Vector2d(TuningConstants.isRed ? TuningConstants.StartGridX : -TuningConstants.StartGridX, TuningConstants.StartFourGridY); //4
        Vector2d StartFiveGrid = new Vector2d(TuningConstants.isRed ? TuningConstants.StartGridX : -TuningConstants.StartGridX, TuningConstants.StartFiveGridY); //5
        Vector2d StartSixGrid = new Vector2d(TuningConstants.isRed ? TuningConstants.StartGridX : -TuningConstants.StartGridX, TuningConstants.StartSixGridY); //6
        Vector2d StartSevenGrid = new Vector2d(TuningConstants.isRed ? TuningConstants.StartGridX : -TuningConstants.StartGridX, TuningConstants.StartSevenGridY); //7
        Vector2d StartEightGrid = new Vector2d(TuningConstants.isRed ? TuningConstants.StartGridX : -TuningConstants.StartGridX, TuningConstants.StartEightGridY); //8
        Vector2d StartNineGrid = new Vector2d(TuningConstants.isRed ? TuningConstants.StartGridX : -TuningConstants.StartGridX, TuningConstants.StartNineGridY); //9
        Vector2d InBetweenLoadClose = new Vector2d(TuningConstants.isRed ? TuningConstants.CloseChargeStationX : -TuningConstants.CloseChargeStationX,TuningConstants.GroundOneY); //10
        Vector2d ChargeStationClose = new Vector2d(TuningConstants.isRed ? TuningConstants.CloseChargeStationX : -TuningConstants.CloseChargeStationX, TuningConstants.ChargeStationY); //11
        Vector2d InBetweenGuardClose = new Vector2d(TuningConstants.isRed ? TuningConstants.CloseChargeStationX : -TuningConstants.CloseChargeStationX, TuningConstants.GroundFourY); //12
        Vector2d ChargeStationFar = new Vector2d(TuningConstants.FarChargeStationX, TuningConstants.ChargeStationY); //13
        Vector2d GroundOne = new Vector2d(TuningConstants.isRed ? TuningConstants.GroundPiecesX : -TuningConstants.GroundPiecesX, TuningConstants.GroundOneY); //14
        Vector2d GroundTwo = new Vector2d(TuningConstants.isRed ? TuningConstants.GroundPiecesX : -TuningConstants.GroundPiecesX, TuningConstants.GroundTwoY); //15
        Vector2d GroundThree = new Vector2d(TuningConstants.isRed ? TuningConstants.GroundPiecesX : -TuningConstants.GroundPiecesX, TuningConstants.GroundThreeY); //16
        Vector2d GroundFour = new Vector2d(TuningConstants.isRed ? TuningConstants.GroundPiecesX : -TuningConstants.GroundPiecesX, TuningConstants.GroundFourY); //17
        Vector2d InBetweenLoadFar = new Vector2d(TuningConstants.isRed ? TuningConstants.FarChargeStationX : -TuningConstants.FarChargeStationX, TuningConstants.GroundOneY); //18
        Vector2d InBetweenGuardFar = new Vector2d(TuningConstants.isRed ? TuningConstants.FarChargeStationX : -TuningConstants.FarChargeStationX, TuningConstants.GroundOneY); //19

        Vector2d P1 = StartOneGrid;
        Vector2d StartP1 = new Vector2d(TuningConstants.isRed ? TuningConstants.StartGridX : -TuningConstants.StartGridX  , TuningConstants.StartOneGridY + 1.05);
        Vector2d P2 = StartTwoGrid;
        Vector2d P3 = StartThreeGrid;
        Vector2d P4 = StartFourGrid;
        Vector2d P5 = StartFiveGrid;
        Vector2d P6 = StartSixGrid;
        Vector2d P7 = StartSevenGrid;
        Vector2d P8 = StartEightGrid;
        Vector2d P9 = StartNineGrid;
        Vector2d StartP9 = new Vector2d(TuningConstants.isRed ? TuningConstants.StartGridX : -TuningConstants.StartGridX, 18.8); //9
        Vector2d P10 = InBetweenLoadClose;
        Vector2d P11 = ChargeStationClose;
        Vector2d P12 = InBetweenGuardClose;
        Vector2d P13 = ChargeStationFar;
        Vector2d P14 = GroundOne;
        Vector2d P15 = GroundTwo;
        Vector2d P16 = GroundThree;
        Vector2d P17 = GroundFour;
        Vector2d P18 = InBetweenLoadFar;
        Vector2d P19 = InBetweenGuardFar;

        //TANGENTS:
        // +x = 0 Towards the red alliance
        // -x = 180 Towards the blue alliance
        // -y = -90 Towards the Guardrail
        // +y = 90 Towards Loading Zone

        //Travels to the ground pieces
        
        //addTrajectory(
        //    trajectoryManager,
        //    startTrajectory(StartP1, BackwardHT * Helpers.DEGREES_TO_RADIANS, -90 * Helpers.DEGREES_TO_RADIANS)
        //        .lineTo(P1),
        //    "bluePosOnePlusOnePartOne");
        //
        //addTrajectory(
        //    trajectoryManager,
        //    startTrajectory(P1, BackwardHT * Helpers.DEGREES_TO_RADIANS, ForwardHT * Helpers.DEGREES_TO_RADIANS)
        //        //Places cone on node
        //
        //        .lineTo(P10) //Goes to closest april tag visiblity point
        //        .splineToSplineHeading(new Pose2d(P18, ForwardHT * Helpers.DEGREES_TO_RADIANS), ForwardHT * Helpers.DEGREES_TO_RADIANS)
        //        .lineTo(P14), //Goes to pick-up first field element
        //
        //    "bluePosOnePlusOnePartTwo");
        //
        //addTrajectory(
        //    trajectoryManager,
        //    startTrajectory(P14, ForwardHT * Helpers.DEGREES_TO_RADIANS, BackwardHT * Helpers.DEGREES_TO_RADIANS)
        //
        //        .splineToSplineHeading(new Pose2d(P10, BackwardHT * Helpers.DEGREES_TO_RADIANS), -0.441)
        //        .lineTo(P2), //Goes to Second Point on Grid
        //         
        //    "bluePosOnePlusOnePartThree");
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

    //Ayush and Jamie constructor, takes Vector instead of X,Y doubles
    private static TrajectoryBuilder startTrajectory(Vector2d startVector, double startHeading, double startTangent)
    {
        return new TrajectoryBuilder(new Pose2d(startVector, startHeading), startTangent, RoadRunnerTrajectoryGenerator.velocityConstraint, RoadRunnerTrajectoryGenerator.accelerationConstraint);
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