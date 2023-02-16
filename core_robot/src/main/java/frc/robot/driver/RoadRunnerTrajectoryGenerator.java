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

        double TurnaryOperatorForwardsHeadingOrTangent = (TuningConstants.isRed ? 180 : 0);
        double TurnaryOperatorBackwardsHeadingOrTangent = (TuningConstants.isRed ? 0 : 180);

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
        Vector2d P2 = StartTwoGrid;
        Vector2d P3 = StartThreeGrid;
        Vector2d P4 = StartFourGrid;
        Vector2d P5 = StartFiveGrid;
        Vector2d P6 = StartSixGrid;
        Vector2d P7 = StartSevenGrid;
        Vector2d P8 = StartEightGrid;
        Vector2d P9 = StartNineGrid;
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
        

        

        // //Blue Alliance X Values
        // final double BlueStartGridX = 70.188;
        // final double BlueCloseChargeStationX = 122.5; // 24 inches from the ChargeStation exact entry
        // final double BlueFarChargeStationX = 183.001; // -5 inches from the ChargeStation exact entry
        // final double BlueInBetweenPointAfterChargeStationX = 213.5;
        // final double BlueGroundPiecesX = 260.455;
        // //Red Alliance X Values
        // final double RedStartGridX = 578.035;
        // final double RedCloseChargeStation = 576.223; // 24 inches from the ChargeStation exact entry
        // final double RedFarChargeStation = 419.097; // 24 inches from the ChargeStation exact entry
        // final double RedGroundPiecesX = 260.455;


        //TANGENTS:
        // +x = 0 Towards the red alliance
        // -x = 180 Towards the blue alliance
        // -y = -90 Towards the Guardrail
        // +y = 90 Towards Loading Zone

        // //Jamie's Old Path
        // addTrajectory(
        //     trajectoryManager,
        //     startTrajectory(BlueStartGridX,  StartEightGridY,  0 * Helpers.DEGREES_TO_RADIANS, 0 * Helpers.DEGREES_TO_RADIANS)
                
        //         .lineTo(new Vector2d(BlueFarChargeStationX, StartEightGridY)) // Goes forward
        //         .splineToConstantHeading(new Vector2d(BlueFarChargeStationX, ChargeStationY), 90 * Helpers.DEGREES_TO_RADIANS),
                
        //         //Jamie's Charge Station task
        //     "BlueNineStartToGuardInBetweenToFarChargeStation");
        
        //New Paths based on (0,0) being in the center bottom of the field


        //Travels to InBetweenClose point 
        addTrajectory(
            trajectoryManager,
            startTrajectory(P1, TurnaryOperatorBackwardsHeadingOrTangent * Helpers.DEGREES_TO_RADIANS, TurnaryOperatorForwardsHeadingOrTangent * Helpers.DEGREES_TO_RADIANS)
                //Places cone on node

                .lineTo(P10), //Goes to closest april tag visiblity point
                
                //Jamie's Charge Station task
            "BlueOneStartToInBetweenLoad");
        
        addTrajectory(
            trajectoryManager,
            startTrajectory(P2, TurnaryOperatorBackwardsHeadingOrTangent * Helpers.DEGREES_TO_RADIANS, TurnaryOperatorForwardsHeadingOrTangent * Helpers.DEGREES_TO_RADIANS)
                //Places cone on node

                .lineTo(P10), //Goes to closest april tag visiblity point
                
                //Jamie's Charge Station task
            "BlueTwoStartToInBetweenLoad");
        
        addTrajectory(
            trajectoryManager,
            startTrajectory(P3, TurnaryOperatorBackwardsHeadingOrTangent * Helpers.DEGREES_TO_RADIANS, TurnaryOperatorForwardsHeadingOrTangent * Helpers.DEGREES_TO_RADIANS)
                //Places cone on node

                .lineTo(P10), //Goes to closest april tag visiblity point
                
                //Jamie's Charge Station task
            "BlueThreeStartToInBetweenLoad");

        addTrajectory(
            trajectoryManager,
            startTrajectory(P4, TurnaryOperatorBackwardsHeadingOrTangent * Helpers.DEGREES_TO_RADIANS, TurnaryOperatorForwardsHeadingOrTangent * Helpers.DEGREES_TO_RADIANS)
                //Places cone on node

                .lineTo(P11), //Goes to closest april tag visiblity point
                
                //Jamie's Charge Station task
            "BlueFourStartToChargeStationClose");
        
        addTrajectory(
            trajectoryManager,
            startTrajectory(P5,  TurnaryOperatorBackwardsHeadingOrTangent * Helpers.DEGREES_TO_RADIANS, TurnaryOperatorForwardsHeadingOrTangent * Helpers.DEGREES_TO_RADIANS)
                //Places cone on node

                .lineTo(P11), //Goes to closest april tag visiblity point
                
                //Jamie's Charge Station task
            "BlueFiveStartToChargeStationClose");
        
        addTrajectory(
            trajectoryManager,
            startTrajectory(P6,  TurnaryOperatorBackwardsHeadingOrTangent * Helpers.DEGREES_TO_RADIANS, TurnaryOperatorForwardsHeadingOrTangent * Helpers.DEGREES_TO_RADIANS)
                //Places cone on node

                .lineTo(P11), //Goes to closest april tag visiblity point
                
                //Jamie's Charge Station task
            "BlueSixStartToChargeStationClose");
        
        addTrajectory(
            trajectoryManager,
            startTrajectory(P7,  TurnaryOperatorBackwardsHeadingOrTangent * Helpers.DEGREES_TO_RADIANS, TurnaryOperatorForwardsHeadingOrTangent * Helpers.DEGREES_TO_RADIANS)
                //Places cone on node

                .lineTo(P12), //Goes to closest april tag visiblity point
                
                //Jamie's Charge Station task
            "BlueSevenStartToInBetweenGuard");

        addTrajectory(
            trajectoryManager,
            startTrajectory(P8,  TurnaryOperatorBackwardsHeadingOrTangent * Helpers.DEGREES_TO_RADIANS, TurnaryOperatorForwardsHeadingOrTangent * Helpers.DEGREES_TO_RADIANS)
                //Places cone on node

                .lineTo(P12), //Goes to closest april tag visiblity point
                
            //Jamie's Charge Station task
            "BlueEightStartToInBetweenGuard");
        addTrajectory(
                trajectoryManager,
                startTrajectory(P9,  TurnaryOperatorBackwardsHeadingOrTangent * Helpers.DEGREES_TO_RADIANS, TurnaryOperatorForwardsHeadingOrTangent * Helpers.DEGREES_TO_RADIANS)
                    //Places cone on node
    
                    .lineTo(P12), //Goes to closest april tag visiblity point
                    
                    //Jamie's Charge Station task
            "BlueNineStartToInBetweenGuard");


        //Travels to close charge station from starting positions
        //Probably not needed
        addTrajectory(
            trajectoryManager,
            startTrajectory(P12, TurnaryOperatorBackwardsHeadingOrTangent * Helpers.DEGREES_TO_RADIANS, TurnaryOperatorForwardsHeadingOrTangent * Helpers.DEGREES_TO_RADIANS)

                .splineToConstantHeading((P11), 90 * Helpers.DEGREES_TO_RADIANS), // Goes to infront of charge station
                
                //Jamie's Charge Station task
            "InBetweenGuardToChargeStation");
        
        addTrajectory(
            trajectoryManager,
            startTrajectory(P10, TurnaryOperatorBackwardsHeadingOrTangent * Helpers.DEGREES_TO_RADIANS, TurnaryOperatorForwardsHeadingOrTangent * Helpers.DEGREES_TO_RADIANS)

                .splineToConstantHeading(P11, 90 * Helpers.DEGREES_TO_RADIANS), // Goes to infront of charge station
                
                //Jamie's Charge Station task
            "InBetweenLoadToChargeStation");
        
        
        //Travels to Start positions from InBetweenPoints
        addTrajectory(
            trajectoryManager,
            startTrajectory(P10, TurnaryOperatorBackwardsHeadingOrTangent * Helpers.DEGREES_TO_RADIANS, TurnaryOperatorForwardsHeadingOrTangent * Helpers.DEGREES_TO_RADIANS)
                //Places cone on node

                .lineTo(P1), //Goes back to grid
                
                //Jamie's Charge Station task
            "InBetweenLoadToBlueOneStart");
        
        addTrajectory(
            trajectoryManager,
            startTrajectory(P12, TurnaryOperatorBackwardsHeadingOrTangent * Helpers.DEGREES_TO_RADIANS, TurnaryOperatorForwardsHeadingOrTangent * Helpers.DEGREES_TO_RADIANS)
                //Places cone on node

                .lineTo(P2), //Goes back to grid
                
                //Jamie's Charge Station task
            "InBetweenLoadToBlueTwoStart");
        
        addTrajectory(
            trajectoryManager,
            startTrajectory(P10, TurnaryOperatorBackwardsHeadingOrTangent * Helpers.DEGREES_TO_RADIANS, TurnaryOperatorForwardsHeadingOrTangent * Helpers.DEGREES_TO_RADIANS)
                //Places cone on node

                .lineTo(P3), //Goes back to grid
                
                //Jamie's Charge Station task
            "InBetweenLoadToBlueThreeStart");

        addTrajectory(
            trajectoryManager,
            startTrajectory(P13, TurnaryOperatorBackwardsHeadingOrTangent * Helpers.DEGREES_TO_RADIANS, TurnaryOperatorForwardsHeadingOrTangent * Helpers.DEGREES_TO_RADIANS)
                //Places cone on node

                .lineTo(P4), //Goes back to grid
                
                //Jamie's Charge Station task
            "ChargeStationFarToBlueFourStart");
        
        addTrajectory(
            trajectoryManager,
            startTrajectory(P13,  TurnaryOperatorBackwardsHeadingOrTangent * Helpers.DEGREES_TO_RADIANS, TurnaryOperatorForwardsHeadingOrTangent * Helpers.DEGREES_TO_RADIANS)
                //Places cone on node

                .lineTo(P5), //Goes back to grid
                
                //Jamie's Charge Station task
            "ChargeStationFarToBlueFiveStart");
        
        addTrajectory(
            trajectoryManager,
            startTrajectory(P13,  TurnaryOperatorBackwardsHeadingOrTangent * Helpers.DEGREES_TO_RADIANS, TurnaryOperatorForwardsHeadingOrTangent * Helpers.DEGREES_TO_RADIANS)
                //Places cone on node

                .lineTo(P6), //Goes back to grid
                
                //Jamie's Charge Station task
            "ChargeStationFarToBlueSixStart");
        
        addTrajectory(
            trajectoryManager,
            startTrajectory(P12,  TurnaryOperatorBackwardsHeadingOrTangent * Helpers.DEGREES_TO_RADIANS, TurnaryOperatorForwardsHeadingOrTangent * Helpers.DEGREES_TO_RADIANS)
                //Places cone on node

                .lineTo(P7), //Goes back to grid
                
                //Jamie's Charge Station task
            "InBetweenGuardToBlueSevenStart");

        addTrajectory(
            trajectoryManager,
            startTrajectory(P12,  TurnaryOperatorBackwardsHeadingOrTangent * Helpers.DEGREES_TO_RADIANS, TurnaryOperatorForwardsHeadingOrTangent * Helpers.DEGREES_TO_RADIANS)
                //Places cone on node

                .lineTo(P8), //Goes back to grid
                
            //Jamie's Charge Station task
            "InBetweenGuardToBlueEightStart");
        addTrajectory(
            trajectoryManager,
            startTrajectory(P12,  TurnaryOperatorBackwardsHeadingOrTangent * Helpers.DEGREES_TO_RADIANS, TurnaryOperatorForwardsHeadingOrTangent * Helpers.DEGREES_TO_RADIANS)
                //Places cone on node

                .lineTo(P9), //Goes back to grid
                
                //Jamie's Charge Station task
            "InBetweenGuardToBlueNineStart");
        
        //InBetweenPointClose to InBetweenPointFar
        addTrajectory(
            trajectoryManager,
            startTrajectory(P10,  TurnaryOperatorBackwardsHeadingOrTangent * Helpers.DEGREES_TO_RADIANS, TurnaryOperatorForwardsHeadingOrTangent * Helpers.DEGREES_TO_RADIANS)

                .splineToConstantHeading(P18, TurnaryOperatorBackwardsHeadingOrTangent * Helpers.DEGREES_TO_RADIANS), //Goes to InBetweenPointLoadFar
                
                //Jamie's Charge Station task
            "InBetweenPointCloseToInBetweenPointFar");
        
        addTrajectory(
            trajectoryManager,
            startTrajectory(P11,  TurnaryOperatorBackwardsHeadingOrTangent * Helpers.DEGREES_TO_RADIANS, TurnaryOperatorForwardsHeadingOrTangent * Helpers.DEGREES_TO_RADIANS)
            
                .splineToConstantHeading(P13, TurnaryOperatorForwardsHeadingOrTangent * Helpers.DEGREES_TO_RADIANS), //Goes to ChargeStationFar over the chargestation
                
                //Jamie's Charge Station task
            "ChargeStationCloseToChargeStationFar");
        
        addTrajectory(
            trajectoryManager,
            startTrajectory(P12,  TurnaryOperatorBackwardsHeadingOrTangent * Helpers.DEGREES_TO_RADIANS, TurnaryOperatorForwardsHeadingOrTangent * Helpers.DEGREES_TO_RADIANS)
            
                .splineToConstantHeading(P19, TurnaryOperatorForwardsHeadingOrTangent * Helpers.DEGREES_TO_RADIANS), //Goes to ChargeStationFar over the chargestation
                
                //Jamie's Charge Station task
            "InBetweenGuardCloseToInBetweenGuardFar");

        //InBetweenPointFar to InBetweenPointClose
        addTrajectory(
            trajectoryManager,
            startTrajectory(P18,  TurnaryOperatorBackwardsHeadingOrTangent * Helpers.DEGREES_TO_RADIANS, TurnaryOperatorBackwardsHeadingOrTangent * Helpers.DEGREES_TO_RADIANS)

                .splineToConstantHeading(P10, TurnaryOperatorBackwardsHeadingOrTangent * Helpers.DEGREES_TO_RADIANS), //Goes to InBetweenPointLoadFar
                
                //Jamie's Charge Station task
            "InBetweenPointCloseToInBetweenPointFar");
        
        addTrajectory(
            trajectoryManager,
            startTrajectory(P13,  TurnaryOperatorBackwardsHeadingOrTangent * Helpers.DEGREES_TO_RADIANS, TurnaryOperatorBackwardsHeadingOrTangent * Helpers.DEGREES_TO_RADIANS)
            
                .splineToConstantHeading(P11, TurnaryOperatorBackwardsHeadingOrTangent * Helpers.DEGREES_TO_RADIANS), //Goes to ChargeStationFar over the chargestation
                
                //Jamie's Charge Station task
            "ChargeStationCloseToChargeStationFar");
        
        addTrajectory(
            trajectoryManager,
            startTrajectory(P19,  TurnaryOperatorBackwardsHeadingOrTangent * Helpers.DEGREES_TO_RADIANS, TurnaryOperatorBackwardsHeadingOrTangent * Helpers.DEGREES_TO_RADIANS)
            
                .splineToConstantHeading(P12, TurnaryOperatorBackwardsHeadingOrTangent * Helpers.DEGREES_TO_RADIANS), //Goes to ChargeStationFar over the chargestation
                
                //Jamie's Charge Station task
            "InBetweenGuardCloseToInBetweenGuardFar");

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