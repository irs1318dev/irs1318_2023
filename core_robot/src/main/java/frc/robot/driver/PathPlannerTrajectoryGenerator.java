package frc.robot.driver;

import frc.robot.TuningConstants;
import frc.robot.common.robotprovider.IPathPlanner;
import frc.robot.common.robotprovider.ITrajectory;
import frc.robot.common.robotprovider.PathPlannerWaypoint;
import frc.robot.common.robotprovider.Point2d;
import frc.robot.driver.common.TrajectoryManager;

public class PathPlannerTrajectoryGenerator
{   
    

    public static void generateTrajectories(TrajectoryManager trajectoryManager, IPathPlanner pathPlanner)
    {

        double ForwardOT = (TuningConstants.isRed ? 180 : 0); //TurnaryOperatorForwardsHeadingOrTangent
        double BackwardOT = (TuningConstants.isRed ? 0 : 180); //TurnaryOperatorBackwardsHeadingOrTangent

        //Vectors
        Point2d StartOneGrid = new Point2d(TuningConstants.isRed ? TuningConstants.StartGridX : -TuningConstants.StartGridX, TuningConstants.StartOneGridY); //1
        Point2d StartTwoGrid = new Point2d(TuningConstants.isRed ? TuningConstants.StartGridX : -TuningConstants.StartGridX, TuningConstants.StartTwoGridY); //2
        Point2d StartThreeGrid = new Point2d(TuningConstants.isRed ? TuningConstants.StartGridX : -TuningConstants.StartGridX, TuningConstants.StartThreeGridY); //3
        Point2d StartFourGrid = new Point2d(TuningConstants.isRed ? TuningConstants.StartGridX : -TuningConstants.StartGridX, TuningConstants.StartFourGridY); //4
        Point2d StartFiveGrid = new Point2d(TuningConstants.isRed ? TuningConstants.StartGridX : -TuningConstants.StartGridX, TuningConstants.StartFiveGridY); //5
        Point2d StartSixGrid = new Point2d(TuningConstants.isRed ? TuningConstants.StartGridX : -TuningConstants.StartGridX, TuningConstants.StartSixGridY); //6
        Point2d StartSevenGrid = new Point2d(TuningConstants.isRed ? TuningConstants.StartGridX : -TuningConstants.StartGridX, TuningConstants.StartSevenGridY); //7
        Point2d StartEightGrid = new Point2d(TuningConstants.isRed ? TuningConstants.StartGridX : -TuningConstants.StartGridX, TuningConstants.StartEightGridY); //8
        Point2d StartNineGrid = new Point2d(TuningConstants.isRed ? TuningConstants.StartGridX : -TuningConstants.StartGridX, TuningConstants.StartNineGridY); //9
        Point2d InBetweenLoadClose = new Point2d(TuningConstants.isRed ? TuningConstants.CloseChargeStationX : -TuningConstants.CloseChargeStationX,TuningConstants.GroundOneY); //10
        Point2d ChargeStationClose = new Point2d(TuningConstants.isRed ? TuningConstants.CloseChargeStationX : -TuningConstants.CloseChargeStationX, TuningConstants.ChargeStationY); //11
        Point2d InBetweenGuardClose = new Point2d(TuningConstants.isRed ? TuningConstants.CloseChargeStationX : -TuningConstants.CloseChargeStationX, TuningConstants.GroundFourY); //12
        Point2d ChargeStationFar = new Point2d(TuningConstants.isRed ? TuningConstants.FarChargeStationX : -TuningConstants.FarChargeStationX, TuningConstants.ChargeStationY); //13
        Point2d GroundOne = new Point2d(TuningConstants.isRed ? TuningConstants.GroundPiecesX : -TuningConstants.GroundPiecesX, TuningConstants.GroundOneY); //14
        Point2d GroundTwo = new Point2d(TuningConstants.isRed ? TuningConstants.GroundPiecesX : -TuningConstants.GroundPiecesX, TuningConstants.GroundTwoY); //15
        Point2d GroundThree = new Point2d(TuningConstants.isRed ? TuningConstants.GroundPiecesX : -TuningConstants.GroundPiecesX, TuningConstants.GroundThreeY); //16
        Point2d GroundFour = new Point2d(TuningConstants.isRed ? TuningConstants.GroundPiecesX : -TuningConstants.GroundPiecesX, TuningConstants.GroundFourY); //17
        Point2d InBetweenLoadFar = new Point2d(TuningConstants.isRed ? TuningConstants.FarChargeStationX : -TuningConstants.FarChargeStationX, TuningConstants.GroundOneY); //18
        Point2d InBetweenGuardFar = new Point2d(TuningConstants.isRed ? TuningConstants.FarChargeStationX : -TuningConstants.FarChargeStationX, TuningConstants.GroundFourY); //19

        Point2d P1 = StartOneGrid;
        Point2d LoadEdge = new Point2d(TuningConstants.isRed ? TuningConstants.StartGridX : -TuningConstants.StartGridX  , TuningConstants.StartOneGridY + 1.05); //edge of grid
        Point2d P2 = StartTwoGrid;
        Point2d P3 = StartThreeGrid;
        Point2d P4 = StartFourGrid;
        Point2d P5 = StartFiveGrid;
        Point2d P6 = StartSixGrid;
        Point2d P7 = StartSevenGrid;
        Point2d P8 = StartEightGrid;
        Point2d P9 = StartNineGrid;
        Point2d GuardEdge = new Point2d(TuningConstants.isRed ? TuningConstants.StartGridX : -TuningConstants.StartGridX, 16.8); //edge of grid
        Point2d P10 = InBetweenLoadClose;
        Point2d P11 = ChargeStationClose;
        Point2d P12 = InBetweenGuardClose;
        Point2d P13 = ChargeStationFar;
        Point2d P14 = GroundOne;
        Point2d P15 = GroundTwo;
        Point2d P16 = GroundThree;
        Point2d P17 = GroundFour;
        Point2d P18 = InBetweenLoadFar;
        Point2d P19 = InBetweenGuardFar;
        Point2d P20 = new Point2d(TuningConstants.isRed ? TuningConstants.FarChargeStationX - 36 : -TuningConstants.FarChargeStationX + 36, TuningConstants.GroundOneY);
        Point2d P21 = new Point2d(TuningConstants.isRed ? TuningConstants.FarChargeStationX - 36 : -TuningConstants.FarChargeStationX + 36, TuningConstants.ChargeStationY);
        Point2d P22 = new Point2d(TuningConstants.isRed ? TuningConstants.FarChargeStationX - 36 : -TuningConstants.FarChargeStationX + 36, TuningConstants.GroundFourY);

        //TANGENTS AND ORIENTATION:
        // +x = 0 Towards the red alliance
        // -x = 180 Towards the blue alliance
        // -y = -90 Towards the Guardrail
        // +y = 90 Towards Loading Zone

        //Sample Paths
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                new PathPlannerWaypoint(0.0, 0.0),
                new PathPlannerWaypoint(48.0, 0.0)),
            "goForward4ft2");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                new PathPlannerWaypoint(0.0, 0.0, 180.0),
                new PathPlannerWaypoint(-4.0, 0.0, 180.0)),
                "goBackwards4inch2");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                new PathPlannerWaypoint(0.0, 0.0, 90.0),
                new PathPlannerWaypoint(0.0, 48.0, 90.0)),
            "goLeft4ft2");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                new PathPlannerWaypoint(0.0, 0.0, 0.0, PathPlannerWaypoint.setOrientation(1)),
                new PathPlannerWaypoint(36.0, 0.0, 180.0, PathPlannerWaypoint.setOrientation(-1))),
            "goForward6ft2AndRotate");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                new PathPlannerWaypoint(0.0, 0.0, 0.0, 0.0),
                new PathPlannerWaypoint(-1.0, 0.0, 180.0, 180.0)),
            "turn180Path2");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                new PathPlannerWaypoint(0.0, 0.0, 180.0, 0.0),
                new PathPlannerWaypoint(-84.0, 0.0, 180.0, 180.0)),
            "goBack7ftRotate2");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                new PathPlannerWaypoint(0.0, 0.0, 0.0, 0.0),
                new PathPlannerWaypoint(48.0, 0.0, 90.0, 0.0),
                new PathPlannerWaypoint(48.0, 48.0, 180.0, 0.0),
                new PathPlannerWaypoint(0.0, 48.0, -90.0, 0.0),
                new PathPlannerWaypoint(0.0, 0.0, 0.0, 0.0)),
            "pranavTest");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                new PathPlannerWaypoint(0.0, 0.0, 180.0, 0.0),
                new PathPlannerWaypoint(-1.0, 0.0, 180.0, 0.0)),
            "goBack6ft2");

        

        //2023 Paths

        //Test Path
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                new PathPlannerWaypoint(LoadEdge, ForwardOT, ForwardOT),
                new PathPlannerWaypoint(P18, ForwardOT, ForwardOT)),
            "LoadEdgeto18");

        //Actual Paths
        
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                new PathPlannerWaypoint(GuardEdge, ForwardOT, BackwardOT),
                new PathPlannerWaypoint(P9, ForwardOT, BackwardOT),
                new PathPlannerWaypoint(P12, ForwardOT, BackwardOT),
                new PathPlannerWaypoint(P19, ForwardOT, BackwardOT),
                new PathPlannerWaypoint(P22, ForwardOT, ForwardOT),
                new PathPlannerWaypoint(P17, ForwardOT, ForwardOT)),
            "GuardEdgeto17");
        
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                new PathPlannerWaypoint(P17, BackwardOT, ForwardOT),
                new PathPlannerWaypoint(P22, BackwardOT, ForwardOT),
                new PathPlannerWaypoint(P19, BackwardOT, BackwardOT),
                new PathPlannerWaypoint(P12, BackwardOT, BackwardOT)),
            "17to12");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                new PathPlannerWaypoint(P10, BackwardOT, BackwardOT),
                new PathPlannerWaypoint(P1, BackwardOT, BackwardOT)),
            "10to1");
        
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                new PathPlannerWaypoint(P12, BackwardOT, BackwardOT),
                new PathPlannerWaypoint(P8, BackwardOT, BackwardOT)),
            "12to8");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                new PathPlannerWaypoint(GuardEdge, 90, BackwardOT),
                new PathPlannerWaypoint(P9, 45, BackwardOT),
                new PathPlannerWaypoint(P12, ForwardOT, BackwardOT),
                new PathPlannerWaypoint(P19, 90, BackwardOT),
                new PathPlannerWaypoint(P13, 90, BackwardOT)
                ),
            "GuardEdgetoChargeStationFar");

        // TESTED AND WORKING
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                new PathPlannerWaypoint(LoadEdge, BackwardOT, BackwardOT),
                new PathPlannerWaypoint(P1, BackwardOT, BackwardOT),
                new PathPlannerWaypoint(P10, BackwardOT, BackwardOT),
                new PathPlannerWaypoint(P18, BackwardOT, BackwardOT),
                new PathPlannerWaypoint(P13, BackwardOT, BackwardOT)),
            "LoadEdgetoChargeStationFar");
        
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                new PathPlannerWaypoint(LoadEdge, -90, BackwardOT),
                new PathPlannerWaypoint(P1, -90, BackwardOT)),
            "LoadEdgeto1");
        
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                new PathPlannerWaypoint(P1, -45, BackwardOT),
                new PathPlannerWaypoint(P10, -45, BackwardOT),
                new PathPlannerWaypoint(P18, ForwardOT, BackwardOT),
                new PathPlannerWaypoint(P20, ForwardOT, ForwardOT),
                new PathPlannerWaypoint(P14, ForwardOT, ForwardOT)),
            "1to14");
        
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                new PathPlannerWaypoint(P14, BackwardOT, ForwardOT),
                new PathPlannerWaypoint(P20, BackwardOT, ForwardOT),
                new PathPlannerWaypoint(P18, BackwardOT, BackwardOT),
                new PathPlannerWaypoint(P10, BackwardOT, BackwardOT)),
            "14to10");
        
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                new PathPlannerWaypoint(P10, -160, BackwardOT),
                new PathPlannerWaypoint(P2, BackwardOT, BackwardOT)),
            "10to2");
        
        

        
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