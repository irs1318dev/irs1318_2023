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
        PathPlannerTrajectoryGenerator.generateTrajectories(false, trajectoryManager, pathPlanner);
        //PathPlannerTrajectoryGenerator.generateTrajectories(true, trajectoryManager, pathPlanner);

        // Macro paths:
        // addTrajectory(
        //     trajectoryManager,
        //     pathPlanner.buildTrajectory(
        //         TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
        //         TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
        //         new PathPlannerWaypoint(0.0, 0.0, 180.0, 0.0),
        //         new PathPlannerWaypoint(-6.0, 0.0, 180.0, 0.0)),
        //         "goBackwards6inches");

        // // Sample/Testing Paths
        // addTrajectory(
        //     trajectoryManager,
        //     pathPlanner.buildTrajectory(
        //         TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
        //         TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
        //         new PathPlannerWaypoint(0.0, 0.0),
        //         new PathPlannerWaypoint(48.0, 0.0)),
        //     "goForward4ft");

        // addTrajectory(
        //     trajectoryManager,
        //     pathPlanner.buildTrajectory(
        //         TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
        //         TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
        //         new PathPlannerWaypoint(0.0, 0.0, 90.0),
        //         new PathPlannerWaypoint(0.0, 48.0, 90.0)),
        //     "goLeft4ft");

        // addTrajectory(
        //     trajectoryManager,
        //     pathPlanner.buildTrajectory(
        //         TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
        //         TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
        //         new PathPlannerWaypoint(0.0, 0.0, 0.0, PathPlannerWaypoint.setOrientation(true, true)),
        //         new PathPlannerWaypoint(36.0, 0.0, 180.0, PathPlannerWaypoint.setOrientation(true, false))),
        //     "goForward6ftAndRotate");

        // addTrajectory(
        //     trajectoryManager,
        //     pathPlanner.buildTrajectory(
        //         TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
        //         TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
        //         new PathPlannerWaypoint(0.0, 0.0, 0.0, 0.0),
        //         new PathPlannerWaypoint(-1.0, 0.0, 180.0, 180.0)),
        //     "turn180Path");

        // addTrajectory(
        //     trajectoryManager,
        //     pathPlanner.buildTrajectory(
        //         TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
        //         TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
        //         new PathPlannerWaypoint(0.0, 0.0, 180.0, 0.0),
        //         new PathPlannerWaypoint(-84.0, 0.0, 180.0, 180.0)),
        //     "goBack7ftRotate");

        // addTrajectory(
        //     trajectoryManager,
        //     pathPlanner.buildTrajectory(
        //         TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
        //         TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
        //         new PathPlannerWaypoint(0.0, 0.0, 180.0, 0.0),
        //         new PathPlannerWaypoint(-1.0, 0.0, 180.0, 0.0)),
        //     "goBack6ft");
    }

    public static void generateTrajectories(boolean isRed, TrajectoryManager trajectoryManager, IPathPlanner pathPlanner)
    {
        // Lower Y of 12, 19, 22
        // Test Everything that uses loadedge again
        // Make X 18, 13, and 19 closer to 0

        // TANGENTS AND ORIENTATION:
        // +x = 0 Towards the red alliance
        // -x = 180 Towards the blue alliance
        // -y = -90 Towards the Guardrail
        // +y = 90 Towards Loading Zone
        double ForwardOT = (isRed ? 180 : 0); //Turnary Operator For Forwards Orientation Or Forwards Tangent
        double BackwardOT = (isRed ? 0 : 180); //Turnary Operator For Backwards Orientation Or Backwards Tangent
        double Blue45_Red135OT = (isRed ? 135 : 45); //Turnary Operator For 45 Degrees On Red Or 135 Degrees On Blue, in terms of Orientation or Tangent
        double Blue135_Red45OT = (isRed ? 45 : 135); //Turnary Operator For 135 Degrees On Red Or 45 Degrees On Blue, in terms of Orientation or Tangent

        //Vectors
        Point2d P1 = new Point2d(isRed ? TuningConstants.StartGridX : -TuningConstants.StartGridX, TuningConstants.StartOneGridY);; //StartOneGrid
        Point2d LoadEdge = new Point2d(isRed ? TuningConstants.StartGridX : -TuningConstants.StartGridX  , TuningConstants.LoadEdgeY); //Load side edge of grid
        Point2d P2 = new Point2d(isRed ? TuningConstants.StartGridX : -TuningConstants.StartGridX, TuningConstants.StartTwoGridY); //StartTwoGrid
        Point2d P3 = new Point2d(isRed ? TuningConstants.StartGridX : -TuningConstants.StartGridX, TuningConstants.StartThreeGridY); //StartThreeGrid
        Point2d P4 = new Point2d(isRed ? TuningConstants.StartGridX : -TuningConstants.StartGridX, TuningConstants.StartFourGridY); //StartFourGrid
        Point2d P5 = new Point2d(isRed ? TuningConstants.StartGridX : -TuningConstants.StartGridX, TuningConstants.StartFiveGridY); //StartFiveGrid
        Point2d P6 = new Point2d(isRed ? TuningConstants.StartGridX : -TuningConstants.StartGridX, TuningConstants.StartSixGridY); //StartSixGrid
        Point2d P7 = new Point2d(isRed ? TuningConstants.StartGridX : -TuningConstants.StartGridX, TuningConstants.StartSevenGridY); //StartSevenGrid
        Point2d P8 = new Point2d(isRed ? TuningConstants.StartGridX : -TuningConstants.StartGridX, TuningConstants.StartEightGridY); //StartEightGrid
        Point2d P9 = new Point2d(isRed ? TuningConstants.StartGridX : -TuningConstants.StartGridX, TuningConstants.StartNineGridY); //StartNineGrid
        Point2d GuardEdge = new Point2d(isRed ? TuningConstants.StartGridX : -TuningConstants.StartGridX, TuningConstants.GuardEdgeY); //Guard side edge of grid
        Point2d P10 = new Point2d(isRed ? TuningConstants.CloseChargeStationX : -TuningConstants.CloseChargeStationX,TuningConstants.GroundOneY); //InBetweenLoadClose
        Point2d P11 = new Point2d(isRed ? TuningConstants.CloseChargeStationX : -TuningConstants.CloseChargeStationX, TuningConstants.ChargeStationY); //ChargeStationClose
        Point2d P12 = new Point2d(isRed ? TuningConstants.CloseChargeStationX : -TuningConstants.CloseChargeStationX, TuningConstants.GroundFourY); //InBetweenGuardClose
        Point2d P13 = new Point2d(isRed ? TuningConstants.FarChargeStationX : -TuningConstants.FarChargeStationX, TuningConstants.ChargeStationY); //ChargeStationFar
        Point2d P14 = new Point2d(isRed ? TuningConstants.GroundPiecesX : -TuningConstants.GroundPiecesX, TuningConstants.GroundOneY); //GroundOne
        Point2d P15 = new Point2d(isRed ? TuningConstants.GroundPiecesX : -TuningConstants.GroundPiecesX, TuningConstants.GroundTwoY);//GroundTwo
        Point2d P16 = new Point2d(isRed ? TuningConstants.GroundPiecesX : -TuningConstants.GroundPiecesX, TuningConstants.GroundThreeY);//GroundThree
        Point2d P17 = new Point2d(isRed ? TuningConstants.GroundPiecesX : -TuningConstants.GroundPiecesX, TuningConstants.GroundFourY);//GroundFour
        Point2d P18 = new Point2d(isRed ? TuningConstants.FarChargeStationX : -TuningConstants.FarChargeStationX, TuningConstants.GroundOneY); //InBetweenLoadFar
        Point2d P19 = new Point2d(isRed ? TuningConstants.FarChargeStationX : -TuningConstants.FarChargeStationX, TuningConstants.GroundFourY); //InBetweenGuardFar
        Point2d P20 = new Point2d(isRed ? TuningConstants.FarChargeStationInBetweenX : -TuningConstants.FarChargeStationInBetweenX, TuningConstants.GroundOneY);
        Point2d P21 = new Point2d(isRed ? TuningConstants.FarChargeStationInBetweenX : -TuningConstants.FarChargeStationInBetweenX, TuningConstants.GroundTwoY);
        Point2d P22 = new Point2d(isRed ? TuningConstants.FarChargeStationInBetweenX : -TuningConstants.FarChargeStationInBetweenX, TuningConstants.GroundThreeY);
        Point2d P23 = new Point2d(isRed ? TuningConstants.FarChargeStationInBetweenX : -TuningConstants.FarChargeStationInBetweenX, TuningConstants.GroundFourY);
        Point2d LoadMid = new Point2d(isRed ? TuningConstants.CloseChargeStationX : -TuningConstants.CloseChargeStationX, TuningConstants.LoadEdgeY);
        Point2d GuardMid = new Point2d(isRed ? TuningConstants.CloseChargeStationX : -TuningConstants.CloseChargeStationX, TuningConstants.GuardEdgeY);
        Point2d LoadStart = new Point2d(isRed ? TuningConstants.LoadEdgeStartX : -TuningConstants.LoadEdgeStartX, TuningConstants.LoadEdgeY);
        Point2d GuardStart = new Point2d(isRed ? TuningConstants.GuardEdgeStartX : -TuningConstants.GuardEdgeStartX, TuningConstants.GuardEdgeY);

        //2023 Paths

        //Test Path
        // addTrajectory(
        //     trajectoryManager,
        //     pathPlanner.buildTrajectory(
        //         TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
        //         TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
        //         new PathPlannerWaypoint(LoadEdge, ForwardOT, ForwardOT),
        //         new PathPlannerWaypoint(P18, ForwardOT, ForwardOT)),
        //     "LoadEdgeTo18");

        // ------------------------------- Actual Paths That should be Used --------------------------------------------
        
        // Does not need Turnary Operators, so we can put outside of isRed
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                new PathPlannerWaypoint(0, 0, ForwardOT, ForwardOT),
                new PathPlannerWaypoint(65, 0, ForwardOT)),
            "GuardTaxi");
        
        // Does not need Turnary Operators, so we can put outside of isRed
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                new PathPlannerWaypoint(0, 0, ForwardOT, ForwardOT),
                new PathPlannerWaypoint(48, 0, ForwardOT)),
            "LoadTaxi");
        
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                new PathPlannerWaypoint(P11, ForwardOT, ForwardOT),
                new PathPlannerWaypoint(P13, ForwardOT, ForwardOT)),
            isRed ? "MidTaxiRed" : "MidTaxiBlue");
        
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                new PathPlannerWaypoint(LoadStart, ForwardOT, BackwardOT),
                new PathPlannerWaypoint(P18, -Blue45_Red135OT, BackwardOT),
                new PathPlannerWaypoint(P13, -90, BackwardOT)),
                isRed ? "LSToChargeRed" : "LSToChargeBlue");
        
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                new PathPlannerWaypoint(GuardStart, ForwardOT, BackwardOT),
                new PathPlannerWaypoint(P19, Blue45_Red135OT, BackwardOT),
                new PathPlannerWaypoint(P13, 90, BackwardOT)),
                isRed ? "GSToChargeRed" : "GSToChargeBlue");
        
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                new PathPlannerWaypoint(P5, ForwardOT, ForwardOT),
                new PathPlannerWaypoint(P11, ForwardOT, ForwardOT),
                new PathPlannerWaypoint(P13, ForwardOT, ForwardOT),
                new PathPlannerWaypoint(P22, BackwardOT, BackwardOT),
                new PathPlannerWaypoint(P13, BackwardOT, BackwardOT)),
                isRed ? "5ToChargeRed" : "5ToChargeBlue");
        
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                new PathPlannerWaypoint(P5, ForwardOT, BackwardOT),
                new PathPlannerWaypoint(P11, ForwardOT, BackwardOT)),
                isRed ? "5To11Red" : "5To11Blue");
        
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                new PathPlannerWaypoint(P11, BackwardOT, BackwardOT),
                new PathPlannerWaypoint(P5, BackwardOT, BackwardOT)),
                isRed ? "11To5Red" : "11To5Blue");
                
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                new PathPlannerWaypoint(P5, ForwardOT, BackwardOT),
                new PathPlannerWaypoint(P11, ForwardOT, ForwardOT)),
                isRed ? "5To11TurnRed" : "5To11TurnBlue");
                        
                
                
                
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                new PathPlannerWaypoint(GuardStart, BackwardOT, BackwardOT),
                new PathPlannerWaypoint(GuardMid, BackwardOT, BackwardOT),
                new PathPlannerWaypoint(P9, Blue135_Red45OT, BackwardOT)),
                isRed ? "GuardStartTo9Red" : "GuardStartTo9Blue");
        
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                new PathPlannerWaypoint(P9, ForwardOT, BackwardOT),
                new PathPlannerWaypoint(P19, Blue45_Red135OT, BackwardOT),
                new PathPlannerWaypoint(P13, 90, BackwardOT)),
                isRed ? "9ToChargeRed" : "9ToChargeBlue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                new PathPlannerWaypoint(P9, Blue45_Red135OT, BackwardOT),
                new PathPlannerWaypoint(P12, Blue45_Red135OT, BackwardOT)),
                isRed ? "9To12Red" : "9To12Blue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                new PathPlannerWaypoint(P12, ForwardOT, BackwardOT),
                new PathPlannerWaypoint(P19, ForwardOT, BackwardOT),
                new PathPlannerWaypoint(P23, ForwardOT, ForwardOT)),
                isRed ? "12To23Red" : "12To23Blue");
        
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                new PathPlannerWaypoint(P23, ForwardOT, ForwardOT),
                new PathPlannerWaypoint(P17, ForwardOT, ForwardOT),
                new PathPlannerWaypoint(P23, BackwardOT, ForwardOT)),
                isRed ? "23To23Red" : "23To23Blue");
        
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                new PathPlannerWaypoint(P23, ForwardOT, ForwardOT),
                new PathPlannerWaypoint(P19, BackwardOT, BackwardOT),
                new PathPlannerWaypoint(P13, BackwardOT, BackwardOT)),
                isRed ? "23ToChargeRed" : "23ToChargeBlue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                new PathPlannerWaypoint(P23, BackwardOT, ForwardOT),
                new PathPlannerWaypoint(P19, BackwardOT, BackwardOT),
                new PathPlannerWaypoint(P12, BackwardOT, BackwardOT)),
                isRed ? "23To12Red" : "23To12Blue");
        
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                new PathPlannerWaypoint(P12, BackwardOT, BackwardOT),
                new PathPlannerWaypoint(P7, Blue135_Red45OT, BackwardOT)),
                isRed ? "12To7Red" : "12To7Blue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                new PathPlannerWaypoint(LoadStart, BackwardOT, BackwardOT),
                new PathPlannerWaypoint(LoadMid, BackwardOT, BackwardOT),
                new PathPlannerWaypoint(P1, -Blue135_Red45OT, BackwardOT)),
                isRed ? "LoadStartTo1Red" : "LoadStartTo1Blue");
        
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                new PathPlannerWaypoint(P1, ForwardOT, BackwardOT),
                new PathPlannerWaypoint(P18, -Blue45_Red135OT, BackwardOT),
                new PathPlannerWaypoint(P13, -90, BackwardOT)),
                isRed ? "1ToChargeRed" : "1ToChargeBlue");
        
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                new PathPlannerWaypoint(P1, -Blue45_Red135OT, BackwardOT),
                new PathPlannerWaypoint(P10, -Blue45_Red135OT, BackwardOT)),
                isRed ? "1To10Red" : "1To10Blue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                new PathPlannerWaypoint(P10, ForwardOT, BackwardOT),
                new PathPlannerWaypoint(P18, ForwardOT, ForwardOT),
                new PathPlannerWaypoint(P20, ForwardOT, ForwardOT)),
                isRed ? "10To20Red" : "10To20Blue");
        
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                new PathPlannerWaypoint(P20, ForwardOT, ForwardOT),
                new PathPlannerWaypoint(P14, ForwardOT, ForwardOT),
                new PathPlannerWaypoint(P20, BackwardOT, ForwardOT)),
                isRed ? "20To20Red" : "20To20Blue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                new PathPlannerWaypoint(P20, BackwardOT, ForwardOT),
                new PathPlannerWaypoint(P18, -Blue135_Red45OT, BackwardOT),
                new PathPlannerWaypoint(P13, -90, BackwardOT)),
                isRed ? "20ToChargeRed" : "20ToChargeBlue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                new PathPlannerWaypoint(P20, BackwardOT, ForwardOT),
                new PathPlannerWaypoint(P18, BackwardOT, ForwardOT),
                new PathPlannerWaypoint(P10, BackwardOT, BackwardOT)),
                isRed ? "20To10Red" : "20To10Blue");
        
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                new PathPlannerWaypoint(P10, BackwardOT, BackwardOT),
                new PathPlannerWaypoint(P3, -Blue135_Red45OT, BackwardOT)),
            "10To3");


        // Random, Should Delete Soon!
        // addTrajectory(
        //     trajectoryManager,
        //     pathPlanner.buildTrajectory(
        //         TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
        //         TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
        //         new PathPlannerWaypoint(LoadEdge, BackwardOT, BackwardOT),
        //         new PathPlannerWaypoint(P1, BackwardOT, BackwardOT),
        //         new PathPlannerWaypoint(P10, BackwardOT, BackwardOT),
        //         new PathPlannerWaypoint(P18, BackwardOT, BackwardOT),
        //         new PathPlannerWaypoint(P13, BackwardOT, BackwardOT)),
        //     "LoadEdgeToChargeStationFar");
        
        // addTrajectory(
        //     trajectoryManager,
        //     pathPlanner.buildTrajectory(
        //         TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
        //         TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
        //         new PathPlannerWaypoint(LoadEdge, -90, BackwardOT),
        //         new PathPlannerWaypoint(P1, -90, BackwardOT)),
        //     "LoadEdgeTo1");
        
        // addTrajectory(
        //     trajectoryManager,
        //     pathPlanner.buildTrajectory(
        //         TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
        //         TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
        //         new PathPlannerWaypoint(P1, -45, BackwardOT),
        //         new PathPlannerWaypoint(P10, -45, BackwardOT),
        //         new PathPlannerWaypoint(P18, ForwardOT, BackwardOT),
        //         new PathPlannerWaypoint(P20, ForwardOT, ForwardOT),
        //         new PathPlannerWaypoint(P14, ForwardOT, ForwardOT)),
        //     "1To14");
        
        // addTrajectory(
        //     trajectoryManager,
        //     pathPlanner.buildTrajectory(
        //         TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
        //         TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
        //         new PathPlannerWaypoint(P14, BackwardOT, ForwardOT),
        //         new PathPlannerWaypoint(P20, BackwardOT, BackwardOT),
        //         new PathPlannerWaypoint(P18, BackwardOT, BackwardOT),
        //         new PathPlannerWaypoint(P10, BackwardOT, BackwardOT)),
        //     "14To10");
        
        // addTrajectory(
        //     trajectoryManager,
        //     pathPlanner.buildTrajectory(
        //         TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
        //         TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
        //         new PathPlannerWaypoint(P10, -160, BackwardOT),
        //         new PathPlannerWaypoint(P2, BackwardOT, BackwardOT)),
        //     "10To2");
            

        // addTrajectory(
        //     trajectoryManager,
        //     pathPlanner.buildTrajectory(
        //         TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
        //         TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
        //         new PathPlannerWaypoint(GuardEdge, 90, BackwardOT),
        //         new PathPlannerWaypoint(P9, 45, BackwardOT),
        //         new PathPlannerWaypoint(P12, ForwardOT, BackwardOT),
        //         new PathPlannerWaypoint(P19, 90, BackwardOT),
        //         new PathPlannerWaypoint(P13, 90, BackwardOT)
        //         ),
        //     "GuardEdgeToChargeStationFar");
        
        
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