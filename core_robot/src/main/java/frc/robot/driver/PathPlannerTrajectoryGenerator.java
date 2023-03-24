package frc.robot.driver;

import frc.lib.driver.TrajectoryManager;
import frc.lib.helpers.ExceptionHelpers;
import frc.lib.robotprovider.IPathPlanner;
import frc.lib.robotprovider.ITrajectory;
import frc.lib.robotprovider.PathPlannerWaypoint;
import frc.lib.robotprovider.Point2d;
import frc.robot.TuningConstants;

public class PathPlannerTrajectoryGenerator
{
    public static void generateTrajectories(TrajectoryManager trajectoryManager, IPathPlanner pathPlanner)
    {
        // ------------------------------- Autonomous paths --------------------------------------------

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                new PathPlannerWaypoint(0, 0, 0.0, 0.0),
                new PathPlannerWaypoint(56.0, 0.0, 0.0, 0.0)),
            "GuardTaxi");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                new PathPlannerWaypoint(0, 0, 0.0, 0.0),
                new PathPlannerWaypoint(48, 0, 0.0, 0.0)),
            "LoadTaxi");
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                new PathPlannerWaypoint(0, 0, 0, 0),
                new PathPlannerWaypoint(90, 0, 0, 0)),
                "MidTaxi");

        PathPlannerTrajectoryGenerator.generateTrajectories(false, trajectoryManager, pathPlanner);
        PathPlannerTrajectoryGenerator.generateTrajectories(true, trajectoryManager, pathPlanner);

        // ------------------------------- Macro paths --------------------------------------------
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.DRIVETRAIN_LOW_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.DRIVETRAIN_LOW_PATH_TRANSLATIONAL_ACCELERATION,
                new PathPlannerWaypoint(0.0, 0.0, 180.0, 0.0),
                new PathPlannerWaypoint(-30.0, 0.0, 180.0, 0.0)),
                "goBackwards30in");
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                new PathPlannerWaypoint(0.0, 0.0, 90.0, 0.0),
                new PathPlannerWaypoint(18.0, 32.0, 0.0, 0.0)),
                "goLeft32inForward18in");
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                new PathPlannerWaypoint(0.0, 0.0, 270.0, 0.0),
                new PathPlannerWaypoint(18.0, -32.0, 0.0, 0.0)),
                "goRight32inForward18in");
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                new PathPlannerWaypoint(0.0, 0.0, 90.0, 0.0),
                new PathPlannerWaypoint(0.0, 22.0, 90.0, 0.0)),
                "goLeft22in");
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                new PathPlannerWaypoint(0.0, 0.0, 270.0, 0.0),
                new PathPlannerWaypoint(0.0, -22.0, 270.0, 0.0)),
                "goRight22in");
    }

    /**
     * Get the desired y position based on whether we are red or blue
     * @param isRed whether we are red or blue
     * @param blueYPosition the y position when we are on the blue alliance
     * @return the y position to use
     */
    public static double getYPosition(boolean isRed, double blueYPosition)
    {
        if (!isRed)
        {
            return blueYPosition;
        }

        return TuningConstants.FullWidth - blueYPosition;
    }

    /**
     * Gets the desired yaw direction based on a series of calculations
     * @param isRed whether we are on the red team or blue team
     * @param reverse whether we want to have "straight" mean backwards (toward grid) or forwards (away from grid)
     * @param towardLoading whether we want to angle ourselves towards the loading zone or the guardrail
     * @param percentFromStraight the percentage from straight we want to go (0.0 to 1.0)
     * @return the yaw direction to use
     */
    public static double getDirection(boolean isRed, boolean reverse, boolean towardLoading, double percentFromStraight)
    {
        ExceptionHelpers.Assert(percentFromStraight >= 0.0 && percentFromStraight <= 1.0, "percentFromStraight should be between 0 and 1");

        if (percentFromStraight == 0.0)
        {
            return reverse ? 180.0 : 0.0;
        }

        if (isRed)
        {
            if (reverse)
            {
                if (towardLoading)
                {
                    return 180.0 + 90.0 * percentFromStraight;
                }
                else
                {
                    return 180.0 - 90.0 * percentFromStraight;
                }
            }
            else
            {
                if (towardLoading)
                {
                    return 360.0 - 90.0 * percentFromStraight;
                }
                else
                {
                    return 0.0 + 90.0 * percentFromStraight;
                }
            }
        }
        else
        {
            if (reverse)
            {
                if (towardLoading)
                {
                    return 180.0 - 90.0 * percentFromStraight;
                }
                else
                {
                    return 180.0 + 90.0 * percentFromStraight;
                }
            }
            else
            {
                if (towardLoading)
                {
                    return 0.0 + 90.0 * percentFromStraight;
                }
                else
                {
                    return 360.0 - 90.0 * percentFromStraight;
                }
            }
        }
    }

    public static void generateTrajectories(boolean isRed, TrajectoryManager trajectoryManager, IPathPlanner pathPlanner)
    {
        // TANGENTS AND ORIENTATION:
        // +x = 0 away from alliance's grid
        // -x = 180 toward alliance's grid
        // -y = -90 to right from alliance's grid
        // +y = 90 to left from alliance's grid
        final double ForwardOT = 0.0;
        final double BackwardOT = 180.0;
        double LoadOT = isRed ? -90.0 : 90.0; // use ternary operator to determine whether we are facing left or right to go towards loading station (substation)
        double GuardOT = isRed ? 90.0 : -90.0; // use ternary operator to determine whether we are facing left or right to go towards guardrail (away from substation)
        double ForwardLoadOT = isRed ? -45.0 : 45.0; // ternary operator for facing forwards and towards the loading station, in terms of Orientation or Tangent
        double ForwardGuard = isRed ? 45.0 : -45.0; // ternary operator for facing forwards and towards the guardrail, in terms of Orientation or Tangent
        double BackwardLoadOT = isRed ? -135.0 : 135.0; // ternary operator for facing backwards and towards the loading station, in terms of Orientation or Tangent
        double BackwardGuardOT = isRed ? 135.0 : -135.0; // ternary operator for facing backwards and towards the guardrail, in terms of Orientation or Tangent

        //Vectors
        Point2d P1 = new Point2d(TuningConstants.StartGridX, getYPosition(isRed, TuningConstants.StartOneGridY)); // StartOneGrid
        Point2d LoadEdge = new Point2d(TuningConstants.StartGridX, getYPosition(isRed, TuningConstants.LoadEdgeY)); // Load side edge of grid
        Point2d P2 = new Point2d(TuningConstants.StartGridX, getYPosition(isRed, TuningConstants.StartTwoGridY)); // StartTwoGrid
        Point2d P3 = new Point2d(TuningConstants.StartGridX, getYPosition(isRed, TuningConstants.StartThreeGridY)); // StartThreeGrid
        Point2d P4 = new Point2d(TuningConstants.StartGridX, getYPosition(isRed, TuningConstants.StartFourGridY)); // StartFourGrid
        Point2d P5 = new Point2d(TuningConstants.StartGridX, getYPosition(isRed, TuningConstants.StartFiveGridY)); // StartFiveGrid
        Point2d P6 = new Point2d(TuningConstants.StartGridX, getYPosition(isRed, TuningConstants.StartSixGridY)); // StartSixGrid
        Point2d P7 = new Point2d(TuningConstants.StartGridX, getYPosition(isRed, TuningConstants.StartSevenGridY)); // StartSevenGrid
        Point2d P8 = new Point2d(TuningConstants.StartGridX, getYPosition(isRed, TuningConstants.StartEightGridY)); // StartEightGrid
        Point2d P9 = new Point2d(TuningConstants.StartGridX, getYPosition(isRed, TuningConstants.StartNineGridY)); // StartNineGrid
        Point2d GuardEdge = new Point2d(TuningConstants.StartGridX, getYPosition(isRed, TuningConstants.GuardEdgeY)); // Guard side edge of grid
        Point2d P10 = new Point2d(TuningConstants.CloseChargeStationX,getYPosition(isRed, TuningConstants.GroundOneY)); // InBetweenLoadClose
        Point2d P11 = new Point2d(TuningConstants.CloseChargeStationX, getYPosition(isRed, TuningConstants.ChargeStationY)); // ChargeStationClose
        Point2d P12 = new Point2d(TuningConstants.CloseChargeStationX, getYPosition(isRed, TuningConstants.GroundFourY)); // InBetweenGuardClose
        Point2d P13 = new Point2d(TuningConstants.FarChargeStationX, getYPosition(isRed, TuningConstants.ChargeStationY)); // ChargeStationFar
        Point2d P14 = new Point2d(TuningConstants.GroundPiecesX, getYPosition(isRed, TuningConstants.GroundOneY)); // GroundOne
        Point2d P15 = new Point2d(TuningConstants.GroundPiecesX, getYPosition(isRed, TuningConstants.GroundTwoY)); // GroundTwo
        Point2d P16 = new Point2d(TuningConstants.GroundPiecesX, getYPosition(isRed, TuningConstants.GroundThreeY)); // GroundThree
        Point2d P17 = new Point2d(TuningConstants.GroundPiecesX, getYPosition(isRed, TuningConstants.GroundFourY)); // GroundFour
        Point2d P18 = new Point2d(TuningConstants.FarChargeStationX, getYPosition(isRed, TuningConstants.GroundOneY)); // InBetweenLoadFar
        Point2d P19 = new Point2d(TuningConstants.FarChargeStationX, getYPosition(isRed, TuningConstants.GroundFourY)); // InBetweenGuardFar
        Point2d P20 = new Point2d(TuningConstants.FarChargeStationInBetweenX, getYPosition(isRed, TuningConstants.GroundOneY));
        Point2d P21 = new Point2d(TuningConstants.FarChargeStationInBetweenX, getYPosition(isRed, TuningConstants.GroundTwoY));
        Point2d P22 = new Point2d(TuningConstants.FarChargeStationInBetweenX, getYPosition(isRed, TuningConstants.GroundThreeY));
        Point2d P23 = new Point2d(TuningConstants.FarChargeStationInBetweenX, getYPosition(isRed, TuningConstants.GroundFourY));
        Point2d P24 = new Point2d(TuningConstants.BetweenBumpAndChargeStationFarX, getYPosition(isRed, TuningConstants.TurnGuardY));
        Point2d P25 = new Point2d(TuningConstants.BetweenBumpAndChargeStationFarX, getYPosition(isRed, TuningConstants.TurnLoadY));
        Point2d LoadMid = new Point2d(TuningConstants.CloseChargeStationX, getYPosition(isRed, TuningConstants.LoadEdgeY));
        Point2d GuardMid = new Point2d(TuningConstants.CloseChargeStationX, getYPosition(isRed, TuningConstants.GuardEdgeY));
        Point2d LoadStart = new Point2d(TuningConstants.LoadEdgeStartX, getYPosition(isRed, TuningConstants.LoadEdgeY));
        Point2d GuardStart = new Point2d(TuningConstants.GuardEdgeStartX, getYPosition(isRed, TuningConstants.GuardEdgeY));

        // addTrajectory(
        //     trajectoryManager,
        //     pathPlanner.buildTrajectory(
        //         TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
        //         TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
        //         new PathPlannerWaypoint(GuardStart, BackwardOT, BackwardOT),
        //         new PathPlannerWaypoint(GuardMid, BackwardOT, BackwardOT),
        //         new PathPlannerWaypoint(P8, ForwardGuard, BackwardOT)),
        //         isRed ? "GuardStartTo8Red" : "GuardStartTo8Blue");

        // addTrajectory(
        //     trajectoryManager,
        //     pathPlanner.buildTrajectory(
        //         TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
        //         TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
        //         new PathPlannerWaypoint(P8, ForwardOT, BackwardOT),
        //         new PathPlannerWaypoint(P19, ForwardLoadOT, BackwardOT),
        //         new PathPlannerWaypoint(P13, LoadOT, BackwardOT)),
        //         isRed ? "8ToChargeRed" : "8ToChargeBlue");

        // addTrajectory(
        //     trajectoryManager,
        //     pathPlanner.buildTrajectory(
        //         TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
        //         TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
        //         new PathPlannerWaypoint(P8, ForwardOT, BackwardOT),
        //         new PathPlannerWaypoint(P12, ForwardOT, BackwardOT)),
        //         isRed ? "8To12Red" : "8To12Blue");

        // addTrajectory(
        //     trajectoryManager,
        //     pathPlanner.buildTrajectory(
        //         TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
        //         TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
        //         new PathPlannerWaypoint(P12, ForwardOT, BackwardOT),
        //         new PathPlannerWaypoint(P19, ForwardOT, BackwardOT),
        //         new PathPlannerWaypoint(P23, ForwardOT, ForwardOT)),
        //         isRed ? "12To23Red" : "12To23Blue");

        // addTrajectory(
        //     trajectoryManager,
        //     pathPlanner.buildTrajectory(
        //         TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
        //         TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
        //         new PathPlannerWaypoint(P23, ForwardOT, ForwardOT),
        //         new PathPlannerWaypoint(P17, ForwardOT, ForwardOT),
        //         new PathPlannerWaypoint(P23, BackwardOT, ForwardOT)),
        //         isRed ? "23To23Red" : "23To23Blue");

        // addTrajectory(
        //     trajectoryManager,
        //     pathPlanner.buildTrajectory(
        //         TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
        //         TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
        //         new PathPlannerWaypoint(P23, ForwardOT, ForwardOT),
        //         new PathPlannerWaypoint(P19, BackwardOT, BackwardOT),
        //         new PathPlannerWaypoint(P13, BackwardOT, BackwardOT)),
        //         isRed ? "23ToChargeRed" : "23ToChargeBlue");

        // addTrajectory(
        //     trajectoryManager,
        //     pathPlanner.buildTrajectory(
        //         TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
        //         TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
        //         new PathPlannerWaypoint(P23, BackwardOT, ForwardOT),
        //         new PathPlannerWaypoint(P19, BackwardOT, BackwardOT),
        //         new PathPlannerWaypoint(P12, BackwardOT, BackwardOT)),
        //         isRed ? "23To12Red" : "23To12Blue");

        // addTrajectory(
        //     trajectoryManager,
        //     pathPlanner.buildTrajectory(
        //         TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
        //         TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
        //         new PathPlannerWaypoint(P12, BackwardOT, BackwardOT),
        //         new PathPlannerWaypoint(P9, BackwardOT, BackwardOT)),
        //         isRed ? "12To9Red" : "12To9Blue");

        // addTrajectory(
        //     trajectoryManager,
        //     pathPlanner.buildTrajectory(
        //         TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
        //         TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
        //         new PathPlannerWaypoint(LoadStart, BackwardOT, BackwardOT),
        //         new PathPlannerWaypoint(LoadMid, BackwardOT, BackwardOT),
        //         new PathPlannerWaypoint(P2, -ForwardGuard, BackwardOT)),
        //         isRed ? "LoadStartTo2Red" : "LoadStartTo2Blue");

        // addTrajectory(
        //     trajectoryManager,
        //     pathPlanner.buildTrajectory(
        //         TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
        //         TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
        //         new PathPlannerWaypoint(P2, ForwardOT, BackwardOT),
        //         new PathPlannerWaypoint(P18, -ForwardLoadOT, BackwardOT),
        //         new PathPlannerWaypoint(P13, GuardOT, BackwardOT)),
        //         isRed ? "2ToChargeRed" : "2ToChargeBlue");

        // addTrajectory(
        //     trajectoryManager,
        //     pathPlanner.buildTrajectory(
        //         TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
        //         TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
        //         new PathPlannerWaypoint(P2, ForwardOT, BackwardOT),
        //         new PathPlannerWaypoint(P10, ForwardOT, BackwardOT)),
        //         isRed ? "2To10Red" : "2To10Blue");

        // addTrajectory(
        //     trajectoryManager,
        //     pathPlanner.buildTrajectory(
        //         TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
        //         TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
        //         new PathPlannerWaypoint(P10, ForwardOT, BackwardOT),
        //         new PathPlannerWaypoint(P18, ForwardOT, ForwardOT),
        //         new PathPlannerWaypoint(P20, ForwardOT, ForwardOT)),
        //         isRed ? "10To20Red" : "10To20Blue");

        // addTrajectory(
        //     trajectoryManager,
        //     pathPlanner.buildTrajectory(
        //         TuningConstants.DRIVETRAIN_LOW_PATH_TRANSLATIONAL_VELOCITY,
        //         TuningConstants.DRIVETRAIN_LOW_PATH_TRANSLATIONAL_ACCELERATION,
        //         new PathPlannerWaypoint(P20, ForwardOT, ForwardOT),
        //         new PathPlannerWaypoint(P14, ForwardOT, ForwardOT),
        //         new PathPlannerWaypoint(P20, BackwardOT, ForwardOT)),
        //         isRed ? "20To20Red" : "20To20Blue");

        // addTrajectory(
        //     trajectoryManager,
        //     pathPlanner.buildTrajectory(
        //         TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
        //         TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
        //         new PathPlannerWaypoint(P20, BackwardOT, ForwardOT),
        //         new PathPlannerWaypoint(P18, -ForwardGuard, BackwardOT),
        //         new PathPlannerWaypoint(P13, GuardOT, BackwardOT)),
        //         isRed ? "20ToChargeRed" : "20ToChargeBlue");

        // addTrajectory(
        //     trajectoryManager,
        //     pathPlanner.buildTrajectory(
        //         TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
        //         TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
        //         new PathPlannerWaypoint(P20, BackwardOT, ForwardOT),
        //         new PathPlannerWaypoint(P18, BackwardOT, ForwardOT),
        //         new PathPlannerWaypoint(P10, BackwardOT, BackwardOT)),
        //         isRed ? "20To10Red" : "20To10Blue");

        // addTrajectory(
        //     trajectoryManager,
        //     pathPlanner.buildTrajectory(
        //         TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
        //         TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
        //         new PathPlannerWaypoint(P10, BackwardOT, BackwardOT),
        //         new PathPlannerWaypoint(P1, BackwardOT, BackwardOT)),
        //     isRed ? "10To1Red" : "10To1Blue");
        
        //New Paths

            //Guard Side
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                new PathPlannerWaypoint(GuardEdge, LoadOT, -180),
                new PathPlannerWaypoint(P9, LoadOT, -180)),
                isRed ? "GuardEdgeTo9Red" : "GuardEdgeTo9Blue");
            
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                new PathPlannerWaypoint(P9, 0, 180),
                //new PathPlannerWaypoint(P12, 0, 180),
                new PathPlannerWaypoint(P24, 0, 180),
                new PathPlannerWaypoint(P19, 0, 0.0000000000000001),
                new PathPlannerWaypoint(P17, 0 , 0)),
                isRed ? "9To17Red" : "9To17Blue");
        
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                new PathPlannerWaypoint(P9, 0, -180),
                new PathPlannerWaypoint(P19, BackwardLoadOT, -180),
                new PathPlannerWaypoint(P13, LoadOT, -180)),
                isRed ? "9ToChargeRed" : "9ToChargeBlue");
        
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                new PathPlannerWaypoint(P9, 0, -180),
                new PathPlannerWaypoint(P23, 0, -180)),
                isRed ? "9To23Red" : "9To23Blue");
        
        
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                new PathPlannerWaypoint(P17, -180, 0),
                new PathPlannerWaypoint(P23, -180, 0),
                //new PathPlannerWaypoint(P12, -180, -180),
                new PathPlannerWaypoint(P7, BackwardLoadOT, -180)),
                isRed ? "17To7Red" : "17To7Blue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                new PathPlannerWaypoint(P17, -180, 0),
                new PathPlannerWaypoint(P19, -180, 0),
                new PathPlannerWaypoint(P13, LoadOT , 0)),
                isRed ? "17ToChargeStationRed" : "17ToChargeStationBlue");
        
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                new PathPlannerWaypoint(P17, 180, 0),
                new PathPlannerWaypoint(P23, 180, LoadOT),
                new PathPlannerWaypoint(P19, 180, 180),
                //new PathPlannerWaypoint(P12, 180, BackwardLoadOT),
                new PathPlannerWaypoint(P7, 180, BackwardLoadOT)),
                isRed ? "17To7Red" : "17To7Blue");

            //Load Side
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                new PathPlannerWaypoint(LoadEdge, GuardOT, -180),
                new PathPlannerWaypoint(P1, GuardOT, -180)),
                isRed ? "LoadEdgeTo1Red" : "LoadEdgeTo1Blue");
            
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                new PathPlannerWaypoint(P1, 0, -180),
                //new PathPlannerWaypoint(P10, 0, -180),
                new PathPlannerWaypoint(P25, 0, 180),
                new PathPlannerWaypoint(P18, 0, 0.0000000000000001), //Check with will to make sure path planner doesn't round stuff
                new PathPlannerWaypoint(P14, 0 , 0)),
                isRed ? "1To14Red" : "1To14Blue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                new PathPlannerWaypoint(P1, 0, -180),
                new PathPlannerWaypoint(P18, BackwardGuardOT, -180),
                new PathPlannerWaypoint(P13, GuardOT, -180)),
                isRed ? "1ToChargeRed" : "1ToChargeBlue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                new PathPlannerWaypoint(P1, 0, -180),
                new PathPlannerWaypoint(P20, 0, -180)),
                isRed ? "1To20Red" : "1To20Blue");
        
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                new PathPlannerWaypoint(P14, -180, 0),
                new PathPlannerWaypoint(P20, -180, 0),
                //new PathPlannerWaypoint(P10, -180, -180),
                new PathPlannerWaypoint(P3, BackwardGuardOT , -180)),
                isRed ? "14To3Red" : "14To3Blue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                new PathPlannerWaypoint(P14, -180, 0),
                new PathPlannerWaypoint(P18, -180, 0),
                new PathPlannerWaypoint(P13, GuardOT , 0)),
                isRed ? "14ToChargeStationRed" : "14ToChargeStationBlue");
        
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                new PathPlannerWaypoint(P14, 180, 0),
                new PathPlannerWaypoint(P20, 180,  LoadOT),
                new PathPlannerWaypoint(P18, 180, 180),
                //new PathPlannerWaypoint(P10, 180, BackwardGuardOT),
                new PathPlannerWaypoint(P3, 180, BackwardGuardOT)),
                isRed ? "14To3Red" : "14To3Blue");
        
            // MID SIDE        
        
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                new PathPlannerWaypoint(P5, 0, 180),
                new PathPlannerWaypoint(P11, 0, 180)),
                isRed ? "5To11Red" : "5To11Blue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.DRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                new PathPlannerWaypoint(P11, 180, 180),
                new PathPlannerWaypoint(P5, 180, 180)),
                isRed ? "11To5Red" : "11To5Blue");
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