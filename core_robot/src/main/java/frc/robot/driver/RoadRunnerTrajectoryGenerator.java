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
import frc.robot.driver.common.PathManager;
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
        PathManager pathManager = new PathManager();
        RoadRunnerTrajectoryGenerator.generateTrajectories(pathManager);
        ITrajectory trajectory = pathManager.getTrajectory("w2ba-goToPickUpBall2");

        try (CsvWriter csvWriter = CsvWriter.builder().build(java.nio.file.Path.of("test.csv"), StandardCharsets.UTF_8))
        {
            csvWriter.writeRow("t", "x", "y", "theta", "vx", "vy", "omega");

            for (double t = 0.0; t < trajectory.getDuration() + 0.01; t += 0.02)
            {
                TrajectoryState state = trajectory.get(t);
                csvWriter.writeRow(
                    Double.toString(t),
                    Double.toString(state.xPosition),
                    Double.toString(state.yPosition),
                    Double.toString(state.angle),
                    Double.toString(state.xVelocity),
                    Double.toString(state.yVelocity),
                    Double.toString(state.angleVelocity));
            }

            csvWriter.close();
        }
        catch (IOException e)
        {
        }
    }

    public static void generateTrajectories(PathManager pathManager)
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
        double BlueStartGridX = 70.188;
        double BlueStartUpGridY = 174.19;
        double BlueStartMiddleGridY = 108.19;
        double BlueStartDownGridY = 42.19;
        double BlueFrontChargeStationX = 98.5;
        double BlueBackChargeStationX = 207.001;
        double BlueUpChargeStationY = 132.3275;
        double BlueMiddleChargeStationY = 108.015;
        double BlueDownChargeStationY = 83.7025;
        double BlueMidpointAfterChargeStationX = 213.5;
        double BlueGroundNodesX = 260.455;
        double BlueGroundNodeFarUpY = 180.19;
        double BlueGroundNodeFarDownY = 36.19;
        double BlueGroundNodeSecondUpY = 132.19;
        double BlueGroundNodeSecondDownY = 84.19;
        double BlueConeGridFarUpY;
        double BlueConeGridSecondUpY;
        double BlueConeGridMiddleY;
        double BlueConeGridSecondDownY;
        double BlueConeGridFarDownY = 20.935;
        


        addPath(
            pathManager,
            startTrajectory(BlueStartGridX,  BlueStartMiddleGridY,  180.0 * Helpers.DEGREES_TO_RADIANS, 0.0 * Helpers.DEGREES_TO_RADIANS)
                //Task places cone on node
                .splineToConstantHeading(new Vector2d(BlueFrontChargeStationX, BlueStartMiddleGridY), 0.0 * Helpers.DEGREES_TO_RADIANS), // Goes infront of charge staton
                //Jamie's Charge Station task
            "MiddleStartToChargeStationFront");

        addPath(
            pathManager,
            startTrajectory(BlueStartGridX, BlueStartUpGridY, 180.0 * Helpers.DEGREES_TO_RADIANS, 180.0 * Helpers.DEGREES_TO_RADIANS)
                //Task places cone on node
                .splineToSplineHeading(new Pose2d(BlueMidpointAfterChargeStationX, BlueStartUpGridY), 0.0 * Helpers.DEGREES_TO_RADIANS) // Spline to midpoint
                .splineToConstantHeading(new Vector2d(BlueGroundNodesX, BlueGroundNodeFarUpY), 0.0 * Helpers.DEGREES_TO_RADIANS) // Spline to ground node far up
                //Pickup cone
                .splineToSplineHeading(new Pose2d(BlueMidpointAfterChargeStationX, BlueStartUpGridY), 180.0 * Helpers.DEGREES_TO_RADIANS) // Spline to midpoint
                .splineToConstantHeading(new Vector2d(BlueStartGridX,BlueStartUpGridY), 180.0 * Helpers.DEGREES_TO_RADIANS), // Spline to infront of the grid
                //Task places cone on node
            "UpStartToGroundNodeFarUpToUpStart");

        addPath(
            pathManager,
            startTrajectory(BlueStartGridX, BlueStartUpGridY, 180.0 * Helpers.DEGREES_TO_RADIANS, 180.0 * Helpers.DEGREES_TO_RADIANS)
                //Task places cone on node
                .splineToSplineHeading(new Pose2d(BlueMidpointAfterChargeStationX, BlueStartUpGridY), 0.0 * Helpers.DEGREES_TO_RADIANS) // Spline to midpoint
                .splineToConstantHeading(new Vector2d(BlueGroundNodesX, BlueGroundNodeFarUpY), 0.0 * Helpers.DEGREES_TO_RADIANS) // Spline to ground node far up
                //Pickup cone
                .splineToSplineHeading(new Pose2d(BlueMidpointAfterChargeStationX, BlueStartUpGridY), 180.0 * Helpers.DEGREES_TO_RADIANS) // Spline to midpoint
                .splineToConstantHeading(new Vector2d(BlueStartGridX, BlueStartUpGridY), 180.0 * Helpers.DEGREES_TO_RADIANS) // Spline to infront of the grid
                //Task places cone on node
                .splineToSplineHeading(new Pose2d(BlueFrontChargeStationX, BlueUpChargeStationY), 0.0 * Helpers.DEGREES_TO_RADIANS), // Goes infront of charge staton
                //Jamie's Charge Station Task

            "UpStartToGroundNodeFarUpToUpStartToFrontChargeStation");
        addPath(
            pathManager,
            startTrajectory(BlueStartGridX, BlueStartUpGridY, 180.0 * Helpers.DEGREES_TO_RADIANS, 180.0 * Helpers.DEGREES_TO_RADIANS)
                //Task places cone on node
                .splineToSplineHeading(new Pose2d(BlueFrontChargeStationX, BlueUpChargeStationY), 0.0 * Helpers.DEGREES_TO_RADIANS), // Goes infront of charge staton
                //Jamie's Charge Station Task

            "UpStartToFrontChargeStation");
    
        addPath(
            pathManager,
            startTrajectory(BlueStartGridX, BlueStartDownGridY, 180.0 * Helpers.DEGREES_TO_RADIANS, 180.0 * Helpers.DEGREES_TO_RADIANS)
                //Task places cone on node
                .splineToSplineHeading(new Pose2d(BlueMidpointAfterChargeStationX, BlueStartDownGridY), 0.0 * Helpers.DEGREES_TO_RADIANS) // Spline to midpoint
                .splineToConstantHeading(new Vector2d(BlueGroundNodesX, BlueGroundNodeFarDownY), 0.0 * Helpers.DEGREES_TO_RADIANS) // Spline to ground node far down
                //Pickup cone
                .splineToSplineHeading(new Pose2d(BlueMidpointAfterChargeStationX, BlueStartDownGridY), 180.0 * Helpers.DEGREES_TO_RADIANS) // Spline to midpoint
                .splineToConstantHeading(new Vector2d(BlueStartGridX, BlueStartDownGridY), 180.0 * Helpers.DEGREES_TO_RADIANS), // Spline to infront of the grid
                //Task places cone on node
            "DownStartToGroundNodeFarDownToDownStart");

        addPath(
            pathManager,
            startTrajectory(BlueStartGridX, BlueStartDownGridY, 180.0 * Helpers.DEGREES_TO_RADIANS, 180.0 * Helpers.DEGREES_TO_RADIANS)
                //Task places cone on node
                .splineToSplineHeading(new Pose2d(BlueMidpointAfterChargeStationX, BlueStartDownGridY), 0.0 * Helpers.DEGREES_TO_RADIANS) // Spline to midpoint
                .splineToConstantHeading(new Vector2d(BlueGroundNodesX, BlueGroundNodeFarDownY), 0.0 * Helpers.DEGREES_TO_RADIANS) // Spline to ground node far down
                //Pickup cone
                .splineToSplineHeading(new Pose2d(BlueMidpointAfterChargeStationX, BlueStartDownGridY), 180.0 * Helpers.DEGREES_TO_RADIANS) // Spline to midpoint
                .splineToConstantHeading(new Vector2d(BlueStartGridX, BlueStartDownGridY), 180.0 * Helpers.DEGREES_TO_RADIANS) // Spline to grid
                //Task places cone on node
                .splineToSplineHeading(new Pose2d(BlueFrontChargeStationX, BlueDownChargeStationY), 0.0 * Helpers.DEGREES_TO_RADIANS), // Goes infront of charge staton
                //Jamie's Charge Station Task
            "DownStartToGroundNodeFarDownToDownStartToFrontChargeStation");
        
        addPath(
            pathManager,
            startTrajectory(BlueStartGridX, BlueStartDownGridY, 180.0 * Helpers.DEGREES_TO_RADIANS, 180.0 * Helpers.DEGREES_TO_RADIANS)
                //Task places cone on node
                .splineToSplineHeading(new Pose2d(BlueFrontChargeStationX, BlueDownChargeStationY), 0.0 * Helpers.DEGREES_TO_RADIANS), // Goes infront of charge staton
                //Jamie's Charge Station Task
            "DownStartToFrontChargeStation");

        
        addPath(
            pathManager,
            startTrajectory(BlueStartGridX, BlueStartMiddleGridY, 180.0 * Helpers.DEGREES_TO_RADIANS, 180.0 * Helpers.DEGREES_TO_RADIANS)
                //Task places cone on node
                .splineToSplineHeading(new Pose2d(BlueMidpointAfterChargeStationX, BlueStartMiddleGridY), 0.0 * Helpers.DEGREES_TO_RADIANS) // Spline to Midpoint past the charge station
                .splineToConstantHeading(new Vector2d(BlueGroundNodesX, BlueGroundNodeSecondUpY), 0.0 * Helpers.DEGREES_TO_RADIANS) // Spline to the ground node Second Up
                //Pickup cone
                .splineToSplineHeading(new Pose2d(BlueMidpointAfterChargeStationX, BlueStartMiddleGridY), 180.0 * Helpers.DEGREES_TO_RADIANS) // Spline to Midpoint past the charge station
                .splineToConstantHeading(new Vector2d(BlueStartGridX, BlueStartMiddleGridY), 180.0 * Helpers.DEGREES_TO_RADIANS) // Spline to Grid
                .splineToSplineHeading(new Pose2d(BlueFrontChargeStationX, BlueMiddleChargeStationY), 0.0 * Helpers.DEGREES_TO_RADIANS), // Goes infront of charge staton
                //Jamie's Charge Station Task
            "MiddleStartToConeSecondUpToMiddleStartToFrontChargeStation");

        addPath(
            pathManager,
            startTrajectory(BlueStartGridX, BlueStartMiddleGridY, 180.0 * Helpers.DEGREES_TO_RADIANS, 180.0 * Helpers.DEGREES_TO_RADIANS)
                //Task places cone on node
                .splineToSplineHeading(new Pose2d(BlueMidpointAfterChargeStationX, BlueStartMiddleGridY), 0.0 * Helpers.DEGREES_TO_RADIANS) // Spline to Midpoint past the charge station
                .splineToConstantHeading(new Vector2d(BlueGroundNodesX, BlueGroundNodeSecondDownY), 0.0 * Helpers.DEGREES_TO_RADIANS) // Spline to the ground node Second Down
                //Pickup cone
                .splineToSplineHeading(new Pose2d(BlueMidpointAfterChargeStationX, BlueStartMiddleGridY), 180.0 * Helpers.DEGREES_TO_RADIANS) // Spline to Midpoint past the charge station
                .splineToConstantHeading(new Vector2d(BlueStartGridX, BlueStartMiddleGridY), 180.0 * Helpers.DEGREES_TO_RADIANS) // Spline to Grid
                .splineToSplineHeading(new Pose2d(BlueFrontChargeStationX, BlueMiddleChargeStationY), 0.0 * Helpers.DEGREES_TO_RADIANS), // Goes infront of charge staton
                //Jamie's Charge Station Task
            "MiddleStartToConeSecondDownToMiddleStartToFrontChargeStation");

        addPath(
            pathManager,
            startTrajectory(BlueStartGridX, BlueStartUpGridY, 180.0 * Helpers.DEGREES_TO_RADIANS, 180.0 * Helpers.DEGREES_TO_RADIANS)
                //Task places cone on node
                .splineToSplineHeading(new Pose2d(BlueMidpointAfterChargeStationX, BlueStartUpGridY), 0.0 * Helpers.DEGREES_TO_RADIANS) // Splines to midpoint
                .splineToSplineHeading(new Pose2d(BlueBackChargeStationX, BlueUpChargeStationY), 180.0 * Helpers.DEGREES_TO_RADIANS), // Goes behind the Charge Station
                //Jamie's Charge Station Task

            "UpStartToBackChargeStation");

        addPath(
            pathManager,
            startTrajectory(BlueStartGridX, BlueStartUpGridY, 180.0 * Helpers.DEGREES_TO_RADIANS, 180.0 * Helpers.DEGREES_TO_RADIANS)
                //Task places cone on node
                .splineToSplineHeading(new Pose2d(BlueMidpointAfterChargeStationX, BlueStartUpGridY), 0.0 * Helpers.DEGREES_TO_RADIANS) // Splines to midpoint
                .splineToSplineHeading(new Pose2d(BlueBackChargeStationX, BlueUpChargeStationY), 180.0 * Helpers.DEGREES_TO_RADIANS), // Goes behind Charge Station
                //Jamie's Charge Station Task

            "UpStartToBackChargeStation");
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

    private static void addPath(PathManager pathManager, TrajectoryBuilder trajectoryBuilder, String name)
    {
        try
        {
            pathManager.addPath(name, new TrajectoryWrapper(trajectoryBuilder.build()));
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