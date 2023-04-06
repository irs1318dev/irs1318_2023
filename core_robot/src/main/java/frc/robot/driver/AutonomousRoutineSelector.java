package frc.robot.driver;

import com.google.inject.Inject;
import com.google.inject.Singleton;

import frc.lib.driver.IControlTask;
import frc.lib.driver.TrajectoryManager;
import frc.lib.mechanisms.LoggingManager;
import frc.lib.robotprovider.*;
import frc.robot.LoggingKey;
import frc.robot.TuningConstants;
import frc.robot.driver.SmartDashboardSelectionManager.AutoRoutine;
import frc.robot.driver.SmartDashboardSelectionManager.StartPosition;
import frc.robot.driver.controltasks.*;
import frc.robot.driver.controltasks.ArmLAPositionTask.IntakeState;
import frc.robot.driver.controltasks.ChargeStationTask.Orientation;
import frc.robot.driver.controltasks.FollowPathTask.Type;

@Singleton
public class AutonomousRoutineSelector
{
    private final ILogger logger;

    private final TrajectoryManager trajectoryManager;
    private final SmartDashboardSelectionManager selectionManager;
    private final IDriverStation driverStation;

    /**
     * Initializes a new AutonomousRoutineSelector
     */
    @Inject
    public AutonomousRoutineSelector(
        LoggingManager logger,
        TrajectoryManager trajectoryManager,
        SmartDashboardSelectionManager selectionManager,
        IRobotProvider provider)
    {
        this.logger = logger;
        this.trajectoryManager = trajectoryManager;
        this.selectionManager = selectionManager;

        this.driverStation = provider.getDriverStation();

        RoadRunnerTrajectoryGenerator.generateTrajectories(this.trajectoryManager);
        PathPlannerTrajectoryGenerator.generateTrajectories(this.trajectoryManager, provider.getPathPlanner());
    }

    /**
     * Check what routine we want to use and return it
     * @param mode that is starting
     * @return autonomous routine to execute during autonomous mode
     */
    public IControlTask selectRoutine(RobotMode mode)
    {
        String driverStationMessage = this.driverStation.getGameSpecificMessage();
        this.logger.logString(LoggingKey.AutonomousDSMessage, driverStationMessage);
        if (mode == RobotMode.Test)
        {
            return SequentialTask.Sequence(
                new ArmZeroTask(),
                new IntakePositionTask(false));
        }

        if (mode == RobotMode.Autonomous)
        {

            StartPosition startPosition = this.selectionManager.getSelectedStartPosition();
            AutoRoutine routine = this.selectionManager.getSelectedAutoRoutine();

            boolean isRed = this.driverStation.getAlliance() == Alliance.Red;

            this.logger.logString(LoggingKey.AutonomousSelection, startPosition.toString() + "." + routine.toString() + "(" + (isRed ? "red" : "blue") + ")");

            if (startPosition == StartPosition.Load)
            {
                if (routine == AutoRoutine.Taxi)
                {
                    return loadTaxi();
                }
                
                else if (routine == AutoRoutine.OnePlusTaxi)
                {
                    return loadOnePlusTaxi(isRed);
                }

                else if (routine == AutoRoutine.OnePlusCharge)
                {
                    return loadOnePlusCharge(isRed);
                }

                else if (routine == AutoRoutine.OnePickupCharge)
                {
                    return loadOnePickupCharge(isRed);
                }

                else if (routine == AutoRoutine.OnePlusPickup)
                {
                    return loadOnePickup(isRed);
                }

                else if (routine == AutoRoutine.OnePlusOne)
                {
                    return loadOnePlusOne(isRed);
                }

                else
                {
                    return ConcurrentTask.AllTasks(
                        new ResetLevelTask(),
                        new PositionStartingTask(
                            TuningConstants.LoadEdgeStartX,
                            PathPlannerTrajectoryGenerator.getYPosition(isRed, TuningConstants.LoadEdgeY),
                            180.0,
                            true,
                            true));
                }
            }
            else if (startPosition == StartPosition.Mid)
            {
                if (routine == AutoRoutine.Taxi)
                {
                    return midTaxi(isRed);
                }

                else if (routine == AutoRoutine.Charge)
                {
                    return midCharge(isRed);
                }
                
                else if (routine == AutoRoutine.OnePlusCharge)
                {
                    return midOnePlusCharge(isRed);
                }
                else if (routine == AutoRoutine.OnePlusTaxi)
                {
                    return midOneTaxiCharge(isRed);
                }

                else
                {
                    return ConcurrentTask.AllTasks(
                        new ResetLevelTask(),
                        new PositionStartingTask(
                            TuningConstants.StartGridX,
                            PathPlannerTrajectoryGenerator.getYPosition(isRed, TuningConstants.StartFiveGridY),
                            0.0,
                            true,
                            true));
                }
            }
            else if (startPosition == StartPosition.Guard)
            {
                if (routine == AutoRoutine.Taxi)
                {
                    return guardTaxi();
                }

                else if (routine == AutoRoutine.OnePlusTaxi)
                {
                    return guardOnePlusTaxi(isRed);
                }

                else if (routine == AutoRoutine.OnePlusCharge)
                {
                    return guardOnePlusCharge(isRed);
                }

                else if (routine == AutoRoutine.OnePickupCharge)
                {
                    return guardOnePickupCharge(isRed);
                }

                else if (routine == AutoRoutine.OnePlusPickup)
                {
                    return guardOnePickup(isRed);
                }

                else if (routine == AutoRoutine.OnePlusOne)
                {
                    return guardOnePlusOne(isRed);
                }

                else if (routine == AutoRoutine.ThreePiece)
                {
                    return guardThreePiece(isRed);
                }

                else
                {
                    return ConcurrentTask.AllTasks(
                        new ResetLevelTask(),
                        new PositionStartingTask(
                            TuningConstants.GuardEdgeStartX,
                            PathPlannerTrajectoryGenerator.getYPosition(isRed, TuningConstants.GuardEdgeY),
                            180.0,
                            true,
                            true),
                        new ResetLevelTask());
                }
            }

            return GetFillerRoutine();
        }

        return GetFillerRoutine();
    }

    /**
     * Gets an autonomous routine that does nothing
     */
    private static IControlTask GetFillerRoutine()
    {
        return new WaitTask(0.0);
    }


    private static IControlTask loadTaxi()
    {
        return SequentialTask.Sequence(
            ConcurrentTask.AllTasks(
                new ResetLevelTask(),
                new PositionStartingTask(
                    0.0,
                    0.0,
                    0.0,
                    true,
                    true)),
            new FollowPathTask("LoadTaxi", Type.RobotRelativeFromCurrentPose)
        );
    }

    // Correct Auton
    private static IControlTask loadOnePlusTaxi(boolean isRed)
    {
        return SequentialTask.Sequence(
            ConcurrentTask.AllTasks(
                new ResetLevelTask(),
                new PositionStartingTask(
                    TuningConstants.StartGridX,
                    PathPlannerTrajectoryGenerator.getYPosition(isRed, TuningConstants.StartOneGridY),
                    180.0,
                    true,
                    true)),

            new ArmLAPositionTask(
                TuningConstants.ARM_LOWER_POSITION_HIGH_CONE_DOWN,
                TuningConstants.ARM_UPPER_POSITION_HIGH_CONE_DOWN,
                true,
                IntakeState.Up,
                true),
            new IntakePositionTask(true),
            
            new WaitTask(0.2),
            new IntakeGamePieceTask(true, 1.0),

            ConcurrentTask.AllTasks(
                new FollowPathTask(isRed ? "1To20Red" : "1To20Blue", Type.Absolute),
                SequentialTask.Sequence(
                    new WaitTask(0.5),
                    new ArmLAPositionTask(
                        TuningConstants.ARM_LOWER_POSITION_STOWED,
                        TuningConstants.ARM_UPPER_POSITION_STOWED,
                        false,
                        IntakeState.Up,
                        true)
                )
            )
        );
    }

    private static IControlTask loadOnePlusCharge(boolean isRed)
    {
        return SequentialTask.Sequence(
            ConcurrentTask.AllTasks(
                new ResetLevelTask(),
                new PositionStartingTask(
                    TuningConstants.StartGridX,
                    PathPlannerTrajectoryGenerator.getYPosition(isRed, TuningConstants.StartOneGridY),
                    180.0,
                    true,
                    true)),

            new ArmLAPositionTask(
                TuningConstants.ARM_LOWER_POSITION_HIGH_CONE_DOWN,
                TuningConstants.ARM_UPPER_POSITION_HIGH_CONE_DOWN,
                true,
                IntakeState.Up,
                true),
            new IntakePositionTask(true),

            new WaitTask(0.2),
            new IntakeGamePieceTask(true, 1.0),

            ConcurrentTask.AllTasks(
                new FollowPathTask(isRed ? "1ToChargeRed" : "1ToChargeBlue", Type.Absolute),
                SequentialTask.Sequence(
                    new WaitTask(0.2),
                    new ArmLAPositionTask(
                        TuningConstants.ARM_LOWER_POSITION_STOWED,
                        TuningConstants.ARM_UPPER_POSITION_STOWED,
                        true,
                        IntakeState.Up,
                        true)
                )
            ),
            new ResetLevelTask(),
            new ChargeStationTask(true, Orientation.Backwards)
        );
    }

    private static IControlTask loadOnePickupCharge(boolean isRed)
    {
        return SequentialTask.Sequence(
            ConcurrentTask.AllTasks(
                new ResetLevelTask(),
                new PositionStartingTask(
                    TuningConstants.StartGridX,
                    PathPlannerTrajectoryGenerator.getYPosition(isRed, TuningConstants.LoadEdgeY),
                    180.0,
                    true,
                    true)),

            new ArmLAPositionTask(
                TuningConstants.ARM_LOWER_POSITION_HIGH_CONE_DOWN,
                TuningConstants.ARM_UPPER_POSITION_HIGH_CONE_DOWN,
                true,
                IntakeState.Up,
                true),
            new IntakePositionTask(true),

            new WaitTask(0.2),
            new IntakeGamePieceTask(true, 1.0),
            
            ConcurrentTask.AllTasks(
                new FollowPathTask(isRed ? "LoadEdgeTo20Red" : "LoadEdgeTo20Blue", Type.Absolute),
                SequentialTask.Sequence(
                    new ArmLAPositionTask(
                        TuningConstants.ARM_LOWER_POSITION_LOWER_INTERMIDATE,
                        TuningConstants.ARM_UPPER_POSITION_LOWER_INTERMIDATE,
                        true,
                        IntakeState.Up,
                        true),
                    new IntakePositionTask(true),
                    new ArmLAPositionTask(
                        TuningConstants.ARM_LOWER_POSITION_CUBE_GROUND_PICKUP,
                        TuningConstants.ARM_UPPER_POSITION_CUBE_GROUND_PICKUP,
                        true,
                        IntakeState.Down,
                        true)
                )
            ),

            ConcurrentTask.AllTasks(
                new FollowPathTask(isRed ? "20To14Red" : "20To14Blue", Type.Absolute),
                new IntakeGamePieceTask(true, 1.0)
            ),

            ConcurrentTask.AllTasks(
                new ArmLAPositionTask(
                    TuningConstants.ARM_LOWER_POSITION_STOWED,
                    TuningConstants.ARM_LOWER_POSITION_STOWED,
                    false,
                    IntakeState.Unchanged,
                    true),
                new FollowPathTask(isRed ? "14ToChargeStationRed" : "14ToChargeStationBlue", Type.Absolute)
            ),

            new ResetLevelTask(),
            new ChargeStationTask(true, Orientation.Forwards)
                
        );
    }

    private static IControlTask loadOnePickup(boolean isRed)
    {
        return SequentialTask.Sequence(
            ConcurrentTask.AllTasks(
                new ResetLevelTask(),
                new PositionStartingTask(
                    TuningConstants.StartGridX,
                    PathPlannerTrajectoryGenerator.getYPosition(isRed, TuningConstants.StartOneGridY),
                    180.0,
                    true,
                    true)),

            new ArmLAPositionTask(
                TuningConstants.ARM_LOWER_POSITION_HIGH_CONE_DOWN,
                TuningConstants.ARM_UPPER_POSITION_HIGH_CONE_DOWN,
                true,
                IntakeState.Up,
                true),
            new IntakePositionTask(true),

            new WaitTask(0.2),
            new IntakeGamePieceTask(true, 1.0),

            new FollowPathTask(isRed ? "1To14Red" : "1To14Blue", Type.Absolute),
                SequentialTask.Sequence(
                    new ArmLAPositionTask(
                        TuningConstants.ARM_LOWER_POSITION_STOWED,
                        TuningConstants.ARM_UPPER_POSITION_STOWED,
                        true,
                        IntakeState.Up,
                        true),
                    new WaitTask(0.5),
                    new ArmLAPositionTask(
                        TuningConstants.ARM_LOWER_POSITION_CONE_GROUND_PICKUP,
                        TuningConstants.ARM_UPPER_POSITION_CONE_GROUND_PICKUP,
                        true,
                        IntakeState.Up,
                        true)
                ),
                SequentialTask.Sequence(
                    new WaitTask(2.0),
                    new IntakeGamePieceTask(false, 3.2)
            ),

            new ArmLAPositionTask(
                TuningConstants.ARM_LOWER_POSITION_STOWED,
                TuningConstants.ARM_UPPER_POSITION_STOWED,
                false,
                IntakeState.Up,
                true)
        );
    }

    private static IControlTask loadOnePlusOne(boolean isRed)
    {
        return SequentialTask.Sequence(
            ConcurrentTask.AllTasks(
                new ResetLevelTask(),
                new PositionStartingTask(
                    TuningConstants.StartGridX,
                    PathPlannerTrajectoryGenerator.getYPosition(isRed, TuningConstants.LoadEdgeY),
                    180.0,
                    true,
                    true)),

            new ArmLAPositionTask(
                TuningConstants.ARM_LOWER_POSITION_HIGH_CONE_DOWN,
                TuningConstants.ARM_UPPER_POSITION_HIGH_CONE_DOWN,
                true,
                IntakeState.Up,
                true),
            new IntakePositionTask(true),

            new WaitTask(0.2),
            new IntakeGamePieceTask(true, 1.0),
            
            ConcurrentTask.AllTasks(
                new FollowPathTask(isRed ? "LoadEdgeTo20Red" : "LoadEdgeTo20Blue", Type.Absolute),
                SequentialTask.Sequence(
                    new ArmLAPositionTask(
                        TuningConstants.ARM_LOWER_POSITION_LOWER_INTERMIDATE,
                        TuningConstants.ARM_UPPER_POSITION_LOWER_INTERMIDATE,
                        true,
                        IntakeState.Up,
                        true),
                    new IntakePositionTask(true),
                    new ArmLAPositionTask(
                        TuningConstants.ARM_LOWER_POSITION_CUBE_GROUND_PICKUP,
                        TuningConstants.ARM_UPPER_POSITION_CUBE_GROUND_PICKUP,
                        true,
                        IntakeState.Down,
                        true)
                )
            ),

            ConcurrentTask.AllTasks(
                new FollowPathTask(isRed ? "20To14Red" : "20To14Blue", Type.Absolute),
                new IntakeGamePieceTask(true, 1.0)
            ),

            ConcurrentTask.AllTasks(
                new FollowPathTask(isRed ? "14To2Red" : "14To2Blue", Type.Absolute),
                SequentialTask.Sequence(
                    new ArmLAPositionTask(
                        TuningConstants.ARM_LOWER_POSITION_LOWER_INTERMIDATE,
                        TuningConstants.ARM_UPPER_POSITION_LOWER_INTERMIDATE,
                        true,
                        IntakeState.Up,
                        true),
                    new ArmLAPositionTask(
                        TuningConstants.ARM_LOWER_POSITION_HIGH_CUBE,
                        TuningConstants.ARM_UPPER_POSITION_HIGH_CUBE,
                        true,
                        IntakeState.Up,
                        true),
                    new IntakePositionTask(false)
                )
            ),
            
            new WaitTask(0.2),
            new IntakeGamePieceTask(false, 0.8),
            ConcurrentTask.AllTasks(
                new ArmLAPositionTask(
                    TuningConstants.ARM_LOWER_POSITION_STOWED,
                    TuningConstants.ARM_LOWER_POSITION_STOWED,
                    false,
                    IntakeState.Unchanged,
                    true),
                new FollowPathTask(isRed ? "2To27Red" : "2To27Blue", Type.Absolute)
            )
                
        );
    }

    private static IControlTask midTaxi(boolean isRed)
    {
        return SequentialTask.Sequence(
            ConcurrentTask.AllTasks(
                new ResetLevelTask(),
                new PositionStartingTask(
                    0.0,
                    0.0,
                    0.0,
                    true,
                    true)),
            new FollowPathTask("MidTaxi", Type.RobotRelativeFromCurrentPose)
        );
    }

    private static IControlTask midCharge(boolean isRed)
    {
        return SequentialTask.Sequence(
            new PositionStartingTask(
                0.0,
                0.0,
                0.0,
                true,
                true),
            new ResetLevelTask(),
            new ChargeStationTask(false, Orientation.Forwards)
        );
    }

    private static IControlTask midOnePlusCharge(boolean isRed)
    {
        return SequentialTask.Sequence(
            ConcurrentTask.AllTasks(
                new ResetLevelTask(),
                new PositionStartingTask(
                    TuningConstants.StartGridX,
                    PathPlannerTrajectoryGenerator.getYPosition(isRed, TuningConstants.StartFiveGridY),
                    180.0,
                    true,
                    true)),

            new ArmLAPositionTask(
                TuningConstants.ARM_LOWER_POSITION_HIGH_CONE_DOWN,
                TuningConstants.ARM_UPPER_POSITION_HIGH_CONE_DOWN,
                true,
                IntakeState.Up,
                true),
            new IntakePositionTask(true),
            new WaitTask(0.2),
            new IntakeGamePieceTask(true, 1.0),


            new FollowPathTask(isRed ? "5To11Red" : "5To11Blue", Type.Absolute),
            new ArmLAPositionTask(
                TuningConstants.ARM_LOWER_POSITION_STOWED,
                TuningConstants.ARM_UPPER_POSITION_STOWED,
                false,
                IntakeState.Up,
                true),
            
            new WaitTask(1.0),
            new ResetLevelTask(),
            
            new ChargeStationTask(false, Orientation.Backwards)
        );
    }

    private static IControlTask midOneTaxiCharge(boolean isRed)
    {
        return SequentialTask.Sequence(
            ConcurrentTask.AllTasks(
                new ResetLevelTask(),
                new PositionStartingTask(
                    TuningConstants.StartGridX,
                    PathPlannerTrajectoryGenerator.getYPosition(isRed, TuningConstants.StartFiveGridY),
                    180.0,
                    true,
                    true)),

            new ArmLAPositionTask(
                TuningConstants.ARM_LOWER_POSITION_HIGH_CONE_DOWN,
                TuningConstants.ARM_UPPER_POSITION_HIGH_CONE_DOWN,
                true,
                IntakeState.Up,
                true),
            new IntakePositionTask(true),

            new WaitTask(0.2),
            new IntakeGamePieceTask(true, 1.0),

            ConcurrentTask.AllTasks(
                new FollowPathTask(isRed ? "5To11Red" : "5To11Blue", Type.Absolute),
                SequentialTask.Sequence(
                        new WaitTask(1.0),
                        new ArmLAPositionTask(
                            TuningConstants.ARM_LOWER_POSITION_STOWED,
                            TuningConstants.ARM_UPPER_POSITION_STOWED,
                            true,
                            IntakeState.Up,
                            true)
                )
            ),

            new ResetLevelTask(),
            ConcurrentTask.AnyTasks(
                SequentialTask.Sequence(
                    new GoOverChargeStationTask(false, true),
                    new WaitTask(0.5)),
                new WaitTask(4.0)),
                new ResetLevelTask(),
                new ChargeStationTaskV3(true, ChargeStationTaskV3.Orientation.Backwards)
        );
    }

    private static IControlTask guardTaxi()
    {
        return SequentialTask.Sequence(
            ConcurrentTask.AllTasks(
                new ResetLevelTask(),
                new PositionStartingTask(
                    0.0,
                    0.0,
                    0.0,
                    true,
                    true)),
            new FollowPathTask("GuardTaxi", Type.RobotRelativeFromCurrentPose)
        );
    }

    private static IControlTask guardOnePlusTaxi(boolean isRed)
    {
        return SequentialTask.Sequence(
            ConcurrentTask.AllTasks(
                new ResetLevelTask(),
                new PositionStartingTask(
                    TuningConstants.StartGridX,
                    PathPlannerTrajectoryGenerator.getYPosition(isRed, TuningConstants.StartNineGridY),
                    180.0,
                    true,
                    true)),

            new ArmLAPositionTask(
                TuningConstants.ARM_LOWER_POSITION_HIGH_CONE_DOWN,
                TuningConstants.ARM_UPPER_POSITION_HIGH_CONE_DOWN,
                true,
                IntakeState.Up,
                true),
            new IntakePositionTask(true),

            new WaitTask(0.2),
            new IntakeGamePieceTask(true, 1.0),

            ConcurrentTask.AllTasks(
                new FollowPathTask(isRed ? "9To23Red" : "9To23Blue", Type.Absolute),
                SequentialTask.Sequence(
                    new WaitTask(1.0),
                    new ArmLAPositionTask(
                        TuningConstants.ARM_LOWER_POSITION_STOWED,
                        TuningConstants.ARM_UPPER_POSITION_STOWED,
                        true,
                        IntakeState.Up,
                        true)
                )
            )
        );
    }

    private static IControlTask guardOnePlusCharge(boolean isRed)
    {
        return SequentialTask.Sequence(
            ConcurrentTask.AllTasks(
                new ResetLevelTask(),
                new PositionStartingTask(
                    TuningConstants.StartGridX,
                    PathPlannerTrajectoryGenerator.getYPosition(isRed, TuningConstants.StartNineGridY),
                    180.0,
                    true,
                    true)),
            new ArmLAPositionTask(
                TuningConstants.ARM_LOWER_POSITION_HIGH_CONE_DOWN,
                TuningConstants.ARM_UPPER_POSITION_HIGH_CONE_DOWN,
                true,
                IntakeState.Up,
                true),
            new IntakePositionTask(true),

            new WaitTask(0.2),
            new IntakeGamePieceTask(true, 1.0),

            ConcurrentTask.AllTasks(
                new FollowPathTask(isRed ? "9ToChargeRed" : "9ToChargeBlue", Type.Absolute),
                new ArmLAPositionTask(
                    TuningConstants.ARM_LOWER_POSITION_STOWED,
                    TuningConstants.ARM_UPPER_POSITION_STOWED,
                    true,
                    IntakeState.Up,
                    true)
            ),
            new ResetLevelTask(),
            new ChargeStationTask(true, Orientation.Backwards)
        );
    }

    private static IControlTask guardOnePickupCharge(boolean isRed)
    {
        return SequentialTask.Sequence(
            ConcurrentTask.AllTasks(
                new ResetLevelTask(),
                new PositionStartingTask(
                    TuningConstants.StartGridX,
                    PathPlannerTrajectoryGenerator.getYPosition(isRed, TuningConstants.GuardEdgeY),
                    180.0,
                    true,
                    true)),
            new ArmLAPositionTask(
                TuningConstants.ARM_LOWER_POSITION_HIGH_CONE_DOWN,
                TuningConstants.ARM_UPPER_POSITION_HIGH_CONE_DOWN,
                true,
                IntakeState.Up,
                true),
            new IntakePositionTask(true),

            new WaitTask(0.2),
            new IntakeGamePieceTask(true, 1.0),

            ConcurrentTask.AllTasks(
                new FollowPathTask(isRed ? "GuardEdgeTo23Red" : "GuardEdgeTo23Blue", Type.Absolute),
                SequentialTask.Sequence(
                    new ArmLAPositionTask(
                        TuningConstants.ARM_LOWER_POSITION_LOWER_INTERMIDATE,
                        TuningConstants.ARM_UPPER_POSITION_LOWER_INTERMIDATE,
                        true,
                        IntakeState.Up,
                        true),
                    new ArmLAPositionTask(
                        TuningConstants.ARM_LOWER_POSITION_CUBE_GROUND_PICKUP,
                        TuningConstants.ARM_UPPER_POSITION_CUBE_GROUND_PICKUP,
                        true,
                        IntakeState.Down,
                        true),
                    new IntakePositionTask(true)
                )
            ),

            ConcurrentTask.AllTasks(
                new FollowPathTask(isRed ? "23To17Red" : "23To17Blue", Type.Absolute),
                new IntakeGamePieceTask(true, 1.0)
            ),

            ConcurrentTask.AllTasks(
                new ArmLAPositionTask(
                    TuningConstants.ARM_LOWER_POSITION_STOWED,
                    TuningConstants.ARM_LOWER_POSITION_STOWED,
                    false,
                    IntakeState.Unchanged,
                    true),
                new FollowPathTask(isRed ? "17ToChargeStationRed" : "17ToChargeStationBlue", Type.Absolute)
            ),

            new ResetLevelTask(),
            new ChargeStationTask(true, Orientation.Forwards)
        );
    }

    private static IControlTask guardOnePickup(boolean isRed)
    {
        return SequentialTask.Sequence(
            ConcurrentTask.AllTasks(
                new ResetLevelTask(),
                new PositionStartingTask(
                    TuningConstants.StartGridX,
                    PathPlannerTrajectoryGenerator.getYPosition(isRed, TuningConstants.StartNineGridY),
                    180.0,
                    true,
                    true)),
            new ArmLAPositionTask(
                TuningConstants.ARM_LOWER_POSITION_HIGH_CONE_DOWN,
                TuningConstants.ARM_UPPER_POSITION_HIGH_CONE_DOWN,
                true,
                IntakeState.Up,
                true),
            new IntakePositionTask(true),

            new WaitTask(0.2),
            new IntakeGamePieceTask(true, 1.0),

            ConcurrentTask.AllTasks(
                new FollowPathTask(isRed ? "9To17Red" : "9To17Blue", Type.Absolute),
                SequentialTask.Sequence(
                    new ArmLAPositionTask(
                        TuningConstants.ARM_LOWER_POSITION_STOWED,
                        TuningConstants.ARM_UPPER_POSITION_STOWED,
                        true,
                        IntakeState.Up,
                        true),
                    new WaitTask(0.5),
                    new ArmLAPositionTask(
                        TuningConstants.ARM_LOWER_POSITION_CONE_GROUND_PICKUP,
                        TuningConstants.ARM_UPPER_POSITION_CONE_GROUND_PICKUP,
                        true,
                        IntakeState.Up,
                        true)
                ),
                SequentialTask.Sequence(
                    new WaitTask(2.0),
                    new IntakeGamePieceTask(false, 5.5)
                )
            ),

            new ArmLAPositionTask(
                TuningConstants.ARM_LOWER_POSITION_STOWED,
                TuningConstants.ARM_UPPER_POSITION_STOWED,
                false,
                IntakeState.Up,
                true)
        );
    }

    private static IControlTask guardOnePlusOne(boolean isRed)
    {
        return SequentialTask.Sequence(
            ConcurrentTask.AllTasks(
                new ResetLevelTask(),
                new PositionStartingTask(
                    TuningConstants.StartGridX,
                    PathPlannerTrajectoryGenerator.getYPosition(isRed, TuningConstants.GuardEdgeY),
                    180.0,
                    true,
                    true)),
            new ArmLAPositionTask(
                TuningConstants.ARM_LOWER_POSITION_HIGH_CONE_DOWN,
                TuningConstants.ARM_UPPER_POSITION_HIGH_CONE_DOWN,
                true,
                IntakeState.Up,
                true),
            new IntakePositionTask(true),

            new WaitTask(0.2),
            new IntakeGamePieceTask(true, 1.0),

            ConcurrentTask.AllTasks(
                new FollowPathTask(isRed ? "GuardEdgeTo23Red" : "GuardEdgeTo23Blue", Type.Absolute),
                SequentialTask.Sequence(
                    new ArmLAPositionTask(
                        TuningConstants.ARM_LOWER_POSITION_LOWER_INTERMIDATE,
                        TuningConstants.ARM_UPPER_POSITION_LOWER_INTERMIDATE,
                        true,
                        IntakeState.Up,
                        true),
                    new ArmLAPositionTask(
                        TuningConstants.ARM_LOWER_POSITION_CUBE_GROUND_PICKUP,
                        TuningConstants.ARM_UPPER_POSITION_CUBE_GROUND_PICKUP,
                        true,
                        IntakeState.Down,
                        true),
                    new IntakePositionTask(true)
                )
            ),

            ConcurrentTask.AllTasks(
                new FollowPathTask(isRed ? "23To17Red" : "23To17Blue", Type.Absolute),
                new IntakeGamePieceTask(true, 1.3)
            ),

            ConcurrentTask.AllTasks(
                SequentialTask.Sequence(
                    new ArmLAPositionTask(
                        TuningConstants.ARM_LOWER_POSITION_LOWER_INTERMIDATE,
                        TuningConstants.ARM_UPPER_POSITION_LOWER_INTERMIDATE,
                        true,
                        IntakeState.Up,
                        true),
                    new ArmLAPositionTask(
                        TuningConstants.ARM_LOWER_POSITION_HIGH_CUBE,
                        TuningConstants.ARM_UPPER_POSITION_HIGH_CUBE,
                        true,
                        IntakeState.Up,
                        true)
                    ),

                new FollowPathTask(isRed ? "17To8Red" : "17To8Blue", Type.Absolute)
            ),

            new WaitTask(0.2),
            new IntakeGamePieceTask(false, 0.8),
            ConcurrentTask.AllTasks(
                SequentialTask.Sequence(
                    new ArmLAPositionTask(
                        TuningConstants.ARM_LOWER_POSITION_STOWED,
                        TuningConstants.ARM_UPPER_POSITION_STOWED,
                        true,
                        IntakeState.Up,
                        true),
                    new WaitTask(1.0),
                    new ArmLAPositionTask(
                        TuningConstants.ARM_LOWER_POSITION_CONE_GROUND_PICKUP,
                        TuningConstants.ARM_UPPER_POSITION_CONE_GROUND_PICKUP,
                        true,
                        IntakeState.Down,
                        true),
                    new IntakeGamePieceTask(false, 1.5)
                ),
                new FollowPathTask(isRed ? "8To16Red" : "8To16Blue", Type.Absolute)
            )
        );
    }

    private static IControlTask guardThreePiece(boolean isRed)
    {
        return SequentialTask.Sequence(
            ConcurrentTask.AllTasks(
                new ResetLevelTask(),
                new PositionStartingTask(
                    TuningConstants.StartGridX,
                    PathPlannerTrajectoryGenerator.getYPosition(isRed, TuningConstants.GuardEdgeY),
                    180.0,
                    true,
                    true)),
            new ArmLAPositionTask(
                TuningConstants.ARM_LOWER_POSITION_MIDDLE_CONE,
                TuningConstants.ARM_UPPER_POSITION_MIDDLE_CONE,
                true,
                IntakeState.Up,
                true),
            new IntakePositionTask(true),

            new WaitTask(0.2),
            new IntakeGamePieceTask(true, 1.0),

            ConcurrentTask.AllTasks(
                new FollowPathTask(isRed ? "GuardEdgeTo17Red" : "GuardEdgeTo17Blue", Type.Absolute),
                SequentialTask.Sequence(
                    new ArmLAPositionTask(
                        TuningConstants.ARM_LOWER_POSITION_LOWER_INTERMIDATE,
                        TuningConstants.ARM_UPPER_POSITION_LOWER_INTERMIDATE,
                        true,
                        IntakeState.Up,
                        true),
                    new WaitTask(1.0),
                    new ArmLAPositionTask(
                        TuningConstants.ARM_LOWER_POSITION_CUBE_GROUND_PICKUP,
                        TuningConstants.ARM_UPPER_POSITION_CUBE_GROUND_PICKUP,
                        true,
                        IntakeState.Down,
                        true),
                    new IntakePositionTask(true),
                    new IntakeGamePieceTask(true, 1.3)
                )
            ),

            ConcurrentTask.AllTasks(
                SequentialTask.Sequence(
                    new ArmLAPositionTask(
                        TuningConstants.ARM_LOWER_POSITION_LOWER_INTERMIDATE,
                        TuningConstants.ARM_UPPER_POSITION_LOWER_INTERMIDATE,
                        true,
                        IntakeState.Up,
                        true),
                    new ArmLAPositionTask(
                        TuningConstants.ARM_LOWER_POSITION_MIDDLE_CUBE,
                        TuningConstants.ARM_UPPER_POSITION_MIDDLE_CUBE,
                        true,
                        IntakeState.Up,
                        true)
                    ),

                new FollowPathTask(isRed ? "17To8Red" : "17To8Blue", Type.Absolute)
            ),

            new WaitTask(0.2),
            new IntakeGamePieceTask(false, 0.8),

            ConcurrentTask.AllTasks(
                new FollowPathTask(isRed ? "8To16Red" : "8To16Blue", Type.Absolute),
                SequentialTask.Sequence(
                    new ArmLAPositionTask(
                        TuningConstants.ARM_LOWER_POSITION_LOWER_INTERMIDATE,
                        TuningConstants.ARM_UPPER_POSITION_LOWER_INTERMIDATE,
                        true,
                        IntakeState.Up,
                        true),
                    new WaitTask(1.0),
                    new ArmLAPositionTask(
                        TuningConstants.ARM_LOWER_POSITION_CUBE_GROUND_PICKUP,
                        TuningConstants.ARM_UPPER_POSITION_CUBE_GROUND_PICKUP,
                        true,
                        IntakeState.Down,
                        true),
                    new IntakePositionTask(true),
                    new IntakeGamePieceTask(true, 1.3)
                )
            ),

            ConcurrentTask.AllTasks(
                SequentialTask.Sequence(
                    new ArmLAPositionTask(
                        TuningConstants.ARM_LOWER_POSITION_LOWER_INTERMIDATE,
                        TuningConstants.ARM_UPPER_POSITION_LOWER_INTERMIDATE,
                        true,
                        IntakeState.Up,
                        true),
                    new WaitTask(1.0),
                    new IntakePositionTask(false),
                    new ArmLAPositionTask(
                        TuningConstants.ARM_LOWER_POSITION_HIGH_CUBE,
                        TuningConstants.ARM_UPPER_POSITION_HIGH_CUBE,
                        true,
                        IntakeState.Up,
                        true),
                    
                    new WaitTask(0.5),
                    new IntakeGamePieceTask(false, 1.0)),

                new FollowPathTask(isRed ? "16To8Red" : "16To8Blue", Type.Absolute)
            )
        );
    }
}








































































































































/*
                                      .
                                    .;+;+
                                    .+;;'   `,+'.
                                    ;';;+:..`` :+'+
                                    ,'+`    .+;;;;;+
                                     ;,,, .+;;;;;'+++;
                                     ;' `+;;;;;#+'+'+''#:.
                                     '`+';;;'+;+;+++'''+'.
                                     #';;;;#';+'+'''+''+'
                                     ;;;;#;,+;;+;;;'''''':
                                     ';'++'.`+;;'';;''+'',
                                     :#'#+'``.'+++'#++'':`
                                      `';++##```##+.''.##
                                      +++#   #`#  `++++
                                      +'#+ # :#: # ##'+
                                      `#+#   +`+   #'#`
                                       :,.+,+,`:+,+..,
                                       `,:```,`,`.`;,
                                        :+.;``.``;.#;
                                        .'``'+'+'``'.
                                         ,````````..
                                          :```````:
                                          +``.:,``'
                                          :```````:
                                           +`````+
                                            ';+##
                                            '```'
                                           `'```'`
                                         .+''''''''
                                        +;;;;;;;;''#
                                       :       `   `:
                                      `,            '
                                      +              '
                                     ,;';,``.``.,,,:;#
                                     +;;;;;;;;;;;;;;;'
                                    ,';;;;;;;;;;;;;;;',
                                    +:;;;;;;';;;;;;;;;+
                                   `.   .:,;+;;:::;.``,
                                   :`       #,       `.`
                                   +       # ;        .;
                                  .;;,`    ,         `,+
                                  +;;;;;;''';;;;;;;';;';
                                  +;;;;;;;';;;;;;;;;;'';;
                                 `';;;;;;';;;;;;;;;;;';;+
                                 + `:;;;;+;;;;;;;;';'''::
                                 '     `:  ```````    ,  ,
                                :       '             ;  +
                                '`     ..             ,  ,
                               ,;;;;;..+,`        ```.':;',
                               +;;;;;;'+;;;;;;;;;;;;;;+;;;+
                               ';;;;;;++;;;;;;;;;;;;;;';;;+
                              `.:';;;;;#;;;;;;;;;;;;;;';;;;`
                              ;    `,; ',:;;';;';;;;;:;``  +
                              +      ; ;              ;    `
                              ;      : +              '    `;
                              ';:`` `` '              :`,:;;+
                             `';;;;'+  +,..```````..:;#;;;;;;.
                             `;;;;;;+  +;;;;;;;;;;;;;':';;;;;#
                             .;;;;;;+  ';;;;;;;;;;;;;;,';;;;` .
                             : `.;;'+  +;;;;;;;;;;;;;','.`    +
                             '      ;  +.,,;:;:;;;,..`: ,     ``
                             +      ,  '              : ;   .;'+
                             +.`   ``  +              ;  ;:;;;;':
                             ';;;';;`  +             .'  ;;;;;;;+
                             ';;;;;'   :+++#++##+#+''',   +;;;;.`.
                             +;;;;;'   +;;::;;;+:+;;'',   ,;;.   +
                            ``:;;;;+   +;;:;;;:+;+;;++;    +     .`
                             `   ``'   +;;;;;;;+;+;;'+;     ,   ;#,
                            .      ;   ';;;;;;;;;;;;++'     + .+``.;
                            ``     ;   ';;;;;;+;';;;'+'      #`````:,
                             +++;,:.   ':;''++;:';:;'';      +``````,`
                             ,```,+    +;;';:;;+;;;;'';      +``````,+
                            .``````:   ;:;;++';;;;;;';,      ,``:#``+`.
                            ,``````'   `';;;;:;;;;;;+;`     '+``+:'`..'
                            ,``````'    +;;;;;;;;;;;''     ;:'``#;;.`++
                            ```````;    `;:;;;;;;;;;;#     ':'``++:+`+;
                            ```'`.`;     +;;;;;;;;;;;+    :::#``' +#`';
                            ,``'`:`#     `';;;;;;;;;;+    +:'.`,. ++`;;
                            +`.``+`'     :#;;;;;;;;;;;`   +:# ,`  +;`.'
                           ,.`+`.:.      ##;;;;;;;;;;;'   ,'`     ;:+#
                           '`;.`+`#      ##+;;;;;;;;;;+          ,::;
                           ,+,`:``,     :###;;;;;;;;;:'          +:;`
                            '`,,`+      ';##';;;;;;;;;;.         +:#
                             '+.+       +;;##;;;;;;;;;;'         ;:;
                               `       :;;;+#;;;;;;;;;;+        ;::`
                                       +;;;;#+;;;;;;;;;;        +:'
                                       ';;;;+#;;;;;;;;;;.       ;:'
                                      ,;;;;;;#;;;;;;;;;;+      +::.
                                      +;;;;;;'';;;;;;;;;'      +:+
                                     `;;;;;;;;#;;;;;;;;;;`    `;:+
                                     ,;;;;;;;;+;;;;;;;;;;+    ':;,
                                     +;;;;;;;;;+;;;;;;;;;'    +:+
                                    .;;;;;;;;;+,;;;;;;;;;;`   ;;+
                                    ';;;;;;;;;, ';;;;;;:;;,  +;:,
                                    ';;;;;;;;'  +;;;;;;;;;'  +:+
                                   ;;;;;;;;;;+  ,;;;;;;;;;+  ;:'
                                   +;;;;;;;;;    ';;;;;;;;;`;:;`
                                   ;;;;;;;;;+    +;;;;;;;;;+#:+
                                  ';;;;;;;;;:    ;;;;;;;;;;';:'
                                 `';;;;;;;:'      ';;;;;;;;;;:.
                                 .;;;;;;;;;+      +;;;;;;;;;'+
                                 +;;;;;;;;;       ';;;;;;;;;#+
                                `;;;;;;;;;+       `;;;;;;;;;;`
                                +;;;;;;;;;.        +;;;;;;;;;`
                                ';;;;;;;:'         ;;;;;;;;;;;
                               :;;;;;;;;;:         `;;;;;;;;;+
                               +;;;;;;;;;           ';;;;;;;;;`
                               ;;;;;;;;;+           ';;;;;;;;;:
                              ';;;;;;;;;;           ,;;;;;;;;;+
                              ':;;;;;;;'             +;;;;;;;;;
                             .;:;;;;;;;'             +;;;;;;;;;:
                             +;;;;;;;;;`             .;;;;;;;;;+
                            `;;;;;;;;;+               ;:;;;;;;;;`
                            ;;;;;;;;;;.               +;;;;;;;::.
                            ';;;;;;;;'`               :;;;;;;;;:+
                           :;;;;;;;;:'                ';;;;;;;;;'
                           ';;;;;;;;'`                +#;;;;;;;;;`
                          `;;;;;;;;;+                 '';;;;;;;;;+
                          +;;;;;;;;;.                '::;;;;;;;;;+
                          ;;;;;;;;;+                 #:'';;;;;;;;;`
                         .#;;;;;;;;'                `;:+;;;;;;;;;;;
                         ':'';;;;;;                 '::.,;;;;;;;;;+
                        +::::+';;;+                 ':'  +:;;;;;;;;`
                       `;;;::::;#+:                `;:+  +;;;;;;;:;;      '#+,
                       +#::::::::;'`               +:;,  `;;;;:;;'#';;;;;::;:'`
                      ';:''::::::::#`              +:'    ';:;;+'::;;:;::::::''
                      #+::;+':::::::'.            .:;+    '''+;::;:;:::;:::;':'
                    `';+';;:;'';:::::':    '      +::.     +:::::::::::::;#;:#
                    :+;#'.''##;#;:;;:::'+  #     `+;'      ;:;::::::::;'+;:'+
                   '#;+". ` `+:;+:;::;::+'#+     +:;#     ';:::;:+#+';:::+.
                   ';#''      ,+::+#';::;+'#+    ';::      #:;;'+';'''++:`
                                '':::;'''#+     ,:;;`      #';:;;:+
                                 `:'++;;':       :++       .;;:;;#,
                                       `                    '':``


*/
