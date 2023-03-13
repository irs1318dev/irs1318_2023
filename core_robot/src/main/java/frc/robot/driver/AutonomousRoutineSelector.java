package frc.robot.driver;

import com.google.inject.Inject;
import com.google.inject.Singleton;

import frc.robot.LoggingKey;
import frc.robot.TuningConstants;
import frc.robot.common.LoggingManager;
import frc.robot.common.robotprovider.*;
import frc.robot.driver.SmartDashboardSelectionManager.AutoRoutine;
import frc.robot.driver.SmartDashboardSelectionManager.StartPosition;
import frc.robot.driver.common.*;
import frc.robot.driver.controltasks.*;
import frc.robot.driver.controltasks.ArmMMPositionTask.IntakeState;
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
            return AutonomousRoutineSelector.GetFillerRoutine();
        }

        if (mode != RobotMode.Autonomous)
        {
            return null;
        }

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
                    TuningConstants.LoadEdgeStartX,
                    PathPlannerTrajectoryGenerator.getYPosition(isRed, TuningConstants.LoadEdgeY),
                    180.0,
                    true,
                    true)),
            new ArmMMPositionTask(
                TuningConstants.ARM_LOWER_POSITION_HIGH_CONE,
                TuningConstants.ARM_UPPER_POSITION_HIGH_CONE,
                true,
                IntakeState.Down),
            new FollowPathTask(isRed ? "LoadStartTo1Red" : "LoadStartTo1Blue", Type.Absolute),
            
            new WaitTask(0.2),
            new IntakeGamePieceTask(true, 1.0),

            ConcurrentTask.AllTasks(
                new FollowPathTask(isRed ? "1To20Red" : "1To20Blue", Type.Absolute),
                SequentialTask.Sequence(
                    new WaitTask(0.7),
                    new ArmMMPositionTask(
                        TuningConstants.ARM_LOWER_POSITION_STOWED,
                        TuningConstants.ARM_UPPER_POSITION_STOWED,
                        IntakeState.Up)
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
                    TuningConstants.LoadEdgeStartX,
                    PathPlannerTrajectoryGenerator.getYPosition(isRed, TuningConstants.LoadEdgeY),
                    180.0,
                    true,
                    true)),
            new ArmMMPositionTask(
                TuningConstants.ARM_LOWER_POSITION_HIGH_CONE,
                TuningConstants.ARM_UPPER_POSITION_HIGH_CONE,
                true,
                IntakeState.Down),
            new FollowPathTask(isRed ? "LoadStartTo1Red" : "LoadStartTo1Blue", Type.Absolute),
            
            new WaitTask(0.2),
            new IntakeGamePieceTask(true, 1.0),

            ConcurrentTask.AllTasks(
                new FollowPathTask(isRed ? "1ToChargeRed" : "1ToChargeBlue", Type.Absolute),
                SequentialTask.Sequence(
                    new WaitTask(0.7),
                    new ArmMMPositionTask(
                        TuningConstants.ARM_LOWER_POSITION_STOWED,
                        TuningConstants.ARM_UPPER_POSITION_STOWED,
                        true,
                        IntakeState.Up)
                )
            ),
            new ResetLevelTask(),
            new ChargeStationTaskv2(true, false)
        );
    }

    private static IControlTask loadOnePickupCharge(boolean isRed)
    {
        return SequentialTask.Sequence(
            ConcurrentTask.AllTasks(
                new ResetLevelTask(),
                new PositionStartingTask(
                    TuningConstants.LoadEdgeStartX,
                    PathPlannerTrajectoryGenerator.getYPosition(isRed, TuningConstants.LoadEdgeY),
                    180.0,
                    true,
                    true)),
            new ArmMMPositionTask(
                TuningConstants.ARM_LOWER_POSITION_HIGH_CONE,
                TuningConstants.ARM_UPPER_POSITION_HIGH_CONE,
                true,
                IntakeState.Down),
            new FollowPathTask(isRed ? "LoadStartTo1Red" : "LoadStartTo1Blue", Type.Absolute),
            
            new WaitTask(0.2),
            new IntakeGamePieceTask(true, 1.0),

            ConcurrentTask.AllTasks(
                new FollowPathTask(isRed ? "1To14Red" : "1To14Blue", Type.Absolute),

                SequentialTask.Sequence(
                    new WaitTask(0.7),
                    new ArmMMPositionTask(
                        TuningConstants.ARM_LOWER_POSITION_GROUND_PICKUP,
                        TuningConstants.ARM_UPPER_POSITION_GROUND_PICKUP,
                        true,
                        IntakeState.Up)
                ),

                new IntakeGamePieceTask(true, 5.0)
            ),

            ConcurrentTask.AllTasks(
                new FollowPathTask(isRed ? "14ToChargeStationRed" : "14ToChargeStationBlue", Type.Absolute),
                new ArmMMPositionTask(
                    TuningConstants.ARM_LOWER_POSITION_STOWED,
                    TuningConstants.ARM_UPPER_POSITION_STOWED,
                    false,
                    IntakeState.Up)
            ),

            new ResetLevelTask(),
            new ChargeStationTaskv2(true, false)
        );
    }

    private static IControlTask loadOnePickup(boolean isRed)
    {
        return SequentialTask.Sequence(
            ConcurrentTask.AllTasks(
                new ResetLevelTask(),
                new PositionStartingTask(
                    TuningConstants.LoadEdgeStartX,
                    PathPlannerTrajectoryGenerator.getYPosition(isRed, TuningConstants.LoadEdgeY),
                    180.0,
                    true,
                    true)),
            new ArmMMPositionTask(
                TuningConstants.ARM_LOWER_POSITION_HIGH_CONE,
                TuningConstants.ARM_UPPER_POSITION_HIGH_CONE,
                true,
                IntakeState.Down),
            new FollowPathTask(isRed ? "LoadStartTo1Red" : "LoadStartTo1Blue", Type.Absolute),
            
            new WaitTask(0.2),
            new IntakeGamePieceTask(true, 1.0),

            ConcurrentTask.AllTasks(
                new FollowPathTask(isRed ? "1To14Red" : "1To14Blue", Type.Absolute),

                SequentialTask.Sequence(
                    new WaitTask(0.7),
                    new ArmMMPositionTask(
                        TuningConstants.ARM_LOWER_POSITION_GROUND_PICKUP,
                        TuningConstants.ARM_UPPER_POSITION_GROUND_PICKUP,
                        true,
                        IntakeState.Up)
                ),

                new IntakeGamePieceTask(true, 5.0)
            ),

            new ArmMMPositionTask(
                TuningConstants.ARM_LOWER_POSITION_STOWED,
                TuningConstants.ARM_UPPER_POSITION_STOWED,
                IntakeState.Up)
        );
    }

    private static IControlTask loadOnePlusOne(boolean isRed)
    {
        return SequentialTask.Sequence(
            ConcurrentTask.AllTasks(
                new ResetLevelTask(),
                new PositionStartingTask(
                    TuningConstants.LoadEdgeStartX,
                    PathPlannerTrajectoryGenerator.getYPosition(isRed, TuningConstants.LoadEdgeY),
                    180.0,
                    true,
                    true)),
            new ArmMMPositionTask(
                TuningConstants.ARM_LOWER_POSITION_HIGH_CONE,
                TuningConstants.ARM_UPPER_POSITION_HIGH_CONE,
                true,
                IntakeState.Down),
            new FollowPathTask(isRed ? "LoadStartTo1Red" : "LoadStartTo1Blue", Type.Absolute),
            
            new WaitTask(0.2),
            new IntakeGamePieceTask(true, 1.0),

            ConcurrentTask.AllTasks(
                new FollowPathTask(isRed ? "1To14Red" : "1To14Blue", Type.Absolute),

                SequentialTask.Sequence(
                    new WaitTask(0.7),
                    new ArmMMPositionTask(
                        TuningConstants.ARM_LOWER_POSITION_GROUND_PICKUP,
                        TuningConstants.ARM_UPPER_POSITION_GROUND_PICKUP,
                        true,
                        IntakeState.Up)
                ),

                new IntakeGamePieceTask(true, 5.0)
            ),

            ConcurrentTask.AllTasks(
                new FollowPathTask(isRed ? "14To2Red" : "14To2Blue", Type.Absolute),
                new ArmMMPositionTask(
                    TuningConstants.ARM_LOWER_POSITION_HIGH_CUBE,
                    TuningConstants.ARM_UPPER_POSITION_HIGH_CUBE,
                    true,
                    IntakeState.Down)
            ),
            
            new WaitTask(0.2),
            new IntakeGamePieceTask(false, 5.0)
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
            new FollowPathTask(isRed ? "MidTaxi" : "MidTaxi", Type.RobotRelativeFromCurrentPose)
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
            new ChargeStationTaskv2(false, false)
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

            ConcurrentTask.AllTasks(
                new FollowPathTask(isRed ? "5To11Red" : "5To11Blue", Type.Absolute),
                SequentialTask.Sequence(
                    new WaitTask(0.5),
                    new ArmMMPositionTask(
                        TuningConstants.ARM_LOWER_POSITION_HIGH_CUBE,
                        TuningConstants.ARM_UPPER_POSITION_HIGH_CUBE,
                        false,
                        IntakeState.Up)
                )
            ),
            new WaitTask(0.5),
            new FollowPathTask(isRed ? "11To5Red" : "11To5Blue", Type.Absolute),

            new IntakeGamePieceTask(false, 1.0),
            new FollowPathTask(isRed ? "5To11Red" : "5To11Blue", Type.Absolute),
            new ArmMMPositionTask(
                TuningConstants.ARM_LOWER_POSITION_STOWED,
                TuningConstants.ARM_UPPER_POSITION_STOWED,
                false,
                IntakeState.Up),
            
            new WaitTask(1.0),
            new ResetLevelTask(),
            
            new ChargeStationTaskv2(false, true)
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
                    TuningConstants.GuardEdgeStartX,
                    PathPlannerTrajectoryGenerator.getYPosition(isRed, TuningConstants.GuardEdgeY),
                    180.0,
                    true,
                    true)),
            new ArmMMPositionTask(
                TuningConstants.ARM_LOWER_POSITION_HIGH_CONE,
                TuningConstants.ARM_UPPER_POSITION_HIGH_CONE,
                true,
                IntakeState.Down),
            new FollowPathTask(isRed ? "GuardStartTo9Red" : "GuardStartTo9Blue", Type.Absolute),

            new WaitTask(0.2),
            new IntakeGamePieceTask(true, 1.0),

            ConcurrentTask.AllTasks(
                new FollowPathTask(isRed ? "9To23Red" : "9To23Blue", Type.Absolute),
                SequentialTask.Sequence(
                    new WaitTask(0.7),
                    new ArmMMPositionTask(
                        TuningConstants.ARM_LOWER_POSITION_STOWED,
                        TuningConstants.ARM_UPPER_POSITION_STOWED,
                        true,
                        IntakeState.Up)
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
                    TuningConstants.GuardEdgeStartX,
                    PathPlannerTrajectoryGenerator.getYPosition(isRed, TuningConstants.GuardEdgeY),
                    180.0,
                    true,
                    true)),
            new ArmMMPositionTask(
                TuningConstants.ARM_LOWER_POSITION_HIGH_CONE,
                TuningConstants.ARM_UPPER_POSITION_HIGH_CONE,
                true,
                IntakeState.Down),
            new FollowPathTask(isRed ? "GuardStartTo9Red" : "GuardStartTo9Blue", Type.Absolute),

            new WaitTask(0.2),
            new IntakeGamePieceTask(true, 1.0),

            ConcurrentTask.AllTasks(
                new FollowPathTask(isRed ? "9ToChargeRed" : "9ToChargeBlue", Type.Absolute),
                SequentialTask.Sequence(
                    new WaitTask(0.7),
                    new ArmMMPositionTask(
                        TuningConstants.ARM_LOWER_POSITION_STOWED,
                        TuningConstants.ARM_UPPER_POSITION_STOWED,
                        true,
                        IntakeState.Up)
                )
            ),
            new ResetLevelTask(),
            new ChargeStationTaskv2(true, false)
        );
    }

    private static IControlTask guardOnePickupCharge(boolean isRed)
    {
        return SequentialTask.Sequence(
            ConcurrentTask.AllTasks(
                new ResetLevelTask(),
                new PositionStartingTask(
                    TuningConstants.GuardEdgeStartX,
                    PathPlannerTrajectoryGenerator.getYPosition(isRed, TuningConstants.GuardEdgeY),
                    180.0,
                    true,
                    true)),
            new ArmMMPositionTask(
                TuningConstants.ARM_LOWER_POSITION_HIGH_CONE,
                TuningConstants.ARM_UPPER_POSITION_HIGH_CONE,
                true,
                IntakeState.Down),
            new FollowPathTask(isRed ? "GuardStartTo9Red" : "GuardStartTo9Blue", Type.Absolute),

            new WaitTask(0.2),
            new IntakeGamePieceTask(true, 1.0),

            ConcurrentTask.AllTasks(
                new FollowPathTask(isRed ? "9To17Red" : "9To17Blue", Type.Absolute),

                SequentialTask.Sequence(
                    new WaitTask(0.7),
                    new ArmMMPositionTask(
                        TuningConstants.ARM_LOWER_POSITION_GROUND_PICKUP,
                        TuningConstants.ARM_UPPER_POSITION_GROUND_PICKUP,
                        true,
                        IntakeState.Up)
                ),

                new IntakeGamePieceTask(true, 5.0)
            ),
            
            ConcurrentTask.AllTasks(
                new FollowPathTask(isRed ? "17ToChargeRed" : "17ToChargeBlue", Type.Absolute),
                new ArmMMPositionTask(
                    TuningConstants.ARM_LOWER_POSITION_STOWED,
                    TuningConstants.ARM_UPPER_POSITION_STOWED,
                    true,
                    IntakeState.Up)
            ),

            new ResetLevelTask(),
            new ChargeStationTaskv2(true, false)
        );
    }

    private static IControlTask guardOnePickup(boolean isRed)
    {
        return SequentialTask.Sequence(
            ConcurrentTask.AllTasks(
                new ResetLevelTask(),
                new PositionStartingTask(
                    TuningConstants.GuardEdgeStartX,
                    PathPlannerTrajectoryGenerator.getYPosition(isRed, TuningConstants.GuardEdgeY),
                    180.0,
                    true,
                    true)),
            new ArmMMPositionTask(
                TuningConstants.ARM_LOWER_POSITION_HIGH_CONE,
                TuningConstants.ARM_UPPER_POSITION_HIGH_CONE,
                true,
                IntakeState.Down),
            new FollowPathTask(isRed ? "GuardStartTo9Red" : "GuardStartTo9Blue", Type.Absolute),

            new WaitTask(0.2),
            new IntakeGamePieceTask(true, 1.0),

            ConcurrentTask.AllTasks(
                new FollowPathTask(isRed ? "9To17Red" : "9To17Blue", Type.Absolute),
                SequentialTask.Sequence(
                    new WaitTask(0.7),
                    new ArmMMPositionTask(
                        TuningConstants.ARM_LOWER_POSITION_GROUND_PICKUP,
                        TuningConstants.ARM_UPPER_POSITION_GROUND_PICKUP,
                        true,
                        IntakeState.Up)
                ),

                new IntakeGamePieceTask(true, 5.0)
            ),
            
            ConcurrentTask.AllTasks(
                new FollowPathTask(isRed ? "9To17Red" : "9To17Blue", Type.Absolute),
                
                SequentialTask.Sequence(
                    new WaitTask(0.7),
                    new ArmMMPositionTask(
                        TuningConstants.ARM_LOWER_POSITION_MIDDLE_CUBE,
                        TuningConstants.ARM_UPPER_POSITION_MIDDLE_CUBE,
                        true,
                        IntakeState.Up)
                ),

                new IntakeGamePieceTask(false, 5.0)
            ),

            new ArmMMPositionTask(
                TuningConstants.ARM_LOWER_POSITION_STOWED,
                TuningConstants.ARM_UPPER_POSITION_STOWED,
                IntakeState.Up)
        );
    }

    private static IControlTask guardOnePlusOne(boolean isRed)
    {
        return SequentialTask.Sequence(
            ConcurrentTask.AllTasks(
                new ResetLevelTask(),
                new PositionStartingTask(
                    TuningConstants.GuardEdgeStartX,
                    PathPlannerTrajectoryGenerator.getYPosition(isRed, TuningConstants.GuardEdgeY),
                    180.0,
                    true,
                    true)),
            new ArmMMPositionTask(
                TuningConstants.ARM_LOWER_POSITION_HIGH_CONE,
                TuningConstants.ARM_UPPER_POSITION_HIGH_CONE,
                true,
                IntakeState.Down),
            new FollowPathTask(isRed ? "GuardStartTo9Red" : "GuardStartTo9Blue", Type.Absolute),

            new WaitTask(0.2),
            new IntakeGamePieceTask(true, 1.0),

            ConcurrentTask.AllTasks(
                new FollowPathTask(isRed ? "9To17Red" : "9To17Blue", Type.Absolute),
                SequentialTask.Sequence(
                    new WaitTask(0.7),
                    new ArmMMPositionTask(
                        TuningConstants.ARM_LOWER_POSITION_GROUND_PICKUP,
                        TuningConstants.ARM_UPPER_POSITION_GROUND_PICKUP,
                        true,
                        IntakeState.Up)
                ),

                new IntakeGamePieceTask(true, 5.0)
            ),

            ConcurrentTask.AllTasks(
                new ArmMMPositionTask(
                    TuningConstants.ARM_LOWER_POSITION_HIGH_CUBE,
                    TuningConstants.ARM_UPPER_POSITION_HIGH_CUBE,
                    true,
                    IntakeState.Up),
                new FollowPathTask(isRed ? "17To8Red" : "17To8Blue", Type.Absolute)
            ),

            new IntakeGamePieceTask(false)
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
