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
            else if (routine == AutoRoutine.TaxiCharge)
            {
                return loadTaxiCharge(isRed);
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
            else if (routine == AutoRoutine.OnePlusOne)
            {
                return loadOnePlusOne(isRed);
            }
            else
            {
                return ConcurrentTask.AllTasks(
                    new ResetLevelTask(),
                    new PositionStartingTask(
                        isRed ? TuningConstants.LoadEdgeStartX : -TuningConstants.LoadEdgeStartX,
                        TuningConstants.LoadEdgeY,
                        isRed ? 0.0 : 180.0,
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
            else if (routine == AutoRoutine.TaxiCharge)
            {
                return midTaxiCharge(isRed);
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
                        isRed ? TuningConstants.StartGridX : -TuningConstants.StartGridX,
                        TuningConstants.StartFiveGridY,
                        isRed ? 180 : 0.0,
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
            else if (routine == AutoRoutine.TaxiCharge)
            {
                return guardTaxiCharge(isRed);
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
            else if (routine == AutoRoutine.OnePlusOne)
            {
                return guardOnePlusOne(isRed);
            }
            else
            {
                return ConcurrentTask.AllTasks(
                    new ResetLevelTask(),
                    new PositionStartingTask(
                        isRed ? TuningConstants.GuardEdgeStartX : -TuningConstants.GuardEdgeStartX,
                        TuningConstants.GuardEdgeY,
                        isRed ? 0 : 180.0,
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

    private static IControlTask loadTaxiCharge(boolean isRed)
    {
        return SequentialTask.Sequence(
            ConcurrentTask.AllTasks(
                new ResetLevelTask(),
                new PositionStartingTask(
                    isRed ? TuningConstants.LoadEdgeStartX : -TuningConstants.LoadEdgeStartX,
                    TuningConstants.LoadEdgeY,
                    isRed ? 0.0 : 180.0,
                    true,
                    true)),
            new FollowPathTask(isRed ? "LSToChargeRed" : "LSToChargeBlue", Type.Absolute),
            new ChargeStationTaskv2(false)
        );
    }

    private static IControlTask loadOnePlusTaxi(boolean isRed)
    {
        return SequentialTask.Sequence(
            ConcurrentTask.AllTasks(
                new ResetLevelTask(),
                new PositionStartingTask(
                    isRed ? TuningConstants.LoadEdgeStartX : -TuningConstants.LoadEdgeStartX,
                    TuningConstants.LoadEdgeY,
                    isRed ? 0.0 : 180.0,
                    true,
                    true)),
            new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_HIGH_CONE, TuningConstants.ARM_UPPER_POSITION_HIGH_CONE),
            new FollowPathTask(isRed ? "LoadStartTo1Red" : "LoadStartTo1Blue", Type.Absolute),
            new IntakeExtendTask(true),
            new WaitTask(0.5),
            new FollowPathTask(isRed ? "1To10Red" : "1To10Blue", Type.Absolute),
            new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_STOWED, TuningConstants.ARM_UPPER_POSITION_STOWED),
            new FollowPathTask(isRed ? "10To20Red" : "10To20Blue", Type.Absolute)
        );
    }

    private static IControlTask loadOnePlusCharge(boolean isRed)
    {
        return SequentialTask.Sequence(
            ConcurrentTask.AllTasks(
                new ResetLevelTask(),
                new PositionStartingTask(
                    isRed ? TuningConstants.LoadEdgeStartX : -TuningConstants.LoadEdgeStartX,
                    TuningConstants.LoadEdgeY,
                    isRed ? 0.0 : 180.0,
                    true,
                    true)),
            new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_HIGH_CONE, TuningConstants.ARM_UPPER_POSITION_HIGH_CONE),
            new FollowPathTask(isRed ? "LoadStartTo1Red" : "LoadStartTo1Blue", Type.Absolute),
            new IntakeExtendTask(true),
            new WaitTask(0.5),
            ConcurrentTask.AllTasks(
                new FollowPathTask(isRed ? "1ToChargeRed" : "1ToChargeBlue", Type.Absolute),
                SequentialTask.Sequence(
                    new WaitTask(1.0),
                    new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_STOWED, TuningConstants.ARM_UPPER_POSITION_STOWED)
                )
            ),
            new ChargeStationTaskv2(false)
        );
    }

    private static IControlTask loadOnePickupCharge(boolean isRed)
    {
        return SequentialTask.Sequence(
            ConcurrentTask.AllTasks(
                new ResetLevelTask(),
                new PositionStartingTask(
                    isRed ? TuningConstants.LoadEdgeStartX : -TuningConstants.LoadEdgeStartX,
                    TuningConstants.LoadEdgeY,
                    isRed ? 0.0 : 180.0,
                    true,
                    true)),
            ConcurrentTask.AllTasks(
                new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_HIGH_CONE, TuningConstants.ARM_UPPER_POSITION_HIGH_CONE),
                new FollowPathTask(isRed ? "LoadStartTo1Red" : "LoadStartTo1Blue", Type.Absolute)
            ),
            new IntakeExtendTask(true),
            new WaitTask(0.5),
            ConcurrentTask.AllTasks(
                new FollowPathTask(isRed ? "1To10Red" : "1To10Blue", Type.Absolute),
                SequentialTask.Sequence(
                    new WaitTask(0.5),
                    new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_STOWED, TuningConstants.ARM_UPPER_POSITION_STOWED)
                )
            ),
            new FollowPathTask(isRed ? "10To20Red" : "10To20Blue", Type.Absolute),
            new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_GROUND_PICKUP, TuningConstants.ARM_UPPER_POSITION_GROUND_PICKUP),
            ConcurrentTask.AllTasks(
                new FollowPathTask(isRed ? "20To20Red" : "20To20Blue", Type.Absolute),
                new IntakeGamePieceTask(3)
            ),
            ConcurrentTask.AllTasks(
                new FollowPathTask(isRed ? "20ToChargeRed" : "20ToChargeBlue", Type.Absolute),
                new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_STOWED, TuningConstants.ARM_LOWER_POSITION_STOWED)
            ),
            new ChargeStationTaskv2(false)
        );
    }

    private static IControlTask loadOnePlusOne(boolean isRed)
    {
        return SequentialTask.Sequence(
            ConcurrentTask.AllTasks(
                new ResetLevelTask(),
                new PositionStartingTask(
                    isRed ? TuningConstants.LoadEdgeStartX : -TuningConstants.LoadEdgeStartX,
                    TuningConstants.LoadEdgeY,
                    isRed ? 0.0 : 180.0,
                    true,
                    true)),
            ConcurrentTask.AllTasks(
                new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_HIGH_CONE, TuningConstants.ARM_UPPER_POSITION_HIGH_CONE),
                new FollowPathTask(isRed ? "LoadStartTo1Red" : "LoadStartTo1Blue", Type.Absolute)
            ),
            new IntakeExtendTask(true),
            new WaitTask(0.5),
            ConcurrentTask.AllTasks(
                new FollowPathTask(isRed ? "1To10Red" : "1To10Blue", Type.Absolute),
                SequentialTask.Sequence(
                    new WaitTask(0.5),
                    new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_STOWED, TuningConstants.ARM_UPPER_POSITION_STOWED)
                )
            ),
            new FollowPathTask(isRed ? "10To20Red" : "10To20Blue", Type.Absolute),
            new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_GROUND_PICKUP, TuningConstants.ARM_UPPER_POSITION_GROUND_PICKUP),
            ConcurrentTask.AllTasks(
                new FollowPathTask(isRed ? "20To20Red" : "20To20Blue", Type.Absolute),
                new IntakeGamePieceTask(3)
            ),
            ConcurrentTask.AllTasks(
                new FollowPathTask(isRed ? "20To10Red" : "20To10Blue", Type.Absolute),
                new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_APPROACH, TuningConstants.ARM_LOWER_POSITION_APPROACH)
            ),
            ConcurrentTask.AllTasks(
                new FollowPathTask(isRed ? "10To3Red" : "10To3Blue", Type.Absolute),
                new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_HIGH_CUBE, TuningConstants.ARM_LOWER_POSITION_HIGH_CUBE)
            ),
            new IntakeExtendTask(true),
            new WaitTask(0.5)
        );
    }

    private static IControlTask midTaxi(boolean isRed)
    {
        return SequentialTask.Sequence(
            ConcurrentTask.AllTasks(
                new ResetLevelTask(),
                new PositionStartingTask(
                    isRed ? TuningConstants.CloseChargeStationX : -TuningConstants.CloseChargeStationX,
                    TuningConstants.ChargeStationY,
                    isRed ? 180.0 : 0.0,
                    true,
                    true)),
            new FollowPathTask(isRed ? "MidTaxiRed" : "MidTaxiBlue", Type.Absolute)
        );
    }

    private static IControlTask midCharge(boolean isRed)
    {
        return SequentialTask.Sequence(
            new ChargeStationTaskv2(true)
        );
    }

    private static IControlTask midTaxiCharge(boolean isRed)
    {
        return SequentialTask.Sequence(
            ConcurrentTask.AllTasks(
                new ResetLevelTask(),
                new PositionStartingTask(
                    isRed ? TuningConstants.StartGridX : -TuningConstants.StartGridX,
                    TuningConstants.StartFiveGridY,
                    isRed ? 180 : 0.0,
                    true,
                    true)),

            new FollowPathTask(isRed ? "5ToChargeRed" : "5ToChargeBlue", Type.Absolute),
            new ChargeStationTaskv2(false)
        );
    }

    private static IControlTask midOnePlusCharge(boolean isRed)
    {
        return SequentialTask.Sequence(
            ConcurrentTask.AllTasks(
                new ResetLevelTask(),
                new PositionStartingTask(
                    isRed ? TuningConstants.StartGridX : -TuningConstants.StartGridX,
                    TuningConstants.StartFiveGridY,
                    isRed ? 0.0 : 180.0,
                    true,
                    true)),

            ConcurrentTask.AllTasks(
                new FollowPathTask(isRed ? "5To11Red" : "5To11Blue", Type.Absolute),
                SequentialTask.Sequence(
                    new WaitTask(0.5),
                    new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_HIGH_CUBE, TuningConstants.ARM_UPPER_POSITION_HIGH_CUBE)
                )
            ),

            new FollowPathTask(isRed ? "11To5Red" : "11To5Blue", Type.Absolute),
            new IntakeExtendTask(true),
            new IntakeInTask(false, 1.5),

            ConcurrentTask.AllTasks(
                new FollowPathTask(isRed ? "5To11Red" : "5To11Blue", Type.Absolute),
                SequentialTask.Sequence(
                    new WaitTask(0.5),
                    new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_STOWED, TuningConstants.ARM_UPPER_POSITION_STOWED)
                )
            ),

            new ChargeStationTaskv2(isRed ? true : false, isRed ? 0.0 : 180.0)
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

    private static IControlTask guardTaxiCharge(boolean isRed)
    {
        return SequentialTask.Sequence(
            ConcurrentTask.AllTasks(
                new ResetLevelTask(),
                new PositionStartingTask(
                    isRed ? TuningConstants.GuardEdgeStartX : -TuningConstants.GuardEdgeStartX,
                    TuningConstants.GuardEdgeY,
                    isRed ? 0 : 180.0,
                    true,
                    true)),
            new FollowPathTask(isRed ? "GSToChargeRed" : "GSToChargeBlue", Type.Absolute),
            new ChargeStationTaskv2(false)
        );
    }

    private static IControlTask guardOnePlusTaxi(boolean isRed)
    {
        return SequentialTask.Sequence(
            ConcurrentTask.AllTasks(
                new ResetLevelTask(),
                new PositionStartingTask(
                    isRed ? TuningConstants.GuardEdgeStartX : -TuningConstants.GuardEdgeStartX,
                    TuningConstants.GuardEdgeY,
                    isRed ? 0 : 180.0,
                    true,
                    true)),
            new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_HIGH_CONE, TuningConstants.ARM_UPPER_POSITION_HIGH_CONE),
            new FollowPathTask(isRed ? "GuardStartTo9Red" : "GuardStartTo9Blue", Type.Absolute),
            new IntakeExtendTask(true),
            new WaitTask(0.5),
            new FollowPathTask(isRed ? "9To12Red" : "9To12Blue", Type.Absolute),
            new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_STOWED, TuningConstants.ARM_UPPER_POSITION_STOWED),
            new FollowPathTask(isRed ? "12To23Red" : "12To23Blue", Type.Absolute)
        );
    }

    private static IControlTask guardOnePlusCharge(boolean isRed)
    {
        return SequentialTask.Sequence(
            ConcurrentTask.AllTasks(
                new ResetLevelTask(),
                new PositionStartingTask(
                    isRed ? TuningConstants.GuardEdgeStartX : -TuningConstants.GuardEdgeStartX,
                    TuningConstants.GuardEdgeY,
                    isRed ? 0 : 180.0,
                    true,
                    true)),
            new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_HIGH_CONE, TuningConstants.ARM_UPPER_POSITION_HIGH_CONE),
            new FollowPathTask(isRed ? "GuardStartTo9Red" : "GuardStartTo9Blue", Type.Absolute),
            new IntakeExtendTask(true),
            new WaitTask(0.5),
            ConcurrentTask.AllTasks(
                new FollowPathTask(isRed ? "9ToChargeRed" : "9ToChargeBlue", Type.Absolute),
                SequentialTask.Sequence(
                    new WaitTask(1.0),
                    new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_STOWED, TuningConstants.ARM_UPPER_POSITION_STOWED)
                )
            ),
            new ChargeStationTaskv2(false)
        );
    }

    private static IControlTask guardOnePickupCharge(boolean isRed)
    {
        return SequentialTask.Sequence(
            ConcurrentTask.AllTasks(
                new ResetLevelTask(),
                new PositionStartingTask(
                    isRed ? TuningConstants.GuardEdgeStartX : -TuningConstants.GuardEdgeStartX,
                    TuningConstants.GuardEdgeY,
                    isRed ? 0 : 180.0,
                    true,
                    true)),
            ConcurrentTask.AllTasks(
                new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_HIGH_CONE, TuningConstants.ARM_UPPER_POSITION_HIGH_CONE),
                new FollowPathTask(isRed ? "GuardStartTo9Red" : "GuardStartTo9Blue", Type.Absolute)
            ),
            new IntakeExtendTask(true),
            new WaitTask(0.5),
            ConcurrentTask.AllTasks(
                new FollowPathTask(isRed ? "9To12Red" : "9To12Blue", Type.Absolute),
                SequentialTask.Sequence(
                    new WaitTask(0.5),
                    new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_STOWED, TuningConstants.ARM_UPPER_POSITION_STOWED)
                )
            ),
            new FollowPathTask(isRed ? "12To23Red" : "12To23Blue", Type.Absolute),
            new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_GROUND_PICKUP, TuningConstants.ARM_UPPER_POSITION_GROUND_PICKUP),
            ConcurrentTask.AllTasks(
                new FollowPathTask(isRed ? "23To23Red" : "23To23Blue", Type.Absolute),
                new IntakeGamePieceTask(3)
            ),
            ConcurrentTask.AllTasks(
                new FollowPathTask(isRed ? "23ToChargeRed" : "23ToChargeBlue", Type.Absolute),
                new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_STOWED, TuningConstants.ARM_LOWER_POSITION_STOWED)
            ),
            new ChargeStationTaskv2(false)
        );
    }

    private static IControlTask guardOnePlusOne(boolean isRed)
    {
        return SequentialTask.Sequence(
            ConcurrentTask.AllTasks(
                new ResetLevelTask(),
                new PositionStartingTask(
                    isRed ? TuningConstants.GuardEdgeStartX : -TuningConstants.GuardEdgeStartX,
                    TuningConstants.GuardEdgeY,
                    isRed ? 0 : 180.0,
                    true,
                    true)),
            ConcurrentTask.AllTasks(
                new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_HIGH_CONE, TuningConstants.ARM_UPPER_POSITION_HIGH_CONE),
                new FollowPathTask(isRed ? "GuardStartTo9Red" : "GuardStartTo9Blue", Type.Absolute)
            ),
            new IntakeExtendTask(true),
            new WaitTask(0.5),
            ConcurrentTask.AllTasks(
                new FollowPathTask(isRed ? "9To12Red" : "9To12Blue", Type.Absolute),
                SequentialTask.Sequence(
                    new WaitTask(0.5),
                    new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_STOWED, TuningConstants.ARM_UPPER_POSITION_STOWED)
                )
            ),
            new FollowPathTask(isRed ? "12To23Red" : "12To23Blue", Type.Absolute),
            new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_GROUND_PICKUP, TuningConstants.ARM_UPPER_POSITION_GROUND_PICKUP),
            ConcurrentTask.AllTasks(
                new FollowPathTask(isRed ? "23To23Red" : "23To23Blue", Type.Absolute),
                new IntakeGamePieceTask(3)
            ),
            ConcurrentTask.AllTasks(
                new FollowPathTask(isRed ? "23To12Red" : "23To12Blue", Type.Absolute),
                new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_APPROACH, TuningConstants.ARM_LOWER_POSITION_APPROACH)
            ),
            ConcurrentTask.AllTasks(
                new FollowPathTask(isRed ? "12To7Red" : "12To7Blue", Type.Absolute),
                new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_HIGH_CUBE, TuningConstants.ARM_LOWER_POSITION_HIGH_CUBE)
            ),
            new IntakeInTask(false),
            new WaitTask(0.5)
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
