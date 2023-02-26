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

        this.logger.logString(LoggingKey.AutonomousSelection, startPosition.toString() + "." + routine.toString());

        if(startPosition == StartPosition.Load)
        {
            if(routine == AutoRoutine.Taxi)
            {
                return guardOnePlusCharge(isRed);
            }

            else if(routine == AutoRoutine.TaxiCharge)
            {
                return loadOnePlusCharge();
            }

            else if(routine == AutoRoutine.OnePlusTaxi)
            {
                return loadOnePlusOne();
            }

            else if(routine == AutoRoutine.OnePlusCharge)
            {
                return guardTaxi();
            }

            else if(routine == AutoRoutine.OnePickupCharge)
            {
                return guardTaxi();
            }

            else if(routine == AutoRoutine.OnePlusOne)
            {
                return guardTaxi();
            }
        }

        else if(startPosition == StartPosition.Mid)
        {
            if(routine == AutoRoutine.Taxi)
            {
                return guardOnePlusCharge(isRed);
            }
            
            else if(routine == AutoRoutine.Charge)
            {
                return guardOnePlusOne();
            }

            else if(routine == AutoRoutine.TaxiCharge)
            {
                return loadOnePlusCharge();
            }

            else if(routine == AutoRoutine.OnePlusCharge)
            {
                return guardTaxi();
            }
        }

        else if(startPosition == StartPosition.Guard)
        {
            if(routine == AutoRoutine.Taxi)
            {
                return guardOnePlusCharge(isRed);
            }

            else if(routine == AutoRoutine.TaxiCharge)
            {
                return loadOnePlusCharge();
            }
            
            else if(routine == AutoRoutine.OnePlusTaxi)
            {
                return loadOnePlusOne();
            }

            else if(routine == AutoRoutine.OnePlusCharge)
            {
                return guardTaxi();
            }
            
            else if(routine == AutoRoutine.OnePickupCharge)
            {
                return guardTaxi();
            }

            else if(routine == AutoRoutine.OnePlusOne)
            {
                return guardTaxi();
            }
        }
        return new PositionStartingTask(0.0, 0.0, 0.0);
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
            new PositionStartingTask(
                0.0, 
                0.0, 
                0.0, 
                false, 
                true),
            new FollowPathTask("LoadTaxi", false, true)
        );
    }

    private static IControlTask guardTaxi()
    {
        return SequentialTask.Sequence(
            new PositionStartingTask(
                0.0, 
                0.0, 
                0.0, 
                false, 
                true),
            new FollowPathTask("GuardTaxi", false, true)
        );
    }
    
    private static IControlTask loadTaxiCharge(boolean isRed)
    {
        return SequentialTask.Sequence(
            new PositionStartingTask(
                isRed ? TuningConstants.LoadEdgeStartX : -TuningConstants.LoadEdgeStartX, 
                TuningConstants.LoadEdgeY,
                180.0, 
                false, 
                false),
            new FollowPathTask(isRed ? "LSToChargeRed" : "LSToChargeBlue", false, true),
            new ChargeStationTask(false)
        );
    }

    private static IControlTask loadOnePlusTaxi(boolean isRed)
    {
        return SequentialTask.Sequence(
            new PositionStartingTask(
                isRed ? TuningConstants.LoadEdgeStartX : -TuningConstants.LoadEdgeStartX, 
                TuningConstants.LoadEdgeY,
                180.0, 
                false, 
                false),
            new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_HIGH_CONE, TuningConstants.ARM_UPPER_POSITION_HIGH_CONE),
            new FollowPathTask(isRed ? "LoadStartTo1Red" : "LoadStartTo1Blue", false, true),
            new IntakeExtendTask(true),
            new WaitTask(0.5),
            new FollowPathTask(isRed ? "1To10Red" : "1To10Blue", false, true),
            new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_STOWED, TuningConstants.ARM_UPPER_POSITION_STOWED),
            new FollowPathTask(isRed ? "10To20Red" : "10To20Blue", false, false)
        );
    }

    private static IControlTask loadOnePlusCharge(boolean isRed)
    {
        return SequentialTask.Sequence(
            new PositionStartingTask(
                isRed ? TuningConstants.LoadEdgeStartX : -TuningConstants.LoadEdgeStartX, 
                TuningConstants.LoadEdgeY,
                180.0, 
                false, 
                false),
            new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_HIGH_CONE, TuningConstants.ARM_UPPER_POSITION_HIGH_CONE),
            new FollowPathTask(isRed ? "LoadStartTo1Red" : "LoadStartTo1Blue", false, true),
            new IntakeExtendTask(true),
            new WaitTask(0.5),
            ConcurrentTask.AllTasks(
                new FollowPathTask(isRed ? "1ToChargeRed" : "1ToChargeBlue", false, true),
                SequentialTask.Sequence(
                    new WaitTask(1.0),
                    new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_STOWED, TuningConstants.ARM_UPPER_POSITION_STOWED)
                )
            ),
            new ChargeStationTask(false)
        );
    }

    private static IControlTask loadOnePickupCharge(boolean isRed)
    {
        return SequentialTask.Sequence(
            new PositionStartingTask(
                isRed ? TuningConstants.LoadEdgeStartX : -TuningConstants.LoadEdgeStartX, 
                TuningConstants.LoadEdgeY,
                180.0, 
                false, 
                false),
            ConcurrentTask.AllTasks(
                new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_HIGH_CONE, TuningConstants.ARM_UPPER_POSITION_HIGH_CONE),
                new FollowPathTask(isRed ? "LoadStartTo1Red" : "LoadStartTo1Blue", false, true)
            ),
            new IntakeExtendTask(true),
            new WaitTask(0.5),
            ConcurrentTask.AllTasks(
                new FollowPathTask(isRed ? "1To10Red" : "1To10Blue", false, true),
                SequentialTask.Sequence(
                    new WaitTask(0.5),
                    new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_STOWED, TuningConstants.ARM_UPPER_POSITION_STOWED)
                )
            ),
            new FollowPathTask(isRed ? "10To20Red" : "10To20Blue", false, false),
            new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_GROUND_PICKUP, TuningConstants.ARM_UPPER_POSITION_GROUND_PICKUP),
            ConcurrentTask.AllTasks(
                new FollowPathTask(isRed ? "20To20Red" : "20To20Blue", false, false),
                new IntakeGamePieceTask(3)
            ),
            ConcurrentTask.AllTasks(
                new FollowPathTask(isRed ? "20ToChargeRed" : "20ToChargeBlue", false, false),
                new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_STOWED, TuningConstants.ARM_LOWER_POSITION_STOWED)
            ),
            new ChargeStationTask(false)
        );
    }

    private static IControlTask loadOnePlusOne(boolean isRed)
    {
        return SequentialTask.Sequence(
            new PositionStartingTask(
                isRed ? TuningConstants.LoadEdgeStartX : -TuningConstants.LoadEdgeStartX, 
                TuningConstants.LoadEdgeY,
                180.0, 
                false, 
                false),
            ConcurrentTask.AllTasks(
                new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_HIGH_CONE, TuningConstants.ARM_UPPER_POSITION_HIGH_CONE),
                new FollowPathTask(isRed ? "LoadStartTo1Red" : "LoadStartTo1Blue", false, true)
            ),
            new IntakeExtendTask(true),
            new WaitTask(0.5),
            ConcurrentTask.AllTasks(
                new FollowPathTask(isRed ? "1To10Red" : "1To10Blue", false, true),
                SequentialTask.Sequence(
                    new WaitTask(0.5),
                    new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_STOWED, TuningConstants.ARM_UPPER_POSITION_STOWED)
                )
            ),
            new FollowPathTask(isRed ? "10To20Red" : "10To20Blue", false, false),
            new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_GROUND_PICKUP, TuningConstants.ARM_UPPER_POSITION_GROUND_PICKUP),
            ConcurrentTask.AllTasks(
                new FollowPathTask(isRed ? "20To20Red" : "20To20Blue", false, false),
                new IntakeGamePieceTask(3)
            ),
            ConcurrentTask.AllTasks(
                new FollowPathTask(isRed ? "20To10Red" : "20To10Blue", false, false),
                new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_APPROACH, TuningConstants.ARM_LOWER_POSITION_APPROACH)
            ),
            ConcurrentTask.AllTasks(
                new FollowPathTask(isRed ? "10To3Red" : "10To3Blue", false, true),
                new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_HIGH_CUBE, TuningConstants.ARM_LOWER_POSITION_HIGH_CUBE)
            ),
            new IntakeExtendTask(true),
            new WaitTask(0.5)
        );
    }

    private static IControlTask midTaxi(boolean isRed)
    {
        return SequentialTask.Sequence(
            new PositionStartingTask(
                isRed ? TuningConstants.CloseChargeStationX : -TuningConstants.CloseChargeStationX, 
                TuningConstants.ChargeStationY, 
                0.0, 
                false, 
                true),
            new FollowPathTask(isRed ? "MidTaxiRed" : "MidTaxiBlue", false, true)
        );
    }

    private static IControlTask midCharge(boolean isRed)
    {
        return SequentialTask.Sequence(
            new ChargeStationTask(false)
        );
    }

    private static IControlTask midTaxiCharge(boolean isRed)
    {
        return SequentialTask.Sequence(
            new PositionStartingTask(
                isRed ? TuningConstants.StartGridX : -TuningConstants.StartGridX, 
                TuningConstants.StartFiveGridY, 
                0.0,
                false, true),

            new FollowPathTask(isRed ? "5ToChargeRed" : "5ToChargeBlue", false, true),
            new ChargeStationTask(false)
        );
    }

    private static IControlTask midOnePlusCharge(boolean isRed)
    {
        return SequentialTask.Sequence(

            new PositionStartingTask(
                isRed ? TuningConstants.StartGridX : -TuningConstants.StartGridX, 
                TuningConstants.StartFiveGridY, 
                180.0, 
                false, false),

            ConcurrentTask.AllTasks(
                new FollowPathTask(isRed ? "5To11Red" : "11To5Blue", false, true),
                SequentialTask.Sequence(
                    new WaitTask(0.5),
                    new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_HIGH_CUBE, TuningConstants.ARM_UPPER_POSITION_HIGH_CUBE)   
                )
            ),

            new FollowPathTask("11to5", false, true),
            new IntakeInTask(false),

            ConcurrentTask.AllTasks(
                new FollowPathTask("5to11Turn", false, true),
                SequentialTask.Sequence(
                    new WaitTask(0.5),
                    new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_STOWED, TuningConstants.ARM_UPPER_POSITION_STOWED)
                )
            ),

            new FollowPathTask("11to13", false, true),
            new FollowPathTask("TurnAround", false, false),
            new ChargeStationTask(false)
        );
    }



    private static IControlTask guardOnePlusCharge(boolean isRed)
    {
        return SequentialTask.Sequence(

            new PositionStartingTask(
                TuningConstants.GuardEdgeStartX, 
                TuningConstants.GuardEdgeY, 
                180.0, 
                false, false),

            ConcurrentTask.AllTasks(
                new FollowPathTask(isRed ? "A" : "GStoGM", false, true) ,
                new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_APPROACH, TuningConstants.ARM_UPPER_POSITION_APPROACH)
            ),

            ConcurrentTask.AllTasks(
                new FollowPathTask("GMtoGE", false, true),
                new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_HIGH_CUBE, TuningConstants.ARM_UPPER_POSITION_HIGH_CUBE)
            ),
            
            new FollowPathTask("GEto8", false, true),
            new IntakeInTask(false),

            ConcurrentTask.AllTasks(
                new FollowPathTask("8to12", false, true),
                SequentialTask.Sequence(
                    new WaitTask(0.5),
                    new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_APPROACH, TuningConstants.ARM_UPPER_POSITION_APPROACH)
                )
            ),

            ConcurrentTask.AllTasks(
                new FollowPathTask("12to19", false, true),
                new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_STOWED, TuningConstants.ARM_UPPER_POSITION_STOWED)
            ),

            new FollowPathTask("19to13", false, true),
            new ChargeStationTask(false)
        );
    }

    private static IControlTask guardOnePlusOne()
    {
        return SequentialTask.Sequence(

            new PositionStartingTask(
                TuningConstants.GuardEdgeStartX, 
                TuningConstants.GuardEdgeY, 
                180.0, 
                false, false),

            ConcurrentTask.AllTasks(
                new FollowPathTask("GStoGM", false, true) ,
                new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_APPROACH, TuningConstants.ARM_UPPER_POSITION_APPROACH)
            ),

            ConcurrentTask.AllTasks(
                new FollowPathTask("GMtoGE", false, true),
                new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_HIGH_CONE, TuningConstants.ARM_UPPER_POSITION_HIGH_CONE)
            ),
            
            new FollowPathTask("GEto9", false, true),
            new IntakeExtendTask(true),

            ConcurrentTask.AllTasks(
                new FollowPathTask("9to12", false, true),
                SequentialTask.Sequence(
                    new WaitTask(0.5),
                    new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_APPROACH, TuningConstants.ARM_UPPER_POSITION_APPROACH)
                )
            ),

            ConcurrentTask.AllTasks(
                new FollowPathTask("12to23", false, false),
                SequentialTask.Sequence(
                    new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_STOWED, TuningConstants.ARM_UPPER_POSITION_STOWED),
                    new WaitTask(1.5),
                    new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_GROUND_PICKUP, TuningConstants.ARM_UPPER_POSITION_GROUND_PICKUP)
                )
            ),

            ConcurrentTask.AllTasks(
                new FollowPathTask("23to17", false, true),
                new IntakeGamePieceTask(3.0)
            ),

            ConcurrentTask.AllTasks(
                new FollowPathTask("17to23", false, false),
                new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_STOWED, TuningConstants.ARM_UPPER_POSITION_STOWED)
            ),

            ConcurrentTask.AllTasks(
                new FollowPathTask("23to12", false, true),
                SequentialTask.Sequence(
                    new WaitTask(2.0),
                    new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_HIGH_CUBE, TuningConstants.ARM_UPPER_POSITION_HIGH_CUBE)
                )
            ),


            new FollowPathTask("12to8", false, true),
            new IntakeInTask(false)
        );
    }

    private static IControlTask loadOnePlusCharge()
    {
        return SequentialTask.Sequence(

            new PositionStartingTask(
                TuningConstants.LoadEdgeStartX, 
                TuningConstants.LoadEdgeY, 
                180.0, 
                false, false),

            ConcurrentTask.AllTasks(
                new FollowPathTask("LStoLM", false, true) ,
                new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_APPROACH, TuningConstants.ARM_UPPER_POSITION_APPROACH)
            ),

            ConcurrentTask.AllTasks(
                new FollowPathTask("LMtoLE", false, true),
                new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_HIGH_CUBE, TuningConstants.ARM_UPPER_POSITION_HIGH_CUBE)
            ),
            
            new FollowPathTask("LEto2", false, true),
            new IntakeInTask(false),

            ConcurrentTask.AllTasks(
                new FollowPathTask("2to12", false, true),
                SequentialTask.Sequence(
                    new WaitTask(0.5),
                    new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_APPROACH, TuningConstants.ARM_UPPER_POSITION_APPROACH)
                )
            ),

            ConcurrentTask.AllTasks(
                new FollowPathTask("10to18", false, true),
                new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_STOWED, TuningConstants.ARM_UPPER_POSITION_STOWED)
            ),

            new FollowPathTask("18to13", false, true),
            new ChargeStationTask(false)
        );
    }

    private static IControlTask loadOnePlusOne()
    {
        return SequentialTask.Sequence(

            new PositionStartingTask(
                TuningConstants.LoadEdgeStartX, 
                TuningConstants.LoadEdgeY, 
                180.0, 
                false, false),

            ConcurrentTask.AllTasks(
                new FollowPathTask("LStoLM", false, true) ,
                new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_APPROACH, TuningConstants.ARM_UPPER_POSITION_APPROACH)
            ),

            ConcurrentTask.AllTasks(
                new FollowPathTask("LMtoLE", false, true),
                new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_HIGH_CONE, TuningConstants.ARM_UPPER_POSITION_HIGH_CONE)
            ),
            
            new FollowPathTask("LEto1", false, true),
            new IntakeExtendTask(true),

            ConcurrentTask.AllTasks(
                new FollowPathTask("1to10", false, true),
                SequentialTask.Sequence(
                    new WaitTask(0.5),
                    new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_APPROACH, TuningConstants.ARM_UPPER_POSITION_APPROACH)
                )
            ),

            ConcurrentTask.AllTasks(
                new FollowPathTask("10to20", false, false),
                SequentialTask.Sequence(
                    new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_STOWED, TuningConstants.ARM_UPPER_POSITION_STOWED),
                    new WaitTask(1.5),
                    new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_GROUND_PICKUP, TuningConstants.ARM_UPPER_POSITION_GROUND_PICKUP)
                )
            ),

            ConcurrentTask.AllTasks(
                new FollowPathTask("20to14", false, true),
                new IntakeGamePieceTask(3.0)
            ),

            ConcurrentTask.AllTasks(
                new FollowPathTask("14to20", false, false),
                new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_STOWED, TuningConstants.ARM_UPPER_POSITION_STOWED)
            ),

            ConcurrentTask.AllTasks(
                new FollowPathTask("20to10", false, true),
                SequentialTask.Sequence(
                    new WaitTask(2.0),
                    new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_HIGH_CUBE, TuningConstants.ARM_UPPER_POSITION_HIGH_CUBE)
                )
            ),


            new FollowPathTask("10to2", false, true),
            new IntakeInTask(false)
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
