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

        if(routine == AutoRoutine.GuardOnePlusCharge)
        {
            return guardOnePlusCharge(isRed);
        }
        else if(routine == AutoRoutine.GuardOnePlusOne)
        {
            return guardOnePlusOne();
        }
        else if(routine == AutoRoutine.LoadOnePlusCharge)
        {
            return loadOnePlusCharge();
        }
        else if(routine == AutoRoutine.LoadOnePlusOne)
        {
            return loadOnePlusOne();
        }
        else if(routine == AutoRoutine.MiddleOnePlusCharge)
        {
            return middleOnePlusCharge();
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

    private static IControlTask guardOnePlusCharge(boolean isRed)
    {
        return SequentialTask.Sequence(

            new PositionStartingTask(
                TuningConstants.GuardEdgeStartX, 
                TuningConstants.GuardEdgeY, 
                180.0, 
                false, false),

            ConcurrentTask.AllTasks(
                // new FollowPathTask(isRed ? "A" : "GStoGM", false, true) ,
                new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_APPROACH, TuningConstants.ARM_UPPER_POSITION_APPROACH)
            ),

            ConcurrentTask.AllTasks(
                // new FollowPathTask("GMtoGE", false, true),
                new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_HIGH_CUBE, TuningConstants.ARM_UPPER_POSITION_HIGH_CUBE)
            ),
            
            //new FollowPathTask("GEto8", false, true),
            new IntakeInTask(false),

            ConcurrentTask.AllTasks(
                //new FollowPathTask("8to12", false, true),
                SequentialTask.Sequence(
                    new WaitTask(0.5),
                    new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_APPROACH, TuningConstants.ARM_UPPER_POSITION_APPROACH)
                )
            ),

            ConcurrentTask.AllTasks(
                //new FollowPathTask("12to19", false, true),
                new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_STOWED, TuningConstants.ARM_UPPER_POSITION_STOWED)
            ),

            //new FollowPathTask("19to13", false, true),
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
                //new FollowPathTask("GStoGM", false, true) ,
                new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_APPROACH, TuningConstants.ARM_UPPER_POSITION_APPROACH)
            ),

            ConcurrentTask.AllTasks(
                //new FollowPathTask("GMtoGE", false, true),
                new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_HIGH_CONE, TuningConstants.ARM_UPPER_POSITION_HIGH_CONE)
            ),
            
            //new FollowPathTask("GEto9", false, true),
            new IntakeExtendTask(true),

            ConcurrentTask.AllTasks(
                //new FollowPathTask("9to12", false, true),
                SequentialTask.Sequence(
                    new WaitTask(0.5),
                    new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_APPROACH, TuningConstants.ARM_UPPER_POSITION_APPROACH)
                )
            ),

            ConcurrentTask.AllTasks(
                //new FollowPathTask("12to23", false, false),
                SequentialTask.Sequence(
                    new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_STOWED, TuningConstants.ARM_UPPER_POSITION_STOWED),
                    new WaitTask(1.5),
                    new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_GROUND_PICKUP, TuningConstants.ARM_UPPER_POSITION_GROUND_PICKUP)
                )
            ),

            ConcurrentTask.AllTasks(
                //new FollowPathTask("23to17", false, true),
                new IntakeGamePieceTask(3.0)
            ),

            ConcurrentTask.AllTasks(
                //new FollowPathTask("17to23", false, false),
                new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_STOWED, TuningConstants.ARM_UPPER_POSITION_STOWED)
            ),

            ConcurrentTask.AllTasks(
                //new FollowPathTask("23to12", false, true),
                SequentialTask.Sequence(
                    new WaitTask(2.0),
                    new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_HIGH_CUBE, TuningConstants.ARM_UPPER_POSITION_HIGH_CUBE)
                )
            ),


            //new FollowPathTask("12to8", false, true),
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
                //new FollowPathTask("LStoLM", false, true) ,
                new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_APPROACH, TuningConstants.ARM_UPPER_POSITION_APPROACH)
            ),

            ConcurrentTask.AllTasks(
                //new FollowPathTask("LMtoLE", false, true),
                new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_HIGH_CUBE, TuningConstants.ARM_UPPER_POSITION_HIGH_CUBE)
            ),
            
            //new FollowPathTask("LEto2", false, true),
            new IntakeInTask(false),

            ConcurrentTask.AllTasks(
                //new FollowPathTask("2to12", false, true),
                SequentialTask.Sequence(
                    new WaitTask(0.5),
                    new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_APPROACH, TuningConstants.ARM_UPPER_POSITION_APPROACH)
                )
            ),

            ConcurrentTask.AllTasks(
                //new FollowPathTask("10to18", false, true),
                new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_STOWED, TuningConstants.ARM_UPPER_POSITION_STOWED)
            ),

            //new FollowPathTask("18to13", false, true),
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
                //new FollowPathTask("LStoLM", false, true) ,
                new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_APPROACH, TuningConstants.ARM_UPPER_POSITION_APPROACH)
            ),

            ConcurrentTask.AllTasks(
                //new FollowPathTask("LMtoLE", false, true),
                new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_HIGH_CONE, TuningConstants.ARM_UPPER_POSITION_HIGH_CONE)
            ),
            
            //new FollowPathTask("LEto1", false, true),
            new IntakeExtendTask(true),

            ConcurrentTask.AllTasks(
                //new FollowPathTask("1to10", false, true),
                SequentialTask.Sequence(
                    new WaitTask(0.5),
                    new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_APPROACH, TuningConstants.ARM_UPPER_POSITION_APPROACH)
                )
            ),

            ConcurrentTask.AllTasks(
                //new FollowPathTask("10to20", false, false),
                SequentialTask.Sequence(
                    new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_STOWED, TuningConstants.ARM_UPPER_POSITION_STOWED),
                    new WaitTask(1.5),
                    new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_GROUND_PICKUP, TuningConstants.ARM_UPPER_POSITION_GROUND_PICKUP)
                )
            ),

            ConcurrentTask.AllTasks(
                //new FollowPathTask("20to14", false, true),
                new IntakeGamePieceTask(3.0)
            ),

            ConcurrentTask.AllTasks(
                //new FollowPathTask("14to20", false, false),
                new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_STOWED, TuningConstants.ARM_UPPER_POSITION_STOWED)
            ),

            ConcurrentTask.AllTasks(
                //new FollowPathTask("20to10", false, true),
                SequentialTask.Sequence(
                    new WaitTask(2.0),
                    new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_HIGH_CUBE, TuningConstants.ARM_UPPER_POSITION_HIGH_CUBE)
                )
            ),


            //new FollowPathTask("10to2", false, true),
            new IntakeInTask(false)
        );
    }

    private static IControlTask middleOnePlusCharge()
    {
        return SequentialTask.Sequence(

            new PositionStartingTask(
                TuningConstants.StartGridX, 
                TuningConstants.StartFiveGridY, 
                180.0, 
                false, false),

            ConcurrentTask.AllTasks(
                //new FollowPathTask("5to11", false, true),
                SequentialTask.Sequence(
                    new WaitTask(0.5),
                    new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_APPROACH, TuningConstants.ARM_UPPER_POSITION_APPROACH)   
                )
            ),

            ConcurrentTask.AllTasks(
                //new FollowPathTask("11to5", false, true),
                new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_HIGH_CUBE, TuningConstants.ARM_UPPER_POSITION_HIGH_CUBE)
            ),
            
            new IntakeInTask(false),

            ConcurrentTask.AllTasks(
                //new FollowPathTask("5to11Turn", false, true),
                SequentialTask.Sequence(
                    new WaitTask(0.5),
                    new ArmMMPositionTask(TuningConstants.ARM_LOWER_POSITION_STOWED, TuningConstants.ARM_UPPER_POSITION_STOWED)
                )
            ),

            //new FollowPathTask("11to13", false, true),
            //new FollowPathTask("TurnAround", false, false),
            new ChargeStationTask(false)
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
