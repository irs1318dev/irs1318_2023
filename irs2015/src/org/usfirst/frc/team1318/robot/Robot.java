package org.usfirst.frc.team1318.robot;

import org.usfirst.frc.team1318.robot.Arm.ArmComponent;
import org.usfirst.frc.team1318.robot.Arm.ArmController;
import org.usfirst.frc.team1318.robot.Autonomous.AutonomousDriver;
import org.usfirst.frc.team1318.robot.Autonomous.IAutonomousTask;
import org.usfirst.frc.team1318.robot.Autonomous.Tasks.ArmExtenderTask;
import org.usfirst.frc.team1318.robot.Autonomous.Tasks.ArmTiltTask;
import org.usfirst.frc.team1318.robot.Autonomous.Tasks.ArmTromboneTask;
import org.usfirst.frc.team1318.robot.Autonomous.Tasks.CollectToteTask;
import org.usfirst.frc.team1318.robot.Autonomous.Tasks.ConcurrentTask;
import org.usfirst.frc.team1318.robot.Autonomous.Tasks.DriveTimedAutonomousTask;
import org.usfirst.frc.team1318.robot.Autonomous.Tasks.ElevatorBottomTask;
import org.usfirst.frc.team1318.robot.Autonomous.Tasks.ElevatorLevelTask;
import org.usfirst.frc.team1318.robot.Autonomous.Tasks.IntakeTask;
import org.usfirst.frc.team1318.robot.Autonomous.Tasks.SequentialTask;
import org.usfirst.frc.team1318.robot.Autonomous.Tasks.WaitAutonomousTask;
import org.usfirst.frc.team1318.robot.Common.IDriver;
import org.usfirst.frc.team1318.robot.Common.SmartDashboardLogger;
import org.usfirst.frc.team1318.robot.Compressor.CompressorComponent;
import org.usfirst.frc.team1318.robot.Compressor.CompressorController;
import org.usfirst.frc.team1318.robot.DriveTrain.DriveTrainComponent;
import org.usfirst.frc.team1318.robot.DriveTrain.DriveTrainController;
import org.usfirst.frc.team1318.robot.DriveTrain.PositionManager;
import org.usfirst.frc.team1318.robot.Elevator.ElevatorComponent;
import org.usfirst.frc.team1318.robot.Elevator.ElevatorController;
import org.usfirst.frc.team1318.robot.Intake.IntakeComponent;
import org.usfirst.frc.team1318.robot.Intake.IntakeController;
import org.usfirst.frc.team1318.robot.UserInterface.UserDriver;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Main class for the FRC 2015 Robot for IRS1318 - [robot_name]
 * 
 * 
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package, you
 * must also update the manifest file in the resource directory.
 * 
 * 
 * General design comments:
 * We have three types of objects:
 * - Driver - describes the driver/operator of the robot ("autonomous" or "user")
 * - Components - describe the electronics of an mechanism and defines the abstract way to control those electronics.
 * - Controllers - define the logic that controls a mechanism given inputs/outputs.
 * 
 * @author Will
 */
public class Robot extends IterativeRobot
{
    // smartdash logging constants
    private static final String ROBOT_STATE_LOG_KEY = "r.s";

    // smartdash other constants 
    private static final String AUTONOMOUS_ROUTINE_PREFERENCE_KEY = "a.routine";

    // smartdash preferences and other inputs
    private SendableChooser autonomousRoutineChooser;

    // Driver (e.g. joystick, autonomous)
    private IDriver driver;

    // Compressor
    private CompressorComponent compressorComponent;
    private CompressorController compressorController;

    // DriveTrain
    private DriveTrainComponent driveTrainComponent;
    private DriveTrainController driveTrainController;

    // Elevator
    private ElevatorComponent elevatorComponent;
    private ElevatorController elevatorController;

    // Arm 
    private ArmComponent armComponent;
    private ArmController armController;

    // Intake
    private IntakeComponent intakeComponent;
    private IntakeController intakeController;

    // Position manager - holds position information relative to our starting point
    private PositionManager position;

    /**
     * Robot-wide initialization code should go here.
     * This default Robot-wide initialization code will be called when 
     * the robot is first powered on.  It will be called exactly 1 time.
     */
    public void robotInit()
    {
        // create mechanism components
        this.compressorComponent = new CompressorComponent();
        this.driveTrainComponent = new DriveTrainComponent();
        this.elevatorComponent = new ElevatorComponent();
        this.armComponent = new ArmComponent();
        this.intakeComponent = new IntakeComponent();

        // create position manager
        this.position = new PositionManager(this.driveTrainComponent);

        SmartDashboardLogger.putString(Robot.ROBOT_STATE_LOG_KEY, "Init");

        // set up chooser on SmartDashboard
        this.autonomousRoutineChooser = new SendableChooser();
        this.autonomousRoutineChooser.addDefault("Drive In Square", 0);
        this.autonomousRoutineChooser.addObject("Drive In Square Positional", 1);
        this.autonomousRoutineChooser.addObject("Drive Forward", 2);
        SmartDashboard.putData(Robot.AUTONOMOUS_ROUTINE_PREFERENCE_KEY, this.autonomousRoutineChooser);
    }

    /**
     * Initialization code for disabled mode should go here.
     * This code will be called each time the robot enters disabled mode.
     */
    public void disabledInit()
    {
        if (this.driver != null)
        {
            this.driver.stop();
            this.driver = null;
        }

        if (this.compressorController != null)
        {
            this.compressorController.stop();
            this.compressorController = null;
        }

        if (this.driveTrainController != null)
        {
            this.driveTrainController.stop();
            this.driveTrainController = null;
        }

        if (this.elevatorController != null)
        {
            this.elevatorController.stop();
            this.elevatorController = null;
        }

        if (this.armController != null)
        {
            this.armController.stop();
            this.armController = null;
        }

        if (this.intakeController != null)
        {
            this.intakeController.stop();
            this.intakeController = null;
        }

        SmartDashboardLogger.putString(Robot.ROBOT_STATE_LOG_KEY, "Disabled");
    }

    /**
     * Initialization code for autonomous mode should go here.
     * This code will be called each time the robot enters autonomous mode.
     */
    public void autonomousInit()
    {
        // Find desired autonomous routine.
        //        IAutonomousTask[] autonomousRoutine = Robot.GetSampleRoutine(this.elevatorComponent, this.driveTrainComponent);
        IAutonomousTask[] autonomousRoutine = Robot.GetRetrieveContainersFromStepRoutine();

        //        int routineSelection = 0;
        //        DigitalInput dipSwitchOne = new DigitalInput(ElectronicsConstants.AUTONOMOUS_DIP_SWITCH_A);
        //        DigitalInput dipSwitchTwo = new DigitalInput(ElectronicsConstants.AUTONOMOUS_DIP_SWITCH_B);
        //        if (dipSwitchOne.get())
        //        {
        //            routineSelection += 1;
        //        }
        //
        //        if (dipSwitchTwo.get())
        //        {
        //            routineSelection += 2;
        //        }
        //
        //        // select autonomous routine based on setting in SmartDashboard
        //        switch (routineSelection)
        //        {
        //            case 0:
        //                autonomousRoutine = Robot.GetDriveForwardRoutine();
        //                break;
        //
        //            case 1:
        //                autonomousRoutine = Robot.GetRetrieveContainersFromStepRoutine();
        //                break;
        //
        //            case 2:
        //                autonomousRoutine = Robot.GetCollectThreeTotesRoutine(this.elevatorComponent);
        //                break;
        //
        //            default:
        //                autonomousRoutine = Robot.GetSampleRoutine(this.elevatorComponent, this.driveTrainComponent);
        //                break;
        //        }

        // create autonomous driver based on our desired routine
        this.driver = new AutonomousDriver(autonomousRoutine);

        this.generalInit();

        // log that we are in autonomous mode
        SmartDashboardLogger.putString(Robot.ROBOT_STATE_LOG_KEY, "Autonomous");
    }

    /**
     * Initialization code for teleop mode should go here.
     * This code will be called each time the robot enters teleop mode.
     */
    public void teleopInit()
    {
        // create driver for user's joystick
        this.driver = new UserDriver();

        this.generalInit();

        // log that we are in teleop mode
        SmartDashboardLogger.putString(Robot.ROBOT_STATE_LOG_KEY, "Teleop");
    }

    /**
     * General initialization code for teleop/autonomous mode should go here.
     */
    public void generalInit()
    {
        // create controllers for each mechanism
        this.compressorController = new CompressorController(this.compressorComponent);
        this.driveTrainController =
            new DriveTrainController(
                this.driver,
                this.driveTrainComponent,
                TuningConstants.DRIVETRAIN_USE_PID_DEFAULT);
        this.elevatorController = new ElevatorController(this.driver, this.elevatorComponent);
        this.armController = new ArmController(this.driver, this.armComponent);
        this.intakeController = new IntakeController(this.driver, this.intakeComponent);

        // we will run the compressor controller here because we should start it in advance...
        this.compressorController.update();
    }

    /**
     * Periodic code for disabled mode should go here.
     * This code will be called periodically at a regular rate while the robot is in disabled mode.
     */
    public void disabledPeriodic()
    {
    }

    /**
     * Periodic code for autonomous mode should go here.
     * This code will be called periodically at a regular rate while the robot is in autonomous mode.
     */
    public void autonomousPeriodic()
    {
        this.generalPeriodic();
    }

    /**
     * Periodic code for teleop mode should go here.
     * This code will be called periodically at a regular rate while the robot is in teleop mode.
     */
    public void teleopPeriodic()
    {
        this.generalPeriodic();
    }

    /**
     * General periodic code for teleop/autonomous mode should go here.
     */
    public void generalPeriodic()
    {
        // update our position
        this.position.update();

        this.driver.update();

        // run each controller
        this.compressorController.update();
        this.driveTrainController.update();
        this.elevatorController.update();
        this.armController.update();
        this.intakeController.update();
    }

    /**
     * Gets an autonomous routine that drives straight forward for a certain length of time
     * 
     * @return list of autonomous tasks
     */
    private static IAutonomousTask[] GetDriveForwardRoutine()
    {
        // Drive forwards at .4 speed for 2 seconds
        return new IAutonomousTask[]
        {
            new DriveTimedAutonomousTask(2.0, 0.0, 0.4),
        };
    }

    /**
     * Gets an autonomous routine that retrieves containers from the step
     * 
     * @return list of autonomous tasks
     */
    private static IAutonomousTask[] GetRetrieveContainersFromStepRoutine()
    {
        // Drive backwards, extend the arm (extender, trombone, tilt), drive forwards, [retract the arm (tilt, trombone, extender)]
        return new IAutonomousTask[]
        {
            new DriveTimedAutonomousTask(1.5, 0.0, -0.4),
            new ArmExtenderTask(2, true),
            new ArmTromboneTask(2, true),
            new ArmTiltTask(2, true),
            new DriveTimedAutonomousTask(4.0, 0.0, 0.2),
        //new ArmTiltTask(2, false),
        //new ArmTromboneTask(2, false),
        //new ArmExtenderTask(2, false),
        };
    }

    /**
     * Gets an autonomous routine that represents driving in a square based on drive times
     * 
     * @return list of autonomous tasks
     */
    private static IAutonomousTask[] GetCollectThreeTotesRoutine(ElevatorComponent elevatorComponent)
    {
        // Move forward and collect a tote 3 times, wait 2 seconds, then move elevator to bottom and unload tote.
        return new IAutonomousTask[]
        {
            ConcurrentTask.AnyTasks(
                new DriveTimedAutonomousTask(3.0, 0.0, 0.25),
                new CollectToteTask(elevatorComponent)),
            ConcurrentTask.AllTasks(
                new SequentialTask(
                    new IAutonomousTask[]
                    {
                        new ElevatorBottomTask(elevatorComponent, false),
                        new ElevatorLevelTask(1.0, 2, 0, true),
                    }),
                new DriveTimedAutonomousTask(1.5, 0.0, 0.2)),
            ConcurrentTask.AnyTasks(
                new DriveTimedAutonomousTask(3.0, 0.0, 0.25),
                new CollectToteTask(elevatorComponent)),
            ConcurrentTask.AllTasks(
                new SequentialTask(
                    new IAutonomousTask[]
                    {
                        new ElevatorBottomTask(elevatorComponent, false),
                        new ElevatorLevelTask(1.0, 2, 0, true),
                    }),
                new DriveTimedAutonomousTask(1.5, 0.0, 0.2)),
            ConcurrentTask.AnyTasks(
                new DriveTimedAutonomousTask(3.0, 0.0, 0.25),
                new CollectToteTask(elevatorComponent)),
            ConcurrentTask.AllTasks(
                new SequentialTask(
                    new IAutonomousTask[]
                    {
                        new ElevatorBottomTask(elevatorComponent, false),
                        new ElevatorLevelTask(1.0, 2, 0, true),
                    }),
                new DriveTimedAutonomousTask(1.5, 0.0, 0.2)),
            new WaitAutonomousTask(1.0),
            new ElevatorBottomTask(elevatorComponent, false),
            ConcurrentTask.AllTasks(
                new IntakeTask(1.5, true),
                new DriveTimedAutonomousTask(1.5, 0.0, -0.25))
        };
    }

    /**
     * Gets an autonomous routine that is currently being experimented on
     * 
     * @return list of autonomous tasks
     */
    private static IAutonomousTask[] GetSampleRoutine(ElevatorComponent elevatorComponent, DriveTrainComponent driveTrainComponent)
    {
        // Move forward and collect a tote 3 times in elevator fast mode, wait 1 second, then move elevator to bottom and unload tote.
        return new IAutonomousTask[]
        {
            ConcurrentTask.AnyTasks(
                new DriveTimedAutonomousTask(3.0, 0.0, 0.25),
                new CollectToteTask(elevatorComponent)),
            ConcurrentTask.AllTasks(
                new SequentialTask(
                    new IAutonomousTask[]
                    {
                        new ElevatorBottomTask(elevatorComponent, true),
                        new ElevatorLevelTask(1.0, 1, 0, true),
                    }),
                new DriveTimedAutonomousTask(1.5, 0.0, 0.2)),
            ConcurrentTask.AnyTasks(
                new DriveTimedAutonomousTask(3.0, 0.0, 0.25),
                new CollectToteTask(elevatorComponent)),
            ConcurrentTask.AllTasks(
                new SequentialTask(
                    new IAutonomousTask[]
                    {
                        new ElevatorBottomTask(elevatorComponent, true),
                        new ElevatorLevelTask(1.0, 1, 0, true),
                    }),
                new DriveTimedAutonomousTask(1.5, 0.0, 0.2)),
            ConcurrentTask.AnyTasks(
                new DriveTimedAutonomousTask(3.0, 0.0, 0.25),
                new CollectToteTask(elevatorComponent)),
            ConcurrentTask.AllTasks(
                new SequentialTask(
                    new IAutonomousTask[]
                    {
                        new ElevatorBottomTask(elevatorComponent, true),
                        new ElevatorLevelTask(1.0, 1, 0, true),
                    }),
                new DriveTimedAutonomousTask(1.5, 0.0, 0.2)),
            new WaitAutonomousTask(1.0),
            new ElevatorBottomTask(elevatorComponent, false),
            ConcurrentTask.AllTasks(
                new IntakeTask(1.5, true),
                new DriveTimedAutonomousTask(1.5, 0.0, -0.25))
        };
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
                       ;:''::::::::#`              +:'    ';:;;+'::;;:;::::::''                     
                       +::;+':::::::'.            .:;+    '''+;::;:;:::;:::;':'                     
                        ';;:;'';:::::':           +::.     +:::::::::::::;#;:#                      
                         .''##;#;:;;:::'+        `+;'      ;:;::::::::;'+;:'+                       
                           ` `+:;+:;::;::+       +:;#      ';:::;:+#+';:::+.                        
                              ,+::+#';::;+       ';::      #:;;'+';'''++:`                          
                                '':::;'''#      ,:;;`      #';:;;:+                                 
                                 `:'++;;':       :++       .;;:;;#,                                 
                                       `                    '':``                                   


*/
