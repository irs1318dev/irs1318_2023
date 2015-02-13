package org.usfirst.frc.team1318.robot;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

import org.usfirst.frc.team1318.robot.Arm.ArmComponent;
import org.usfirst.frc.team1318.robot.Arm.ArmController;
import org.usfirst.frc.team1318.robot.Autonomous.AutonomousDriver;
import org.usfirst.frc.team1318.robot.Autonomous.IAutonomousTask;
import org.usfirst.frc.team1318.robot.Autonomous.Tasks.DriveDistanceAutonomousTask;
import org.usfirst.frc.team1318.robot.Autonomous.Tasks.DriveForwardTask;
import org.usfirst.frc.team1318.robot.Autonomous.Tasks.DriveTimedAutonomousTask;
import org.usfirst.frc.team1318.robot.Autonomous.Tasks.TurnAutonomousTask;
import org.usfirst.frc.team1318.robot.Autonomous.Tasks.WaitAutonomousTask;
import org.usfirst.frc.team1318.robot.Common.IDriver;
import org.usfirst.frc.team1318.robot.Common.SmartDashboardLogger;
import org.usfirst.frc.team1318.robot.Compressor.CompressorComponent;
import org.usfirst.frc.team1318.robot.Compressor.CompressorController;
import org.usfirst.frc.team1318.robot.DriveTrain.DriveTrainComponent;
import org.usfirst.frc.team1318.robot.DriveTrain.DriveTrainController;
import org.usfirst.frc.team1318.robot.DriveTrain.IDriveTrainComponent;
import org.usfirst.frc.team1318.robot.DriveTrain.PositionManager;
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

    // Arm 
    private ArmComponent armComponent;
    private ArmController armController;

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
        this.armComponent = new ArmComponent();

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

        if (this.armController != null)
        {
            this.armController.stop();
            this.armController = null;
        }

        SmartDashboardLogger.putString(Robot.ROBOT_STATE_LOG_KEY, "Disabled");
    }

    /**
     * Initialization code for autonomous mode should go here.
     * This code will be called each time the robot enters autonomous mode.
     */
    public void autonomousInit()
    {
        // determine our desired autonomous routine
        List<IAutonomousTask> autonomousRoutine;

        // select autonomous routine based on setting in SmartDashboard
        switch ((int)this.autonomousRoutineChooser.getSelected() % 3)
        {
            case 0:
                autonomousRoutine = Robot.GetDriveInSquareRoutine();
                break;

            case 1:
                autonomousRoutine = Robot.GetDriveInSquareByDistanceRoutine(this.driveTrainComponent);
                break;

            case 2:
                autonomousRoutine = Robot.GetDriveForwardRoutine();
                break;

            default:
                autonomousRoutine = Robot.GetDriveInSquareRoutine();
                break;
        }

        // create autonomous driver based on our desired routine
        this.driver = new AutonomousDriver(new LinkedList<IAutonomousTask>(autonomousRoutine));

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
        this.armController = new ArmController(this.driver, this.armComponent);

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
        this.armController.update();
    }

    /**
     * Gets an autonomous routine that represents driving in a square based on positional PID
     * 
     * @return list of autonomous tasks
     */
    private static List<IAutonomousTask> GetDriveInSquareByDistanceRoutine(IDriveTrainComponent driveTrainComponent)
    {
        return Arrays.asList(
            // drive in a square
            new DriveDistanceAutonomousTask(600, driveTrainComponent),
            new WaitAutonomousTask(5),
            new TurnAutonomousTask(90, driveTrainComponent),
            new DriveDistanceAutonomousTask(600, driveTrainComponent),
            new WaitAutonomousTask(5),
            new TurnAutonomousTask(90, driveTrainComponent),
            new DriveDistanceAutonomousTask(600, driveTrainComponent),
            new WaitAutonomousTask(5),
            new TurnAutonomousTask(90, driveTrainComponent),
            new DriveDistanceAutonomousTask(600, driveTrainComponent),
            new WaitAutonomousTask(5),
            new TurnAutonomousTask(90, driveTrainComponent));
    }

    /**
     * Gets an autonomous routine that represents driving in a square based on drive times
     * 
     * @return list of autonomous tasks
     */
    private static List<IAutonomousTask> GetDriveInSquareRoutine()
    {
        return Arrays.asList(
            // drive in a square
            new DriveTimedAutonomousTask(5, 0.0, 0.8),  // drive forward
            new WaitAutonomousTask(5),
            new DriveTimedAutonomousTask(2, 0.8, 0.0),  // turn right
            new DriveTimedAutonomousTask(5, 0.0, 0.8),  // drive forward
            new WaitAutonomousTask(5),
            new DriveTimedAutonomousTask(2, 0.8, 0.0),  // turn right
            new DriveTimedAutonomousTask(5, 0.0, 0.8),  // drive forward
            new WaitAutonomousTask(5),
            new DriveTimedAutonomousTask(2, 0.8, 0.0),  // turn right
            new DriveTimedAutonomousTask(5, 0.0, 0.8),  // drive forward
            new WaitAutonomousTask(5),
            new DriveTimedAutonomousTask(2, 0.8, 0.0)); // turn right
    }

    /**
     * Gets an autonomous routine that represents driving straight forward forever.
     * 
     * @return list of autonomous tasks
     */
    private static List<IAutonomousTask> GetDriveForwardRoutine()
    {
        return Arrays.asList(new DriveForwardTask());
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
