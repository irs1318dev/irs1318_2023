package org.usfirst.frc.team1318.robot;

import org.usfirst.frc.team1318.robot.Common.DashboardLogger;
import org.usfirst.frc.team1318.robot.Driver.Driver;
import org.usfirst.frc.team1318.robot.Driver.IControlTask;
import org.usfirst.frc.team1318.robot.Driver.Autonomous.AutonomousDriver;
import org.usfirst.frc.team1318.robot.Driver.ControlTasks.WaitTask;
import org.usfirst.frc.team1318.robot.Driver.User.UserDriver;

import edu.wpi.first.wpilibj.IterativeRobot;

/**
 * Main class for the FRC 201? [competition name] Competition
 * Robot for IRS1318 - [robot name]
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
    private static final String AUTONOMOUS_ROUTINE_PREFERENCE_KEY = "a.routine value";

    // Driver.  This could either be the UserDriver (joystick) or the AutonomousDriver
    private Driver driver;

    // Components
    private ComponentManager components;

    // Controllers
    private ControllerManager controllers;

    /**
     * Robot-wide initialization code should go here.
     * This default Robot-wide initialization code will be called when 
     * the robot is first powered on.  It will be called exactly 1 time.
     */
    public void robotInit()
    {
        // create mechanism components and controllers
        this.components = new ComponentManager();
        this.controllers = new ControllerManager(this.components);

        DashboardLogger.putString(Robot.ROBOT_STATE_LOG_KEY, "Init");
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
        }

        if (this.controllers != null)
        {
            this.controllers.stop();
        }

        DashboardLogger.putString(Robot.ROBOT_STATE_LOG_KEY, "Disabled");
    }

    /**
     * Initialization code for autonomous mode should go here.
     * This code will be called each time the robot enters autonomous mode.
     */
    public void autonomousInit()
    {
        // reset the position manager so that we consider ourself at the origin (0,0) and facing the 0 direction.
        this.components.getPositionManager().reset();

        // Find desired autonomous routine.
        IControlTask autonomousRoutine = Robot.GetFillerRoutine();

        DashboardLogger.putInteger(Robot.AUTONOMOUS_ROUTINE_PREFERENCE_KEY, 0);

        // Create autonomous driver based on our desired routine
        this.driver = new AutonomousDriver(autonomousRoutine, this.components);

        this.generalInit();

        // log that we are in autonomous mode
        DashboardLogger.putString(Robot.ROBOT_STATE_LOG_KEY, "Autonomous");
    }

    /**
     * Initialization code for teleop mode should go here.
     * This code will be called each time the robot enters teleop mode.
     */
    public void teleopInit()
    {
        // create driver for user's joystick
        this.driver = new UserDriver(this.components);

        this.generalInit();

        // log that we are in teleop mode
        DashboardLogger.putString(Robot.ROBOT_STATE_LOG_KEY, "Teleop");
    }

    /**
     * General initialization code for teleop/autonomous mode should go here.
     */
    public void generalInit()
    {
        // apply the driver to the controllers
        this.controllers.setDriver(this.driver);
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
        this.driver.update();

        // run each controller
        this.controllers.update();
    }

    /**
     * Gets an autonomous routine that does nothing
     * 
     * @return very long WaitTask
     */
    private static IControlTask GetFillerRoutine()
    {
        return new WaitTask(0);
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
