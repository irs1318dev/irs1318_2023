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
import org.usfirst.frc.team1318.robot.Autonomous.Tasks.DriveSinusoidalTimedAutonomousTask;
import org.usfirst.frc.team1318.robot.Autonomous.Tasks.DriveSinusoidalTimedWithAngleTask;
import org.usfirst.frc.team1318.robot.Autonomous.Tasks.DriveTimedAutonomousTask;
import org.usfirst.frc.team1318.robot.Autonomous.Tasks.ElevatorBottomTask;
import org.usfirst.frc.team1318.robot.Autonomous.Tasks.ElevatorLevelTask;
import org.usfirst.frc.team1318.robot.Autonomous.Tasks.IntakeTask;
import org.usfirst.frc.team1318.robot.Autonomous.Tasks.SequentialTask;
import org.usfirst.frc.team1318.robot.Autonomous.Tasks.TurnAutonomousTask;
import org.usfirst.frc.team1318.robot.Autonomous.Tasks.WaitAutonomousTask;
import org.usfirst.frc.team1318.robot.Autonomous.Tasks.WaitForeverTask;
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

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.IterativeRobot;

/**
 * Main class for the FRC 2015 Robot for IRS1318 - Toothless
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

    // DipSwitches for selecting autonomous mode
    private DigitalInput dipSwitchA;
    private DigitalInput dipSwitchB;

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

        this.driver = new UserDriver();

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

        // create position manager
        this.position = new PositionManager(this.driveTrainComponent);

        SmartDashboardLogger.putString(Robot.ROBOT_STATE_LOG_KEY, "Init");

        this.dipSwitchA = new DigitalInput(ElectronicsConstants.AUTONOMOUS_DIP_SWITCH_A);
        this.dipSwitchB = new DigitalInput(ElectronicsConstants.AUTONOMOUS_DIP_SWITCH_B);
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

        if (this.compressorController != null)
        {
            this.compressorController.stop();
        }

        if (this.driveTrainController != null)
        {
            this.driveTrainController.stop();
        }

        if (this.elevatorController != null)
        {
            this.elevatorController.stop();
        }

        if (this.armController != null)
        {
            this.armController.stop();
        }

        if (this.intakeController != null)
        {
            this.intakeController.stop();
        }

        SmartDashboardLogger.putString(Robot.ROBOT_STATE_LOG_KEY, "Disabled");
    }

    /**
     * Initialization code for autonomous mode should go here.
     * This code will be called each time the robot enters autonomous mode.
     */
    public void autonomousInit()
    {
        // reset the drivetrain component and position manager so that we consider ourself at the origin (0,0) and facing the 0 direction.
        this.driveTrainComponent.reset();
        this.position.reset();

        // Find desired autonomous routine.
        //        IAutonomousTask[] autonomousRoutine = Robot.GetSampleRoutine(this.elevatorComponent, this.driveTrainComponent);
        //        IAutonomousTask[] autonomousRoutine = Robot.GetMoveForwardRoutine(this.driveTrainComponent);
        //        IAutonomousTask[] autonomousRoutine = Robot.GetRetrieveContainersFromStepRoutine(this.driveTrainComponent);
        //        IAutonomousTask[] autonomousRoutine = Robot.GetContainerlessCollectThreeTotesRoutine(this.elevatorComponent);
        //        IAutonomousTask[] autonomousRoutine = Robot.GetSpitContainersCollectThreeTotesRoutine(this.elevatorComponent, this.driveTrainComponent);
        IAutonomousTask[] autonomousRoutine = Robot.GetFillerRoutine();

        int routineSelection = 0;
        if (this.dipSwitchA.get())
        {
            routineSelection += 1;
        }

        if (this.dipSwitchB.get())
        {
            routineSelection += 2;
        }

        //select autonomous routine based on the dipswitch positions
        switch (routineSelection)
        {
            case 0: //neither flipped 
                autonomousRoutine = Robot.GetSinusoidalCollectThreeTotesRoutine(this.driveTrainComponent, this.position,
                    this.elevatorComponent);
                break;

            case 1: //switch A flipped 
                autonomousRoutine = Robot.GetRetrieveContainersFromStepRoutine(this.driveTrainComponent);
                break;

            case 2: //switch B flipped 
                autonomousRoutine = Robot.GetContainerlessCollectThreeTotesRoutine(this.elevatorComponent);
                break;

            default:    //both flipped or can't read 
                autonomousRoutine = Robot.GetFillerRoutine();
                break;
        }

        SmartDashboardLogger.putNumber(Robot.AUTONOMOUS_ROUTINE_PREFERENCE_KEY, routineSelection);

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
        // apply the driver to the controllers
        this.driveTrainController.setDriver(this.driver);
        this.elevatorController.setDriver(this.driver);
        this.armController.setDriver(this.driver);
        this.intakeController.setDriver(this.driver);

        // we will run the compressor controller here because we should start it in advance...
        this.compressorController.update();

        // by default we want to be in Velocity PID mode whenever we start (or switch between) a periodic mode
        this.driveTrainController.setVelocityPIDMode();
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
     * Gets an autonomous routine that drives forward into the appropriate zone
     * 
     * @return list of autonomous tasks
     */
    private static IAutonomousTask[] GetMoveForwardRoutine(DriveTrainComponent driveTrainComponent)
    {
        return new IAutonomousTask[]
        {
            //            new DriveDistanceAutonomousTask(10 * 12 * 2.54, driveTrainComponent),
            new DriveTimedAutonomousTask(2.0, 0.0, 0.32)
        };
    }

    private static IAutonomousTask[] GetTurnAndSpitTestRoutine(DriveTrainComponent driveTrainComponent)
    {
        return new IAutonomousTask[]
        {
            // 1: Spin intake in while driving forward for 2 seconds to collect the container
            ConcurrentTask.AllTasks(
                new IntakeTask(1.5, false),
                new DriveTimedAutonomousTask(1.5, 0.0, 0.1)),

            // 2: Turn to the right side, spit out the container, and then turn back
            ConcurrentTask.AllTasks(
                new IntakeTask(2.0, false),
                new SequentialTask(
                    new IAutonomousTask[]
                    {
                        new TurnAutonomousTask(45, driveTrainComponent),
                        new TurnAutonomousTask(-45, driveTrainComponent),
                    })),

        };
    }

    /**
     * Gets an autonomous routine that represents collecting 3 totes using the "other robots push containers away" method
     * 
     * @return list of autonomous tasks
     */
    private static IAutonomousTask[] GetSinusoidalCollectThreeTotesRoutineOld(ElevatorComponent elevatorComponent)
    {
        return new IAutonomousTask[]
        {
            // Collect tote #1, which should be pre-set in place
            new CollectToteTask(elevatorComponent),

            // Lift tote #1 while driving slowly around container #1
            ConcurrentTask.AllTasks(
                new SequentialTask(
                    new IAutonomousTask[]
                    {
                        new WaitAutonomousTask(2),
                        new ElevatorBottomTask(elevatorComponent, true),
                        new ElevatorLevelTask(elevatorComponent, 1, 0, true),
                    }),
                new DriveSinusoidalTimedAutonomousTask(3.5, .2, -.6, 1.25)),

            // Collect tote #2 while driving forward
            ConcurrentTask.AnyTasks(
                new SequentialTask(
                    new IAutonomousTask[]
                    {
                        new DriveTimedAutonomousTask(3.0, 0.0, 0.15),
                        new WaitAutonomousTask(20),
                    }),
                new CollectToteTask(elevatorComponent)),

            // Lift tote #2 while driving slowly around container #2
            ConcurrentTask.AllTasks(
                new SequentialTask(
                    new IAutonomousTask[]
                    {
                        new WaitAutonomousTask(2),
                        new ElevatorBottomTask(elevatorComponent, true),
                        new ElevatorLevelTask(elevatorComponent, 1, 0, true),
                    }),
                new DriveSinusoidalTimedAutonomousTask(3.5, .2, -.6, 1)),

            // Collect tote #3 while driving forward                
            ConcurrentTask.AnyTasks(
                new SequentialTask(
                    new IAutonomousTask[]
                    {
                        new DriveTimedAutonomousTask(3.0, 0.0, 0.15),
                        new WaitAutonomousTask(20),
                    }),
                new CollectToteTask(elevatorComponent)),

            // Set first two totes on top of 3rd tote
            // Turn around to the right, dragging all 3 totes at regular level
            ConcurrentTask.AllTasks(
                new ElevatorBottomTask(elevatorComponent, true),
                new DriveTimedAutonomousTask(4.0, 0.325, 0.3)),

            // Spit out the totes while driving backwards slowly
            ConcurrentTask.AllTasks(
                new IntakeTask(1.5, true),
                new DriveTimedAutonomousTask(1.5, 0.0, -0.25))
        };
    }

    /**
     * Gets an autonomous routine that retrieves containers from the step
     * 
     * @return list of autonomous tasks
     */
    private static IAutonomousTask[] GetRetrieveContainersFromStepRoutine(DriveTrainComponent driveTrainComponent)
    {
        // Drive backwards, extend the arm (extender, trombone, tilt), drive forwards, [retract the arm (tilt, trombone, extender)]
        return new IAutonomousTask[]
        {
            //            new DriveTimedAutonomousTask(0.5, 0.0, -0.2),

            //            new DriveDistanceAutonomousTask(-50, driveTrainComponent),
            ConcurrentTask.AllTasks(
                new ArmExtenderTask(1.5, true),
                new SequentialTask(
                    new IAutonomousTask[]
                    {
                        new WaitAutonomousTask(.5),
                        new DriveTimedAutonomousTask(0.7, 0.0, -0.3)
                    })

                ),
            new DriveTimedAutonomousTask(0.4, 0.0, 0.17),
            //            new ArmTromboneTask(1, true),
            new ArmTiltTask(2, true),
            new DriveTimedAutonomousTask(2.25, 0.0, 0.32),
            new WaitAutonomousTask(0.3),
            new DriveTimedAutonomousTask(1, 0, -0.2),
            //            new DriveDistanceAutonomousTask(100, driveTrainComponent)
            new ArmTiltTask(1, false),
            new ArmTromboneTask(1, false),
            new ArmExtenderTask(1, false),
        };
    }

    private static IAutonomousTask[] GetRetrieveContainersFromStepFastRoutine(DriveTrainComponent driveTrainComponent)
    {
        // Drive backwards, extend the arm (extender, trombone, tilt), drive forwards, [retract the arm (tilt, trombone, extender)]
        return new IAutonomousTask[]
        {
            //            new DriveTimedAutonomousTask(0.5, 0.0, -0.2),

            //            new DriveDistanceAutonomousTask(-50, driveTrainComponent),
            ConcurrentTask.AllTasks(
                new ArmExtenderTask(1.5, true),
                new SequentialTask(
                    new IAutonomousTask[]
                    {
                        new WaitAutonomousTask(.5),
                        new DriveTimedAutonomousTask(0.7, 0.0, -0.3)
                    })

                ),
            new DriveTimedAutonomousTask(0.4, 0.0, 0.17),
            //            new ArmTromboneTask(1, true),
            new ArmTiltTask(2, true),
            new DriveTimedAutonomousTask(2.25, 0.0, 0.32),
            new WaitAutonomousTask(0.3),
            new DriveTimedAutonomousTask(1, 0, -0.2),
            //            new DriveDistanceAutonomousTask(100, driveTrainComponent)
            new ArmTiltTask(1, false),
            new ArmTromboneTask(1, false),
            new ArmExtenderTask(1, false),
        };
    }

    private static IAutonomousTask[] GetFillerRoutine()
    {
        return new IAutonomousTask[]
        {
            new WaitAutonomousTask(0)
        };
    }

    /**
     * Gets an autonomous routine that represents collecting 3 totes using the "other robots push containers away" method
     * 
     * @return list of autonomous tasks
     */
    private static IAutonomousTask[] GetContainerlessCollectThreeTotesRoutine(ElevatorComponent elevatorComponent)
    {
        // Move forward and collect a tote 3 times, wait 2 seconds, then move elevator to bottom and unload tote.
        return new IAutonomousTask[]
        {
            new WaitAutonomousTask(2.5),

            // Collect tote #1, which should be pre-set in place
            new CollectToteTask(elevatorComponent),

            // Lift tote #1 while driving forward more slowly
            ConcurrentTask.AnyTasks(
                new SequentialTask(
                    new IAutonomousTask[]
                    {
                        new ElevatorBottomTask(elevatorComponent, true),
                        new ElevatorLevelTask(elevatorComponent, 2, 0, true),
                    }),
                new DriveTimedAutonomousTask(2.5, 0.0, 0.2)),

            // Collect tote #2 while driving forward
            ConcurrentTask.AnyTasks(
                new DriveTimedAutonomousTask(3.0, 0.0, 0.25),
                new CollectToteTask(elevatorComponent)),

            // Lift tote #2 while driving forward more slowly
            ConcurrentTask.AnyTasks(
                new SequentialTask(
                    new IAutonomousTask[]
                    {
                        new ElevatorBottomTask(elevatorComponent, true),
                        new ElevatorLevelTask(elevatorComponent, 2, 0, true),
                    }),
                new DriveTimedAutonomousTask(2.5, 0.0, 0.2)),

            // Collect tote #3 while driving forward                
            ConcurrentTask.AnyTasks(
                new DriveTimedAutonomousTask(3.0, 0.0, 0.25),
                new CollectToteTask(elevatorComponent)),

            // Set first two totes on top of 3rd tote
            // Turn around to the right, dragging all 3 totes at regular level
            ConcurrentTask.AllTasks(
                new ElevatorBottomTask(elevatorComponent, true),
                new DriveTimedAutonomousTask(4.0, 0.325, 0.3)),

            // Spit out the totes while driving backwards slowly
            ConcurrentTask.AllTasks(
                new IntakeTask(1.5, true),
                new DriveTimedAutonomousTask(1.5, 0.0, -0.25))
        };
    }

    /**
     * Gets an autonomous routine that collects 3 totes using the "push container aside" method
     * 
     * @return list of autonomous tasks
     */
    private static IAutonomousTask[] GetPushContainersCollectThreeTotesRoutine(ElevatorComponent elevatorComponent)
    {
        // Move forward and collect a tote 3 times, wait 2 seconds, then move elevator to bottom and unload tote.
        return new IAutonomousTask[]
        {
            // Collect tote #1
            new CollectToteTask(elevatorComponent),

            // Lift tote #1 to level 3 (fast)
            new ElevatorLevelTask(elevatorComponent, 3, 0, true),

            // Spin intake out while driving forward for 1 second to kick container out of the way
            ConcurrentTask.AllTasks(
                new IntakeTask(1.0, true),
                new DriveTimedAutonomousTask(1.0, 0.0, 0.2)),

            // Drive forward until we have collected tote #2
            ConcurrentTask.AnyTasks(
                new DriveTimedAutonomousTask(2.0, 0.0, 0.25),
                new CollectToteTask(elevatorComponent)),

            // Drop elevator to bottom, then lift tote #2
            new ElevatorBottomTask(elevatorComponent, true),
            new ElevatorLevelTask(elevatorComponent, 3, 0, true),

            // Spin intake out while driving forward for 1 second to kick container out of the way
            ConcurrentTask.AllTasks(
                new IntakeTask(1.0, true),
                new DriveTimedAutonomousTask(1.0, 0.0, 0.2)),

            // Drive forward until we have collected tote #3
            ConcurrentTask.AnyTasks(
                new DriveTimedAutonomousTask(2.0, 0.0, 0.25),
                new CollectToteTask(elevatorComponent)),

            // Drop elevator to bottom.  This will put the tote under our control, but we don't want to lift it.
            new ElevatorBottomTask(elevatorComponent, true),

            // Drive in a curve to the right.
            new DriveTimedAutonomousTask(2.0, 0.3, 0.2),

            // Spit out the totes while driving backwards slowly
            ConcurrentTask.AllTasks(
                new IntakeTask(1.5, true),
                new DriveTimedAutonomousTask(1.5, 0.0, -0.25))
        };
    }

    /**
     * Gets an autonomous routine that collects 3 totes using the "push container aside" method
     * 
     * @return list of autonomous tasks
     */
    private static IAutonomousTask[] GetSpitContainersCollectThreeTotesRoutine(
        ElevatorComponent elevatorComponent, DriveTrainComponent driveTrainComponent)
    {
        // Move forward and collect a tote 3 times, wait 2 seconds, then move elevator to bottom and unload tote.
        return new IAutonomousTask[]
        {
            // 0: Collect tote #1 (spin intake in until the through beam sensor is hit)
            new CollectToteTask(elevatorComponent),

            // 1: Lift tote #1 to level 3 (fast)
            new ElevatorLevelTask(elevatorComponent, 3, 0, true),

            // 2: Spin intake in while driving forward for 2 seconds to collect the container
            ConcurrentTask.AllTasks(
                new IntakeTask(1.5, false),
                new DriveTimedAutonomousTask(1.5, 0.0, 0.2)),

            // 3: Turn to the right side, spit out the container, and then turn back
            ConcurrentTask.AllTasks(
                new IntakeTask(2.0, false),
                new SequentialTask(
                    new IAutonomousTask[]
                    {
                        new TurnAutonomousTask(45, driveTrainComponent),
                        new TurnAutonomousTask(-45, driveTrainComponent),
                    })),

            // 4: Drive forward until we have collected tote #2
            ConcurrentTask.AnyTasks(
                new DriveTimedAutonomousTask(2.0, 0.0, 0.25),
                new CollectToteTask(elevatorComponent)),

            // 5: Drop elevator to bottom, then lift tote #2
            new ElevatorBottomTask(elevatorComponent, true),
            new ElevatorLevelTask(elevatorComponent, 3, 0, true),

            // 6: Spin intake in while driving forward for 2 seconds to collect the container
            ConcurrentTask.AllTasks(
                new IntakeTask(2.0, false),
                new DriveTimedAutonomousTask(2.0, 0.0, 0.2)),

            // 7: Turn to the right side, spit out the container, and then turn back
            ConcurrentTask.AllTasks(
                new IntakeTask(2.0, false),
                new SequentialTask(
                    new IAutonomousTask[]
                    {
                        new TurnAutonomousTask(45, driveTrainComponent),
                        new TurnAutonomousTask(-45, driveTrainComponent)
                    })),

            // 8: Drive forward until we have collected tote #3
            ConcurrentTask.AnyTasks(
                new DriveTimedAutonomousTask(2.0, 0.0, 0.25),
                new CollectToteTask(elevatorComponent)),

            // Drop elevator to bottom.  This will complete the stack with the tote under our control, but we don't want to lift it.
            new ElevatorBottomTask(elevatorComponent, true),

            // Drive in a curve to the right.
            new DriveTimedAutonomousTask(2.0, 0.3, 0.2),

            // Spit out the totes while driving backwards slowly
            ConcurrentTask.AllTasks(
                new IntakeTask(1.5, true),
                new DriveTimedAutonomousTask(1.5, 0.0, -0.25)),
        };
    }

    /**
     * Gets an autonomous routine that drives around the containers to collect the three autonomous totes and deposit them in the auto zone 
     * 
     * @return list of autonomous tasks
     */
    private static IAutonomousTask[] GetSinusoidalCollectThreeTotesRoutine(
        DriveTrainComponent driveTrainComponent, PositionManager positionManager, ElevatorComponent elevatorComponent)
    {
        //        return new IAutonomousTask[]
        //        {
        //            new DriveSinusoidalTimedAutonomousTask(3.5, .2, -.6, 1.25),
        //            new TurnAbsoluteTask(0, driveTrainComponent, positionManager)
        //        };

        return new IAutonomousTask[]
        {
            // Collect tote #1, which should be pre-set in place
            new CollectToteTask(elevatorComponent),

            // Lift tote #1 while driving slowly around container #1
            ConcurrentTask.AllTasks(
                new SequentialTask(
                    new IAutonomousTask[]
                    {
                        new WaitAutonomousTask(2),
                        new ElevatorBottomTask(elevatorComponent, true),
                        new ElevatorLevelTask(elevatorComponent, 1, 0, true),
                    }),
                //                new DriveSinusoidalTimedWithAngleTask(3.5, .2, .6, 0.95, positionManager, driveTrainComponent)),
                new DriveSinusoidalTimedWithAngleTask(3.5, .2, .7, 0.9, positionManager, driveTrainComponent)),

            // Collect tote #2 while driving forward
            ConcurrentTask.AnyTasks(
                new SequentialTask(
                    new IAutonomousTask[]
                    {
                        new DriveTimedAutonomousTask(3.0, 0.0, 0.175),
                        new WaitForeverTask(),
                    }),
                new CollectToteTask(elevatorComponent)),

            // Lift tote #2 while driving slowly around container #2
            ConcurrentTask.AllTasks(
                new SequentialTask(
                    new IAutonomousTask[]
                    {
                        new WaitAutonomousTask(2),
                        new ElevatorBottomTask(elevatorComponent, true),
                        new ElevatorLevelTask(elevatorComponent, 1, 0, true),
                    }),
                //                new DriveSinusoidalTimedWithAngleTask(3.5, .2, .6, 0.8, positionManager, driveTrainComponent)),
                new DriveSinusoidalTimedWithAngleTask(3.5, .2, .65, 0.75, positionManager, driveTrainComponent)),
            //                new DriveSinusoidalTimedAutonomousTask(3.5, .2, .6, 0.8)),

            // Collect tote #3 while driving forward                
            ConcurrentTask.AnyTasks(
                new SequentialTask(
                    new IAutonomousTask[]
                    {
                        new DriveTimedAutonomousTask(2.0, 0.0, 0.25),
                    }),
                new CollectToteTask(elevatorComponent)),

            // Set first two totes on top of 3rd tote
            // Turn around to the right, dragging all 3 totes at regular level
            ConcurrentTask.AllTasks(
                new ElevatorLevelTask(elevatorComponent, 0, 2, true),
                new SequentialTask(
                    new IAutonomousTask[]
                    {
                        new DriveTimedAutonomousTask(1, .5, 0),
                        new DriveTimedAutonomousTask(2, 0.2, 0.325)
                    })),

            // Spit out the totes while driving backwards slowly
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
