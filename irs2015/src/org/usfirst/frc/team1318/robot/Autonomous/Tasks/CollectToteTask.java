package org.usfirst.frc.team1318.robot.Autonomous.Tasks;

import org.usfirst.frc.team1318.robot.Autonomous.AutonomousControlData;
import org.usfirst.frc.team1318.robot.Autonomous.IAutonomousTask;
import org.usfirst.frc.team1318.robot.Elevator.ElevatorComponent;

import edu.wpi.first.wpilibj.Timer;

/**
 * CollectToteTask:
 * 
 * This task runs the intake until it notices that the through beam sensor has been broken, and then it 
 * stops the intake and indicates that the elevator pick-up macro should be run (for enough time that the command will be detected).
 * 
 * @author Will
 *
 */
public class CollectToteTask implements IAutonomousTask
{
    private static final double MACRO_DETECTION_TIME = 0.4;

    private ElevatorComponent elevatorComponent;
    private Timer timer;

    private Double startPickUpTime;
    private boolean hasDetectedThroughBeamBroken;

    public CollectToteTask(ElevatorComponent elevatorComponent)
    {
        this.elevatorComponent = elevatorComponent;
        this.timer = new Timer();

        this.startPickUpTime = null;
        this.hasDetectedThroughBeamBroken = false;
    }

    @Override
    public void begin()
    {
        this.hasDetectedThroughBeamBroken = false;
        this.timer.start();
        this.startPickUpTime = null;
    }

    @Override
    public void update(AutonomousControlData data)
    {
        if (!this.hasDetectedThroughBeamBroken)
        {
            if (this.elevatorComponent.getThroughBeamBroken())
            {
                this.startPickUpTime = this.timer.get();
                this.hasDetectedThroughBeamBroken = true;
            }
        }

        data.setElevatorTotePickUpMacroState(this.hasDetectedThroughBeamBroken);
        data.setIntakeForwardState(!this.hasDetectedThroughBeamBroken);
    }

    @Override
    public void cancel(AutonomousControlData data)
    {
        data.setIntakeForwardState(false);
        data.setElevatorTotePickUpMacroState(false);
    }

    @Override
    public void end(AutonomousControlData data)
    {
        data.setIntakeForwardState(false);
        data.setElevatorTotePickUpMacroState(false);
    }

    @Override
    public boolean shouldContinue()
    {
        if (this.startPickUpTime == null)
        {
            return true;
        }

        return this.timer.get() < this.startPickUpTime + CollectToteTask.MACRO_DETECTION_TIME;
    }
}
