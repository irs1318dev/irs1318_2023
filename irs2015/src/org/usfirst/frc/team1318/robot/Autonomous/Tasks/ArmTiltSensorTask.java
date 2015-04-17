package org.usfirst.frc.team1318.robot.Autonomous.Tasks;

import org.usfirst.frc.team1318.robot.Arm.ArmComponent;
import org.usfirst.frc.team1318.robot.Autonomous.AutonomousControlData;
import org.usfirst.frc.team1318.robot.Autonomous.IAutonomousTask;

public class ArmTiltSensorTask extends TimedAutonomousTask implements IAutonomousTask
{

    private final ArmComponent armComponent;

    private boolean sensorTriggered;
    //    private boolean sensor;
    //    private boolean timer;
    private boolean extend;

    public ArmTiltSensorTask(ArmComponent armComponent, double duration, boolean extend)
    {
        super(duration);
        this.armComponent = armComponent;
        this.sensorTriggered = false;
        this.extend = extend;
        //        this.sensor = false;
        //        this.timer = false;
    }

    @Override
    public void begin()
    {
        this.sensorTriggered = false;
        super.begin();
    }

    @Override
    public void update(AutonomousControlData data)
    {
        if (!this.sensorTriggered)
        {
            if (this.armComponent.getExtendSensorTripped() || super.hasCompleted())
            {
                this.sensorTriggered = true;
            }
            //            if (this.armComponent.getExtendSensorTripped())
            //            {
            //                sensor = true;
            //                this.sensorTriggered = true;
            //            }
            //            if (super.hasCompleted())
            //            {
            //                timer = true;
            //                this.sensorTriggered = true;
            //            }
        }
        if (this.sensorTriggered)
        {
            data.setArmTiltExtendOverrideState(!this.extend);
            data.setArmTiltRetractOverrideState(this.extend);
        }
    }

    @Override
    public void cancel(AutonomousControlData data)
    {

    }

    @Override
    public void end(AutonomousControlData data)
    {

    }

    @Override
    public boolean hasCompleted()
    {
        if (this.sensorTriggered)
        {
            return this.sensorTriggered;
        }
        else
        {
            return false;
        }
    }

}
