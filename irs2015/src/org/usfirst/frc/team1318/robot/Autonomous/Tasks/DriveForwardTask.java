package org.usfirst.frc.team1318.robot.Autonomous.Tasks;

import org.usfirst.frc.team1318.robot.Autonomous.AutonomousControlData;
import org.usfirst.frc.team1318.robot.Autonomous.IAutonomousTask;

/**
 * Simple drive-forward task
 * 
 * @author Caroline
 *
 */
public class DriveForwardTask implements IAutonomousTask
{
    public DriveForwardTask()
    {
    }

    @Override
    public void begin()
    {

    }

    @Override
    public void update(AutonomousControlData data)
    {
        data.setDriveTrainYVelocity(.1);
    }

    @Override
    public void cancel(AutonomousControlData data)
    {
        data.setDriveTrainYVelocity(0);
    }

    @Override
    public void end(AutonomousControlData data)
    {
        data.setDriveTrainYVelocity(0);
    }

    @Override
    public boolean shouldContinueProcessingTask()
    {
        return true;
    }
}
