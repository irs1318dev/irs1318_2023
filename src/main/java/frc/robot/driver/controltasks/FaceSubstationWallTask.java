package frc.robot.driver.controltasks;

import frc.lib.robotprovider.Alliance;
import frc.lib.robotprovider.IDriverStation;
import frc.lib.robotprovider.IRobotProvider;
import frc.robot.TuningConstants;

public class FaceSubstationWallTask extends DecisionSequentialTask
{
    public FaceSubstationWallTask()
    {
    }

    @Override
    public void begin()
    {
        IRobotProvider provider = this.getInjector().getInstance(IRobotProvider.class);
        IDriverStation driverStation = provider.getDriverStation();
        if (driverStation.getAlliance() == Alliance.Blue)
        {
            this.AppendTask(new OrientationTask(TuningConstants.DRIVETRAIN_ORIENTATION_FACE_SUBSTATION_WALL));
        }
        else // if (driverStation.getAlliance() == Alliance.Red)
        {
            this.AppendTask(new OrientationTask(-TuningConstants.DRIVETRAIN_ORIENTATION_FACE_SUBSTATION_WALL));
        }
    }
}
