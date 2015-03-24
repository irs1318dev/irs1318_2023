package org.usfirst.frc.team1318.robot.DriveTrain;

public class DriveTrainMacroData
{
    public enum MacroStates
    {
        WAIT_FOR_PRESS_0, DRIVE_FORWARD_EXTEND_1, DRIVE_BACK_2, TILT_3, WAIT_4, DRIVE_BACK_5, WAIT_6, DRIVE_FORWARD_7, UNTILT_8, UNEXTEND_9
    }

    private boolean runningMacro;
    public MacroStates state;

    public DriveTrainMacroData()
    {
        runningMacro = false;
        state = MacroStates.WAIT_FOR_PRESS_0;
    }

    public boolean getRunningMacro()
    {
        return runningMacro;
    }

    public void setRunningMacro(boolean runningMacro)
    {
        this.runningMacro = runningMacro;
    }

}
