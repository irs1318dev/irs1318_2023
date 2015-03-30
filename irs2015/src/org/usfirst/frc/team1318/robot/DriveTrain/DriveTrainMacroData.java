package org.usfirst.frc.team1318.robot.DriveTrain;

public class DriveTrainMacroData
{
    public enum MacroStates
    {
        STATE_0_WAIT_FOR_PRESS, STATE_1_DRIVE_BACK, STATE_2_SETTLE_WAIT, STATE_3_DRIVE_BACK, STATE_4_WAIT, STATE_5_DRIVE_FORWARD, STATE_6_UNTILT, STATE_7_UNEXTEND
    }

    public static final double DRIVE_BACK_TIME_1 = 0.4;

    private boolean runningMacro;
    public MacroStates state;

    private boolean extenderState;
    private boolean tiltState;
    private boolean tromboneState;

    public DriveTrainMacroData()
    {
        runningMacro = false;
        state = MacroStates.STATE_0_WAIT_FOR_PRESS;
    }

    public boolean getRunningMacro()
    {
        return runningMacro;
    }

    public void setRunningMacro(boolean runningMacro)
    {
        this.runningMacro = runningMacro;
    }

    public boolean getExtenderState()
    {
        return extenderState;
    }

    public void setExtenderState(boolean extenderState)
    {
        this.extenderState = extenderState;
    }

    public boolean getTiltState()
    {
        return tiltState;
    }

    public void setTiltState(boolean tiltState)
    {
        this.tiltState = tiltState;
    }

    public boolean getTromboneState()
    {
        return tromboneState;
    }

    public void setTromboneState(boolean tromboneState)
    {
        this.tromboneState = tromboneState;
    }

}
