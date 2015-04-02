package org.usfirst.frc.team1318.robot.DriveTrain;

public class DriveTrainMacroData
{
    public enum MacroStates
    {
        STATE_0_WAIT_FOR_PRESS, STATE_1_DRIVE_BACK, STATE_2_SETTLE_WAIT, STATE_3_DRIVE_BACK, STATE_4_WAIT, STATE_5_DRIVE_FORWARD, STATE_6_UNTILT, STATE_7_UNEXTEND
    }

    public static final double DRIVE_BACK_TIME_1 = 0.4;
    public static final double DRIVE_BACK_SPEED_1 = 0.17;
    public static final double SETTLE_WAIT_TIME_2 = 2.0;
    public static final double DRIVE_BACK_TIME_3 = 2.25;
    public static final double DRIVE_BACK_SPEED_3 = 0.32;
    public static final double SETTLE_WAIT_TIME_4 = 0.3;
    public static final double DRIVE_FORWARD_TIME_5 = 1.0;
    public static final double DRIVE_FORWARD_SPEED_5 = -0.2;
    public static final double UNTILT_WAIT_TIME_6 = 1;
    public static final double UNEXTEND_WAIT_TIME_7 = 1;

    private boolean runningMacro;
    public MacroStates state;

    private boolean extenderState = false;
    private boolean tiltState = false;
    private boolean tromboneState = false;

    private boolean extenderExtendActivated = false;
    private boolean extenderRetractActivated = false;
    private boolean tiltExtendActivated = false;
    private boolean tiltRetractActivated = false;
    private boolean tromboneExtendActivated = false;
    private boolean tromboneRetractActivated = false;

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
        if (extenderState && !this.extenderState)
        {
            this.extenderExtendActivated = true;
        }
        else if (!extenderState && this.extenderState)
        {
            this.extenderRetractActivated = true;
        }
        this.extenderState = extenderState;
    }

    public boolean getTiltState()
    {
        return tiltState;
    }

    public void setTiltState(boolean tiltState)
    {
        if (tiltState && !this.tiltState)
        {
            this.tiltExtendActivated = true;
        }
        else if (!tiltState && this.tiltState)
        {
            this.tiltRetractActivated = true;
        }
        this.tiltState = tiltState;
    }

    public boolean getTromboneState()
    {
        return tromboneState;
    }

    public void setTromboneState(boolean tromboneState)
    {
        if (tromboneState && !this.tromboneState)
        {
            this.tromboneExtendActivated = true;
        }
        else if (!tromboneState && this.tromboneState)
        {
            this.tromboneRetractActivated = true;
        }
        this.tromboneState = tromboneState;
    }

    public boolean getExtenderExtendActivated()
    {
        boolean temp = this.extenderExtendActivated;
        this.extenderExtendActivated = false;
        return temp;
    }

    public boolean getExtenderRetractActivated()
    {
        boolean temp = this.extenderRetractActivated;
        this.extenderRetractActivated = false;
        return temp;
    }

    public boolean getTiltExtendActivated()
    {
        boolean temp = this.tiltExtendActivated;
        this.tiltExtendActivated = false;
        return temp;
    }

    public boolean getTiltRetractActivated()
    {
        boolean temp = this.tiltRetractActivated;
        this.tiltRetractActivated = false;
        return temp;
    }

    public boolean getTromboneExtendActivated()
    {
        boolean temp = this.tromboneExtendActivated;
        this.tromboneExtendActivated = false;
        return temp;
    }

    public boolean getTromboneRetractActivated()
    {
        boolean temp = this.tromboneRetractActivated;
        this.tromboneRetractActivated = false;
        return temp;
    }

}
