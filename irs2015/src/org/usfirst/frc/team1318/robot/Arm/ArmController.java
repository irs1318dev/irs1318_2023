package org.usfirst.frc.team1318.robot.Arm;

import org.usfirst.frc.team1318.robot.TuningConstants;
import org.usfirst.frc.team1318.robot.Common.IController;
import org.usfirst.frc.team1318.robot.Common.IDriver;

import edu.wpi.first.wpilibj.Timer;

public class ArmController implements IController
{
    private enum ArmStates
    {
        STAGE_0, STAGE_1, STAGE_2, STAGE_3, STAGE_4;
    }

    private ArmStates armstateExtendor;
    private ArmStates armstateRetractor;

    private Timer timer;
    private double continueTime;

    private final ArmComponent component;
    private IDriver driver;

    private Boolean tromboneState;
    private Boolean tiltState;
    private Boolean extenderState;

    private boolean prevTiltHoldState;

    public ArmController(IDriver driver, ArmComponent component)
    {
        this.armstateExtendor = ArmStates.STAGE_0;
        this.armstateRetractor = ArmStates.STAGE_0;
        this.component = component;
        this.driver = driver;

        //in rest position: extendLinkage is extended, tiltLinkage is extended, trombone is retracted 
        this.tromboneState = null;
        this.tiltState = null;
        this.extenderState = null;

        this.prevTiltHoldState = false;

        //initialize new Timer but start it between actual stages of arm-movement
        this.timer = new Timer();
        this.timer.start();
    }

    @Override
    public void setDriver(IDriver driver)
    {
        this.driver = driver;
    }

    @Override
    public void update()
    {
        this.component.getExtendSensorTripped();
        /* 
         * Macro Extend operation will retract the extender (Stage 1), extend the trombone (Stage 2), and then retract the tilt (Stage 3)
         * Fully extends entire arm, starting with a check that tilts the arm up
         */
        switch (this.armstateExtendor)
        {
            case STAGE_0:
                if (this.driver.getArmMacroExtendButton())
                {
                    this.armstateExtendor = ArmStates.STAGE_1;
                    this.armstateRetractor = ArmStates.STAGE_0;
                }
                break;
            case STAGE_1:
                if (this.tiltState != null && this.tiltState == true)
                {
                    //don't wait for tilt to extend, necessary for the beginning of the macro
                    this.continueTime = this.timer.get();
                }
                else
                {
                    this.continueTime = this.timer.get();// + TuningConstants.ARM_TILT_EXTEND_WAIT_TIME;
                    this.tiltState = true;
                }
                this.armstateExtendor = ArmStates.STAGE_2;
                break;
            case STAGE_2:
                if (this.timer.get() >= this.continueTime)
                {
                    if (this.extenderState != null && this.extenderState == true)
                    {
                        //don't wait for extender to retract
                        this.continueTime = this.timer.get() + TuningConstants.ARM_SAFETY_WAIT;
                    }
                    else
                    {
                        this.continueTime = this.timer.get() + TuningConstants.ARM_EXTENDOR_EXTEND_WAIT_TIME;
                        this.extenderState = true;
                    }
                    this.armstateExtendor = ArmStates.STAGE_3;
                }
                break;
            case STAGE_3:
                if (this.timer.get() >= this.continueTime)
                {
                    if (this.tromboneState != null && this.tromboneState == true)
                    {
                        //don't wait for trombone to extend
                        this.continueTime = this.timer.get() + TuningConstants.ARM_SAFETY_WAIT;
                    }
                    else
                    {
                        this.continueTime = this.timer.get() + TuningConstants.ARM_TROMBONE_EXTEND_WAIT_TIME;
                        this.tromboneState = true;
                    }
                    this.armstateExtendor = ArmStates.STAGE_0;//ArmStates.STAGE_4;
                }
                break;
            case STAGE_4:
                if (this.timer.get() >= this.continueTime)
                {
                    //this.tiltState = false;
                    this.armstateExtendor = ArmStates.STAGE_0;
                }
                break;
            default:
                break;
        }

        /* 
         * Macro Retract operation extends the tilt (Stage 1), retracts the trombone (Stage 2), and extends the extender (Stage 3)
         * Returns arm to most compact position - SHOULD BE OK IF EXTENDER IS ALREADY COMPACT
         * The Retract operation takes precedence over the Extend operation.
         */
        switch (this.armstateRetractor)
        {
            case STAGE_0:
                if (this.driver.getArmMacroRetractButton())
                {
                    this.armstateRetractor = ArmStates.STAGE_1;
                    this.armstateExtendor = ArmStates.STAGE_0;
                }
                break;
            case STAGE_1:
                if (this.tiltState != null && this.tiltState == true)
                {
                    //don't wait for tilt to extend
                    this.continueTime = this.timer.get();// + TuningConstants.ARM_SAFETY_WAIT;
                }
                else
                {
                    this.continueTime = this.timer.get() + TuningConstants.ARM_TILT_RETRACT_WAIT_TIME;
                    this.tiltState = true;
                }
                this.armstateRetractor = ArmStates.STAGE_2;
                break;
            case STAGE_2:
                if (this.timer.get() >= this.continueTime)
                {
                    if (this.tromboneState != null && this.tromboneState == false)
                    {
                        //don't wait for trombone to retract
                        this.continueTime = this.timer.get() + TuningConstants.ARM_SAFETY_WAIT;
                    }
                    else
                    {
                        this.continueTime = this.timer.get() + TuningConstants.ARM_TROMBONE_RETRACT_WAIT_TIME;
                        this.tromboneState = false;
                    }
                    this.armstateRetractor = ArmStates.STAGE_3;
                }
                break;
            case STAGE_3:
                if (this.timer.get() >= this.continueTime)
                {
                    this.extenderState = false;
                    this.armstateRetractor = ArmStates.STAGE_0;
                }
                break;
            default:
                break;
        }

        /* 
         * Any override takes precedence over any macro
         * The retract overrides take precedence over the corresponding extend overrides.
         */

        if (this.driver.getArmTiltRetractHoldButton())
        {
            this.tiltState = false;
            this.prevTiltHoldState = true;
        }
        else if (prevTiltHoldState)
        {
            this.tiltState = true;
            this.prevTiltHoldState = false;
        }

        if (this.driver.getArmExtenderExtendOverride())
        {
            this.extenderState = true;
            this.armstateExtendor = ArmStates.STAGE_0;
            this.armstateRetractor = ArmStates.STAGE_0;
        }
        if (this.driver.getArmExtenderRetractOverride())
        {
            this.extenderState = false;
            this.armstateExtendor = ArmStates.STAGE_0;
            this.armstateRetractor = ArmStates.STAGE_0;
        }

        if (this.driver.getArmTiltExtendOverride())
        {
            this.tiltState = true;
            this.armstateExtendor = ArmStates.STAGE_0;
            this.armstateRetractor = ArmStates.STAGE_0;
        }
        if (this.driver.getArmTiltRetractOverride())
        {
            this.tiltState = false;
            this.armstateExtendor = ArmStates.STAGE_0;
            this.armstateRetractor = ArmStates.STAGE_0;
        }

        if (this.driver.getArmTromboneExtendOverride())
        {
            this.tromboneState = true;
            this.armstateExtendor = ArmStates.STAGE_0;
            this.armstateRetractor = ArmStates.STAGE_0;
        }
        if (this.driver.getArmTromboneRetractOverride())
        {
            this.tromboneState = false;
            this.armstateExtendor = ArmStates.STAGE_0;
            this.armstateRetractor = ArmStates.STAGE_0;
        }

        //actually set the solenoid states only once per update cycle in order to prevent opposite commands
        if (this.extenderState != null)
        {
            this.component.setExtendLinkageSolenoidState(this.extenderState);
        }

        if (this.tiltState != null)
        {
            this.component.setTiltLinkageSolenoidState(this.tiltState);
        }

        if (this.tromboneState != null)
        {
            this.component.setTromboneSolenoidState(this.tromboneState);
        }
    }

    @Override
    public void stop()
    {
    }
}
