package org.usfirst.frc.team1318.robot.Arm;

import org.usfirst.frc.team1318.robot.Common.IController;
import org.usfirst.frc.team1318.robot.Common.IDriver;

import edu.wpi.first.wpilibj.Timer;

public class ArmController implements IController
{
    private enum ArmStates
    {
        STAGE_0, STAGE_1, STAGE_2, STAGE_3
    };

    private ArmStates armstateExtendor;
    private ArmStates armstateRetractor;

    private Timer timer;
    private double startTime;
    private final double PARTIAL_EXTEND_WAIT_TIME = 5;
    private final double FULL_EXTEND_WAIT_TIME = 5;
    private final double PARTIAL_RETRACT_WAIT_TIME = 5;
    private final double FULL_RETRACT_WAIT_TIME = 5;
    private final double SAFETY_WAIT = 2;

    private ArmComponent component;
    private IDriver driver;

    private boolean tromboneState;
    private boolean tiltState;
    private boolean extenderState;

    public ArmController(ArmComponent component, IDriver driver)
    {
        this.armstateExtendor = ArmStates.STAGE_0;
        this.armstateRetractor = ArmStates.STAGE_0;
        this.component = component;
        this.driver = driver;

        //in rest position: extendLinkage is extended, tiltLinkage is extended, trombone is retracted 
        this.tromboneState = false;
        this.tiltState = true;
        this.extenderState = true;

        //initialize new Timer but start it between actual stages of arm-movement
        this.timer = new Timer();
        this.timer.start();
    }

    @Override
    public void update()
    {
        /*WILL"S SUGGESTION FOR FUTURE CONSIDERATION: to check all of the buttons and determine the current user intent 
         * (i.e. first check for extend macro, then retract macro, then each of the extend/retract overrides)
         * before calculating any action you want to perform -  as opposed to currently convoluted logic
         */

        /* 
         * Macro Extend operation will retract the extender (Stage 1), extend the trombone (Stage 2), and then retract the tilt (Stage 3)
         * Fully extends entire arm
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
                if (this.extenderState == false)
                {
                    //don't wait for extender to retract
                    this.startTime = this.timer.get() + this.PARTIAL_EXTEND_WAIT_TIME - this.SAFETY_WAIT;
                }
                else
                {
                    this.startTime = this.timer.get();
                    this.extenderState = false;
                }
                this.armstateExtendor = ArmStates.STAGE_2;
                break;
            case STAGE_2:
                if (this.timer.get() - this.startTime >= this.PARTIAL_EXTEND_WAIT_TIME)
                {
                    if (this.tromboneState == true)
                    {
                        //don't wait for trombone to extend
                        this.startTime = this.timer.get() + this.FULL_EXTEND_WAIT_TIME - this.SAFETY_WAIT;
                    }
                    else
                    {
                        this.startTime = this.timer.get();
                        this.tromboneState = true;
                    }
                    this.armstateExtendor = ArmStates.STAGE_3;
                }
                break;
            case STAGE_3:
                if (this.timer.get() - this.startTime >= this.FULL_EXTEND_WAIT_TIME)
                {
                    this.tiltState = false;
                    this.armstateExtendor = ArmStates.STAGE_0;
                }
                break;
            default:
                break;
        }

        /* 
         * Macro Retract operation extends the tilt (Stage 1), retracts the trombone (Stage 2), and extends the extender (Stage 3)
         * Returns arm to most compact position
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
                if (this.tiltState == true)
                {
                    //don't wait for tilt to extend
                    this.startTime = this.timer.get() + this.PARTIAL_RETRACT_WAIT_TIME - this.SAFETY_WAIT;
                }
                else
                {
                    this.startTime = this.timer.get();
                    this.tiltState = true;
                }
                this.armstateRetractor = ArmStates.STAGE_2;
                break;
            case STAGE_2:
                if (this.timer.get() - this.startTime >= this.PARTIAL_RETRACT_WAIT_TIME)
                {
                    if (this.tromboneState == false)
                    {
                        //don't wait for trombone to retract
                        this.startTime = this.timer.get() + this.FULL_RETRACT_WAIT_TIME - this.SAFETY_WAIT;
                    }
                    else
                    {
                        this.startTime = this.timer.get();
                        this.tromboneState = false;
                    }
                    this.armstateRetractor = ArmStates.STAGE_3;
                }
                break;
            case STAGE_3:
                if (this.timer.get() - this.startTime >= this.FULL_RETRACT_WAIT_TIME)
                {
                    this.extenderState = true;
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
        this.component.setExtendLinkageSolenoidState(extenderState);
        this.component.setTiltLinkageSolenoidState(tiltState);
        this.component.setTromboneSolenoidState(tromboneState);
    }

    @Override
    //return to completely retracted position - NOT SURE??
    public void stop()
    {
        this.component.setExtendLinkageSolenoidState(true);
        this.component.setTiltLinkageSolenoidState(true);
        this.component.setTromboneSolenoidState(false);
    }
}
