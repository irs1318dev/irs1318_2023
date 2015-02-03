package org.usfirst.frc.team1318.robot.Arm;

import java.util.Date;

import org.usfirst.frc.team1318.robot.Common.IController;
import org.usfirst.frc.team1318.robot.Common.IDriver;

public class ArmController implements IController
{
    private enum ArmStates
    {
        STAGE_0, STAGE_1, STAGE_2, STAGE_3
    };

    private ArmStates armstateExtendor;
    private ArmStates armstateRetractor;

    private Date timer;
    private long startTime;
    private final long EXTEND_WAIT_TIME = 7000;
    private final long RETRACT_WAIT_TIME = 3000;

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

        this.tromboneState = false;
        this.tiltState = false;
        this.extenderState = true;
    }

    @Override
    public void update()
    {
        //when are MacroButtons being reset? AFTER THEY ARE GOTTEN

        //in rest position: extendLinkage is open, trombone is closed, tiltLinkage is closed 
        //Macro Retract state machine
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
                this.tiltState = false;

                this.startTime = this.timer.getTime();
                this.armstateRetractor = ArmStates.STAGE_2;
                break;
            case STAGE_2:
                if (this.timer.getTime() - this.startTime >= this.RETRACT_WAIT_TIME)
                {
                    this.armstateRetractor = ArmStates.STAGE_3;
                }
                break;
            case STAGE_3:
                this.extenderState = true;
                this.tromboneState = false;

                this.armstateRetractor = ArmStates.STAGE_0;
                break;
            default:
                break;
        }

        //Macro Extend state machine
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
                this.extenderState = false;
                this.tromboneState = true;

                this.startTime = this.timer.getTime();
                this.armstateExtendor = ArmStates.STAGE_2;
                break;
            case STAGE_2:
                if (this.timer.getTime() - this.startTime >= this.EXTEND_WAIT_TIME)
                {
                    this.armstateExtendor = ArmStates.STAGE_3;
                }
                break;
            case STAGE_3:
                this.tiltState = true;
                this.armstateExtendor = ArmStates.STAGE_0;
                break;
            default:
                break;
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

        this.component.setExtendLinkageSolenoidState(extenderState);
        this.component.setTiltLinkageSolenoidState(tiltState);
        this.component.setTromboneSolenoidState(tromboneState);
    }

    @Override
    public void stop()
    {
        this.component.setExtendLinkageSolenoidState(false);
        this.component.setTiltLinkageSolenoidState(false);
        this.component.setTromboneSolenoidState(false);
    }
}
