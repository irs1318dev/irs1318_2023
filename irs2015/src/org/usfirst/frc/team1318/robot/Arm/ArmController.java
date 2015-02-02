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

    private ArmStates armstate;

    private Date timer;
    private long startTime;
    private final long EXTEND_WAIT_TIME = 7000;

    private ArmComponent component;
    private IDriver driver;

    private boolean tromboneState;
    private boolean tiltState;
    private boolean extenderState;

    public ArmController(ArmComponent component, IDriver driver)
    {
        this.armstate = ArmStates.STAGE_0;
        this.component = component;
        this.driver = driver;

        tromboneState = false;
        tiltState = false;
        extenderState = false;
    }

    @Override
    public void update()
    {

        switch (this.armstate)
        {
            case STAGE_0:
                if (this.driver.getArmMacroExtendButton())
                {
                    this.armstate = ArmStates.STAGE_1;
                }
                break;
            case STAGE_1:
                this.component.setExtendLinkageSolenoidState(false);
                this.component.setTromboneSolenoidState(true);

                this.startTime = this.timer.getTime();
                this.armstate = ArmStates.STAGE_2;

                break;
            case STAGE_2:
                if (this.timer.getTime() - this.startTime >= this.EXTEND_WAIT_TIME)
                {
                    this.armstate = ArmStates.STAGE_3;
                }
                break;
            case STAGE_3:
                this.component.setTiltLinkageSolenoidState(true);
                this.armstate = ArmStates.STAGE_0;
                break;
            default:
                break;
        }

        if (this.driver.getArmExtenderExtendOverride())
        {
            this.extenderState = true;
        }
        if (this.driver.getArmExtenderRetractOverride())
        {
            this.extenderState = false;
        }

        if (this.driver.getArmTiltExtendOverride())
        {
            this.tiltState = true;
        }
        if (this.driver.getArmTiltRetractOverride())
        {
            this.tiltState = false;
        }

        if (this.driver.getArmTromboneExtendOverride())
        {
            this.tromboneState = true;
        }
        if (this.driver.getArmTromboneRetractOverride())
        {
            this.tromboneState = false;
        }

        this.component.setExtendLinkageSolenoidState(extenderState);
        this.component.setTiltLinkageSolenoidState(tiltState);
        this.component.setTromboneSolenoidState(tromboneState);
    }

    /*//toggle state of ArmExtender solenoid if button pressed
    if (this.driver.getArmExtenderToggleOverride())
    {
        switch (this.component.extendLinkage.get().value)
        {
            case Value.kForward_val:
                component.setExtendLinkageSolenoidState(false);
                break;
            case Value.kReverse_val:
                component.setExtendLinkageSolenoidState(true);
                break;
        }
    }
    =======
    /*        toggle state of ArmExtender solenoid if button pressed
            if (this.driver.getArmExtenderToggleOverride())
    =======
    this.component.setTiltLinkageSolenoidState(this.driver.getArmTiltToggleOverride());
    this.component.setTromboneSolenoidState(this.driver.getArmTromboneToggleOverride());

    /*        this.component.setExtendLinkageSolenoidState(this.driver.getArmExtenderToggleOverride());
                    toggle state of ArmExtender solenoid if button pressed
                    if (this.driver.getArmExtenderToggleOverride())
                    {
                        switch (this.component.extendLinkage.get().value)
                        {
                            case Value.kForward_val:
                                component.setExtendLinkageSolenoidState(false);
                                break;
                            case Value.kReverse_val:
                                component.setExtendLinkageSolenoidState(true);
                                break;
                        }
                    }

            //toggle state of ArmTilting solenoid if button pressed
            if (this.driver.getArmTiltToggleOverride())
    >>>>>>> branch 'Arm' of https://github.com/irs1318dev/irs1318_2015.git
            {
                switch (this.component.moveLinkage.get().value)
                {
                    case Value.kForward_val:
                        component.setMoveLinkageSolenoidState(false);
                        break;
                    case Value.kReverse_val:
                        component.setMoveLinkageSolenoidState(true);
                        break;
                }
            }

    <<<<<<< HEAD
    }
    =======
            //toggle state of Trombone solenoid if button pressed
            if (this.driver.getArmTromboneToggleOverride())
            {
                switch (this.component.trombone.get().value)
                {
                    case Value.kForward_val:
                        component.setTromboneSolenoidState(false);
                        break;
                    case Value.kReverse_val:
                        component.setTromboneSolenoidState(true);
                        break;
                }
            }*/
    //}

    @Override
    public void stop()
    {
        this.component.setExtendLinkageSolenoidState(false);
        this.component.setTiltLinkageSolenoidState(false);
        this.component.setTromboneSolenoidState(false);
    }
}
