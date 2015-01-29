package org.usfirst.frc.team1318.robot.Arm;

import org.usfirst.frc.team1318.robot.Common.IController;
import org.usfirst.frc.team1318.robot.Common.IDriver;

public class ArmController implements IController
{
    ArmComponent component;
    IDriver driver;

    public ArmController(ArmComponent component, IDriver driver)
    {
        this.component = component;
        this.driver = driver;
    }

    @Override
    public void update()
    {
        this.component.setExtendLinkageSolenoidState(this.driver.getArmExtenderToggleOverride());

        this.component.setTiltLinkageSolenoidState(this.driver.getArmTiltToggleOverride());

        this.component.setTromboneSolenoidState(this.driver.getArmTromboneToggleOverride());

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
                }*/

    }

    @Override
    public void stop()
    {
        this.component.setExtendLinkageSolenoidState(false);
        this.component.setTiltLinkageSolenoidState(false);
        this.component.setTromboneSolenoidState(false);
    }
}
