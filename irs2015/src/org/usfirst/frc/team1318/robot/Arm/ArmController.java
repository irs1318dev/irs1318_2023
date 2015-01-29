package org.usfirst.frc.team1318.robot.Arm;

import org.usfirst.frc.team1318.robot.Common.IController;
import org.usfirst.frc.team1318.robot.Common.IDriver;

public class ArmController implements IController
{
    private ArmComponent component;
    private IDriver driver;

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
    }

    @Override
    public void stop()
    {
        this.component.setExtendLinkageSolenoidState(false);
        this.component.setTiltLinkageSolenoidState(false);
        this.component.setTromboneSolenoidState(false);
    }
}
