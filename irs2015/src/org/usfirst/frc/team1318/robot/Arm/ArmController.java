package org.usfirst.frc.team1318.robot.Arm;

import org.usfirst.frc.team1318.robot.Common.IDriver;

public class ArmController
{
    ArmComponent component;
    IDriver driver;

    public ArmController(ArmComponent component, IDriver driver)
    {
        this.component = component;
        this.driver = driver;
    }
}
