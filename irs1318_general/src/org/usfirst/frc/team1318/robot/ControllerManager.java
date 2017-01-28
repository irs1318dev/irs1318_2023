package org.usfirst.frc.team1318.robot;

import java.util.ArrayList;

import org.usfirst.frc.team1318.robot.common.IController;
import org.usfirst.frc.team1318.robot.compressor.CompressorController;
import org.usfirst.frc.team1318.robot.driver.Driver;
import org.usfirst.frc.team1318.robot.drivetrain.DriveTrainController;

public class ControllerManager implements IController
{
    public final ComponentManager components;
    public final ArrayList<IController> controllerList;

    public ControllerManager(ComponentManager components)
    {
        this.components = components;
        this.controllerList = new ArrayList<IController>();
        this.controllerList.add(this.components.getPowerManager());
        this.controllerList.add(this.components.getPositionManager());
        this.controllerList.add(new CompressorController(this.components.getCompressor()));
        this.controllerList.add(new DriveTrainController(this.components.getDriveTrain(), TuningConstants.DRIVETRAIN_USE_PID_DEFAULT));
    }

    @Override
    public void update()
    {
        for (IController controller : this.controllerList)
        {
            controller.update();
        }
    }

    @Override
    public void stop()
    {
        for (IController controller : this.controllerList)
        {
            controller.stop();
        }
    }

    @Override
    public void setDriver(Driver driver)
    {
        for (IController controller : this.controllerList)
        {
            controller.setDriver(driver);
        }
    }
}
