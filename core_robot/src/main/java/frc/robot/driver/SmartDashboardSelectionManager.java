package frc.robot.driver;

import com.google.inject.Inject;
import com.google.inject.Singleton;

import frc.robot.common.robotprovider.*;

@Singleton
public class SmartDashboardSelectionManager
{
    private final ISendableChooser<StartPosition> positionChooser;
    private final ISendableChooser<AutoRoutine> routineChooser;

    public enum StartPosition
    {
        Center,
        Left,
        Right
    }

    public enum AutoRoutine
    {
        None,
        MiddleOnePlusCharge,
        LoadOnePlusOne,
        LoadOnePlusCharge,
        GuardOnePlusOne,
        GuardOnePlusCharge
    }

    /**
     * Initializes a new SmartDashboardSelectionManager
     */
    @Inject
    public SmartDashboardSelectionManager(
        IRobotProvider provider)
    {
        INetworkTableProvider networkTableProvider = provider.getNetworkTableProvider();

        this.routineChooser = networkTableProvider.getSendableChooser();
        this.routineChooser.addDefault("None", AutoRoutine.None);
        this.routineChooser.addObject("Load One Charge", AutoRoutine.LoadOnePlusCharge);
        this.routineChooser.addObject("Load Two", AutoRoutine.LoadOnePlusOne);
        this.routineChooser.addObject("Guard One Charge", AutoRoutine.GuardOnePlusCharge);
        this.routineChooser.addObject("Guard Two", AutoRoutine.GuardOnePlusOne);
        this.routineChooser.addObject("Middle One Charge", AutoRoutine.MiddleOnePlusCharge);
        networkTableProvider.addChooser("Auto Routine", this.routineChooser);

        this.positionChooser = networkTableProvider.getSendableChooser();
        this.positionChooser.addDefault("center", StartPosition.Center);
        this.positionChooser.addObject("left", StartPosition.Left);
        this.positionChooser.addObject("right", StartPosition.Right);
        networkTableProvider.addChooser("Start Position", this.positionChooser);
    }

    public StartPosition getSelectedStartPosition()
    {
        return SmartDashboardSelectionManager.GetSelectedOrDefault(this.positionChooser, StartPosition.Center);
    }

    public AutoRoutine getSelectedAutoRoutine()
    {
        return SmartDashboardSelectionManager.GetSelectedOrDefault(this.routineChooser, AutoRoutine.None);
    }

    private static <T> T GetSelectedOrDefault(ISendableChooser<T> chooser, T defaultValue)
    {
        T selected = chooser.getSelected();
        if (selected == null)
        {
            selected = defaultValue;
        }

        return selected;
    }
}
