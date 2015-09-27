package org.usfirst.frc.team1318.robot.Driver.States;

import org.usfirst.frc.team1318.robot.Driver.Descriptions.AnalogOperationDescription;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Joystick.AxisType;

public class AnalogOperationState extends OperationState
{
    private double currentValue;

    public AnalogOperationState(AnalogOperationDescription description)
    {
        super(description);

        this.currentValue = 0.0;
    }

    /**
     * Sets whether the current operation is being interrupted by a macro
     * @param enable value of true indicates that we are interrupted
     */
    @Override
    public void setInterrupt(boolean enable)
    {
    }

    /**
     * Update the operation state based on the driver and co-driver joysticks 
     * @param driver joystick to update from
     * @param coDriver joystick to update from
     */
    @Override
    public void update(Joystick driver, Joystick coDriver)
    {
        AnalogOperationDescription description = (AnalogOperationDescription)this.getDescription();

        Joystick relevantJoystick;
        AxisType relevantAxis;
        switch (description.getUserInputDevice())
        {
            case None:
                return;

            case Driver:
                relevantJoystick = driver;
                break;

            case CoDriver:
                relevantJoystick = coDriver;
                break;

            default:
                throw new RuntimeException("unexpected user input device " + description.getUserInputDevice().toString());
        }

        switch (description.getUserInputDeviceAxis())
        {
            case None:
                return;

            case X:
                relevantAxis = AxisType.kX;
                break;

            case Y:
                relevantAxis = AxisType.kY;
                break;

            case Z:
                relevantAxis = AxisType.kZ;
                break;

            case Twist:
                relevantAxis = AxisType.kTwist;
                break;

            case Throttle:
                relevantAxis = AxisType.kThrottle;
                break;

            default:
                throw new RuntimeException("unknown axis type " + description.getUserInputDeviceAxis());
        }

        this.currentValue = relevantJoystick.getAxis(relevantAxis);
    }

    public double getState()
    {
        return this.currentValue;
    }
}
