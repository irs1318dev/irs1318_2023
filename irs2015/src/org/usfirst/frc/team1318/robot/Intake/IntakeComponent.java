package org.usfirst.frc.team1318.robot.Intake;

import org.usfirst.frc.team1318.robot.ElectronicsConstants;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Talon;

/* buttons 
 * up
 * down
 * toggle left 
 * toggle right
 * press and hold for in
 * press and hold for out
 * */

public class IntakeComponent
{

    private Talon motorLeft;
    private Talon motorRight;
    private DoubleSolenoid intakeDoubleSolenoidLeft;
    private DoubleSolenoid intakeDoubleSolenoidRight;

    public IntakeComponent(int motorLeftConstant, int motorRightConstant,
        int collectorSolenoidConstantLeft, int collectorSolenoidConstantRight)
    {
        this.motorLeft = new Talon(motorLeftConstant);
        this.motorRight = new Talon(motorRightConstant);
        this.intakeDoubleSolenoidLeft = new DoubleSolenoid(ElectronicsConstants.SOLENOID_MODULE_PORT_1,
            ElectronicsConstants.SOLENOID_MODULE_PORT_2);
        this.intakeDoubleSolenoidRight = new DoubleSolenoid(ElectronicsConstants.SOLENOID_MODULE_PORT_3,
            ElectronicsConstants.SOLENOID_MODULE_PORT_4);
    }

    /**
     * Method to raise
    */

    public void setIntake(boolean shouldForward)
    {
        if (shouldForward)
        {
            intakeDoubleSolenoidLeft.set(Value.kForward);
            intakeDoubleSolenoidRight.set(Value.kForward);
        }
        else
        {
            intakeDoubleSolenoidLeft.set(Value.kReverse);
            intakeDoubleSolenoidRight.set(Value.kReverse);
        }

    }

    /**
     * Method to lower
    */

    //    public void lowerIntake()
    //    {
    //        intakeDoubleSolenoidRight.set(false);
    //        intakeDoubleSolenoidLeft.set(false);
    //    }

    /**
     * Method left intake solenoid toggle
    */

    public void toggleLeftIntake()
    {
        intakeDoubleSolenoidLeft.set(!intakeDoubleSolenoidLeft.get());
        //SmartDashboardLogger.
    }

    /**
     * Method right intake solenoid toggle
    */

    public void toggleRightIntake()
    {
        intakeDoubleSolenoidRight.set(!intakeDoubleSolenoidRight.get());
    }

    /**
     * sets the intake motors speed
     * @args A double to represent speed from -1 to 1
     * values < 0 are inwards, values > 0 forwards, 0 motor off
    */

    public void setIntakeMotorSpeed(double speed)
    {
        this.motorLeft.set(speed);
        this.motorRight.set(speed);
    }
}
