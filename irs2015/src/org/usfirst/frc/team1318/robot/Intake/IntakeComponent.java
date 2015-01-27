package org.usfirst.frc.team1318.robot.Intake;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Talon;

/* buttons 
 * up
 * down
 * toggle up and down
 * press and hold for in
 * press and hold for out
 * */

public class IntakeComponent
{

    private Talon motorLeft;
    private Talon motorRight;
    private Solenoid intakeSolenoidLeft;
    private Solenoid intakeSolenoidRight;
    private double INTAKE_SPEED = .7;

    public IntakeComponent(int motorLeftConstant, int motorRightConstant,
        int collectorSolenoidConstantLeft, int collectorSolenoidConstantRight)
    {
        this.motorLeft = new Talon(motorLeftConstant);
        this.motorRight = new Talon(motorRightConstant);
        this.intakeSolenoidLeft = new Solenoid(collectorSolenoidConstantLeft);
        this.intakeSolenoidRight = new Solenoid(collectorSolenoidConstantRight);
    }

    /**
     * Method to raise
    */

    public void raiseIntake()
    {
        intakeSolenoidLeft.set(true);
        intakeSolenoidRight.set(true);
    }

    /**
     * Method to lower
    */

    public void lowerIntake()
    {
        intakeSolenoidRight.set(false);
        intakeSolenoidLeft.set(false);
    }

    /**
     * Method left intake solenoid toggle
    */

    public void toggleLeftIntake()
    {
        intakeSolenoidLeft.set(!intakeSolenoidLeft.get());
    }

    /**
     * Method right intake solenoid toggle
    */

    public void toggleRightIntake()
    {
        intakeSolenoidRight.set(!intakeSolenoidRight.get());
    }

    /**
     * Method intake in
    */

    public void intakeIn()
    {
        this.motorLeft.set(INTAKE_SPEED);
        this.motorRight.set(INTAKE_SPEED);
    }

    /**
     * Method intake out
    */

    public void intakeOut()
    {
        this.motorLeft.set(-INTAKE_SPEED);
        this.motorRight.set(-INTAKE_SPEED);
    }

    /**
     * Method intake off
     */
    public void intakeOff()
    {
        this.motorLeft.set(0);
        this.motorRight.set(0);
    }
}
