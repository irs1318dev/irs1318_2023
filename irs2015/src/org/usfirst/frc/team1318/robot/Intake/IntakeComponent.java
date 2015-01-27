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
    private Solenoid intakeSolenoidUp;
    private Solenoid intakeSolenoidDown;
    private double INTAKE_SPEED = .5;

    public IntakeComponent(int motorLeftConstant, int motorRightConstant,
        int collectorSolenoidConstantForward, int collectorSolenoidConstantReverse)
    {
        this.motorLeft = new Talon(motorLeftConstant);
        this.motorRight = new Talon(motorRightConstant);
        this.intakeSolenoidUp = new Solenoid(collectorSolenoidConstantForward);
        this.intakeSolenoidDown = new Solenoid(collectorSolenoidConstantReverse);
    }

    /**
     * Method to raise
    */

    public void raiseIntake()
    {
        intakeSolenoidUp.set(true);
        intakeSolenoidDown.set(false);
    }

    /**
     * Method to lower
    */

    public void lowerIntake()
    {
        intakeSolenoidDown.set(true);
        intakeSolenoidUp.set(false);
    }

    /**
     * Changes the state of the solenoids causing the intake to switch from
     * upwards movement to downwards movement
    */

    public void toggleIntakeRaise()
    {
        intakeSolenoidUp.set(!intakeSolenoidUp.get());
        intakeSolenoidDown.set(!intakeSolenoidDown.get());
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
