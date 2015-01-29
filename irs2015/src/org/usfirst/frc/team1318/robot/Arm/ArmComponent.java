package org.usfirst.frc.team1318.robot.Arm;

import org.usfirst.frc.team1318.robot.ElectronicsConstants;
import org.usfirst.frc.team1318.robot.Common.SmartDashboardLogger;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class ArmComponent
{
    private DoubleSolenoid trombone;
    private DoubleSolenoid tiltLinkage;
    private DoubleSolenoid extendLinkage;

    //logging constants
    private static final String TROMBONE_STATE_LOG_KEY = "dt.TromboneState";
    private static final String EXTEND_LINKAGE_STATE_LOG_KEY = "dt.ExtendLinkageState";
    private static final String TILT_LINKAGE_STATE_LOG_KEY = "dt.TiltLinkageState";

    //constructor uses constants for extension and retraction of 3 solenoids that make up the arm 
    public ArmComponent()
    {
        this.trombone = new DoubleSolenoid(ElectronicsConstants.TROMBONE_SOLANOID_EXTEND, ElectronicsConstants.TROMBONE_SOLANOID_RETRACT);
        this.tiltLinkage = new DoubleSolenoid(ElectronicsConstants.TILT_LINK_SOLANOID_EXTEND,
            ElectronicsConstants.TILT_LINK_SOLANOID_RETRACT);
        this.extendLinkage = new DoubleSolenoid(ElectronicsConstants.EXTEND_LINK_SOLANOID_EXTEND,
            ElectronicsConstants.EXTEND_LINK_SOLANOID_RETRACT);
    }

    public DoubleSolenoid getTrombone()
    {
        return this.trombone;
    }

    public DoubleSolenoid getTiltLinkage()
    {
        return this.tiltLinkage;
    }

    public DoubleSolenoid getExtendLinkage()
    {
        return this.extendLinkage;
    }

    /**
     * sets state of Trombone_Solenoid
     * @param true for extended state; false for retracted state
     */
    public void setTromboneSolenoidState(boolean state)
    {
        if (state)
        {
            this.trombone.set(Value.kForward);
        }
        else
        {
            this.trombone.set(Value.kReverse);

        }
        SmartDashboardLogger.putNumber(ArmComponent.TROMBONE_STATE_LOG_KEY, trombone.get().value);

    }

    /**
     * sets state of TiltLinkage_Solenoid
     * @param true for extended state; false for retracted state
     */
    public void setTiltLinkageSolenoidState(boolean state)
    {
        if (state)
        {
            this.tiltLinkage.set(Value.kForward);
        }
        else
        {
            this.tiltLinkage.set(Value.kReverse);
        }
        SmartDashboardLogger.putNumber(ArmComponent.TILT_LINKAGE_STATE_LOG_KEY, this.tiltLinkage.get().value);
    }

    /**
     * sets state of Extend_Linkage_Solenoid
     * @param true for extended state; false for retracted state
     */
    public void setExtendLinkageSolenoidState(boolean state)
    {
        if (state)
        {
            this.trombone.set(Value.kForward);
        }
        else
        {
            this.trombone.set(Value.kReverse);
        }

        SmartDashboardLogger.putNumber(ArmComponent.EXTEND_LINKAGE_STATE_LOG_KEY, this.extendLinkage.get().value);
    }

}
