package org.usfirst.frc.team1318.robot.Arm;

import org.usfirst.frc.team1318.robot.ElectronicsConstants;
import org.usfirst.frc.team1318.robot.Common.SmartDashboardLogger;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class ArmComponent
{
    public DoubleSolenoid trombone;
    public DoubleSolenoid moveLinkage;
    public DoubleSolenoid extendLinkage;

    //logging constants
    private static final String TROMBONE_STATE_LOG_KEY = "dt.TromboneState";
    private static final String MOVE_LINKAGE_STATE_LOG_KEY = "dt.MoveLinkageState";
    private static final String EXTEND_LINKAGE_STATE_LOG_KEY = "dt.ExtendLinkageState";

    //constructor uses constants for extension and retraction of 3 solenoids that make up the arm 
    public ArmComponent()
    {
        this.trombone = new DoubleSolenoid(ElectronicsConstants.TROMBONE_SOLANOID_EXTEND, ElectronicsConstants.TROMBONE_SOLANOID_RETRACT);
        this.moveLinkage = new DoubleSolenoid(ElectronicsConstants.MOVE_LINK_SOLANOID_EXTEND,
            ElectronicsConstants.MOVE_LINK_SOLANOID_RETRACT);
        this.extendLinkage = new DoubleSolenoid(ElectronicsConstants.EXTEND_LINK_SOLANOID_EXTEND,
            ElectronicsConstants.EXTEND_LINK_SOLANOID_RETRACT);
    }

    /**
     * changes state of Trombone_Solenoid
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
     * changes state of TiltLinkage_Solenoid
     * @param true for extended state; false for retracted state
     */
    public void setMoveLinkageSolenoidState(boolean state)
    {
        if (state)
        {
            this.trombone.set(Value.kForward);
        }
        else
        {
            this.trombone.set(Value.kReverse);
        }
        SmartDashboardLogger.putNumber(ArmComponent.MOVE_LINKAGE_STATE_LOG_KEY, moveLinkage.get().value);
    }

    /**
     * changes state of Extend_Linkage_Solenoid
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

        SmartDashboardLogger.putNumber(ArmComponent.EXTEND_LINKAGE_STATE_LOG_KEY, extendLinkage.get().value);
    }

}
