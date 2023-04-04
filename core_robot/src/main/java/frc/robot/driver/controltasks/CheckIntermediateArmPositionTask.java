package frc.robot.driver.controltasks;

//import frc.robot.TuningConstants;
import frc.robot.driver.controltasks.ArmMMPositionTask.IntakeState;
import frc.robot.mechanisms.ArmMechanism;

public class CheckIntermediateArmPositionTask extends DecisionSequentialTask
{
    private ArmMechanism arm;
    private final IntakeState desiredState;
    private final double lowerExtensionLength;
    private final double upperExtensionLength;
    public CheckIntermediateArmPositionTask(double lowerExtensionLength, double upperExtensionLength, IntakeState state)
    {
        super();
        this.lowerExtensionLength = lowerExtensionLength;
        this.upperExtensionLength = upperExtensionLength;
        this.desiredState = state;
    }

    @Override
    public void begin()
    {
        super.begin();
        this.arm = this.getInjector().getInstance(ArmMechanism.class);

        double fkZ = this.arm.getFKZPosition();
        if (fkZ < upperExtensionLength) //Arm Upper Position Approach
        {
            this.AppendTask(new ArmMMPositionTask(lowerExtensionLength, upperExtensionLength, IntakeState.Up));
        }

        else
        {
            return;
        }
    }
}
