package frc.robot.driver.controltasks;

//import frc.robot.TuningConstants;
import frc.robot.driver.controltasks.ArmLAPositionTask.IntakeState;
import frc.robot.mechanisms.ArmMechanism;

public class CheckArmPositionTask extends DecisionSequentialTask
{
    private final double lowerExtensionLength;
    private final double upperExtensionLength;
    private final IntakeState desiredState;
    private final double minimumZ;

    private ArmMechanism arm;

    public CheckArmPositionTask(
        double lowerExtensionLength,
        double upperExtensionLength,
        IntakeState state,
        double minimumZ)
    {
        super();

        this.lowerExtensionLength = lowerExtensionLength;
        this.upperExtensionLength = upperExtensionLength;
        this.desiredState = state;

        this.minimumZ = minimumZ;
    }

    @Override
    public void begin()
    {
        super.begin();
        this.arm = this.getInjector().getInstance(ArmMechanism.class);

        double fkZ = this.arm.getFKZPosition();
        if (fkZ < this.minimumZ)
        {
            this.AppendTask(new ArmLAPositionTask(this.lowerExtensionLength, this.upperExtensionLength, this.desiredState));
        }
    }
}
