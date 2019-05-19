package frc.robot.driver.controltasks;

import java.util.List;

import frc.robot.HardwareConstants;
import frc.robot.common.robotprovider.*;
import frc.robot.driver.Operation;
import frc.robot.driver.PathManager;
import frc.robot.driver.common.IControlTask;
import frc.robot.driver.common.PathStep;
import frc.robot.mechanisms.DriveTrainMechanism;
import frc.robot.mechanisms.PositionManager;

public class FollowPathTask extends ControlTaskBase implements IControlTask
{
    private static final double Timestep = 0.01;
    private final String pathName;

    protected ITimer timer;

    private double startTime;
    private double startLeftPosition;
    private double startRightPosition;
    private double startHeading;

    private PositionManager positionManager;
    private List<PathStep> path;
    private double duration;

    /**
     * Initializes a new FollowPathTask
     */
    public FollowPathTask(String pathName)
    {
        this.pathName = pathName;
    }

    /**
     * Begin the current task
     */
    @Override
    public void begin()
    {
        this.timer = this.getInjector().getInstance(ITimer.class);
        this.startTime = this.timer.get();

        DriveTrainMechanism driveTrain = this.getInjector().getInstance(DriveTrainMechanism.class);
        this.startLeftPosition = driveTrain.getLeftPosition();
        this.startRightPosition = driveTrain.getRightPosition();

        this.positionManager = this.getInjector().getInstance(PositionManager.class);
        this.startHeading = this.positionManager.getNavxAngle();

        PathManager pathManager = this.getInjector().getInstance(PathManager.class);
        this.path = pathManager.getPath(this.pathName);
        this.duration = this.path.size() * FollowPathTask.Timestep;

        this.setDigitalOperationState(Operation.DriveTrainUsePathMode, true);
        this.setAnalogOperationState(Operation.DriveTrainLeftPosition, this.startLeftPosition);
        this.setAnalogOperationState(Operation.DriveTrainRightPosition, this.startRightPosition);
        this.setAnalogOperationState(Operation.DriveTrainLeftVelocity, 0.0);
        this.setAnalogOperationState(Operation.DriveTrainRightVelocity, 0.0);
        this.setAnalogOperationState(Operation.DriveTrainHeadingCorrection, 0.0);
    }

    /**
     * Run an iteration of the current task and apply any control changes
     */
    @Override
    public void update()
    {
        double elapsedTime = this.timer.get() - this.startTime;
        if (elapsedTime > this.duration)
        {
            elapsedTime = this.duration;
        }

        double currentIndex = Math.floor(elapsedTime / FollowPathTask.Timestep);
        if (currentIndex >= this.path.size())
        {
            currentIndex = this.path.size() - 1;
        }

        double currentHeading = this.positionManager.getNavxAngle();

        PathStep step = this.path.get((int)currentIndex);
        double leftGoalPosition = step.getLeftPosition() * HardwareConstants.DRIVETRAIN_LEFT_TICKS_PER_INCH;
        double rightGoalPosition = step.getRightPosition() * HardwareConstants.DRIVETRAIN_RIGHT_TICKS_PER_INCH;
        this.setAnalogOperationState(Operation.DriveTrainLeftPosition, this.startLeftPosition + leftGoalPosition);
        this.setAnalogOperationState(Operation.DriveTrainRightPosition, this.startRightPosition + rightGoalPosition);
        this.setAnalogOperationState(Operation.DriveTrainLeftVelocity, step.getLeftVelocity());
        this.setAnalogOperationState(Operation.DriveTrainRightVelocity, step.getRightVelocity());
        this.setAnalogOperationState(Operation.DriveTrainHeadingCorrection, (this.startHeading + step.getHeading()) - currentHeading);
    }

    /**
     * Cancel the current task and clear control changes
     */
    @Override
    public void stop()
    {
        super.stop();

        this.setDigitalOperationState(Operation.DriveTrainUsePathMode, false);
        this.setAnalogOperationState(Operation.DriveTrainLeftPosition, 0.0);
        this.setAnalogOperationState(Operation.DriveTrainRightPosition, 0.0);
        this.setAnalogOperationState(Operation.DriveTrainLeftVelocity, 0.0);
        this.setAnalogOperationState(Operation.DriveTrainRightVelocity, 0.0);
        this.setAnalogOperationState(Operation.DriveTrainHeadingCorrection, 0.0);
    }

    /**
     * End the current task and reset control changes appropriately
     */
    @Override
    public void end()
    {
        this.setDigitalOperationState(Operation.DriveTrainUsePathMode, false);
        this.setAnalogOperationState(Operation.DriveTrainLeftPosition, 0.0);
        this.setAnalogOperationState(Operation.DriveTrainRightPosition, 0.0);
        this.setAnalogOperationState(Operation.DriveTrainLeftVelocity, 0.0);
        this.setAnalogOperationState(Operation.DriveTrainRightVelocity, 0.0);
        this.setAnalogOperationState(Operation.DriveTrainHeadingCorrection, 0.0);
    }

    @Override
    public boolean hasCompleted()
    {
        return this.timer.get() - this.startTime >= this.duration;
    }
}
