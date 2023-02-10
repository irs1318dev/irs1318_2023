package frc.robot.driver.controltasks;

import frc.robot.TuningConstants;
import frc.robot.common.PIDHandler;
import frc.robot.common.robotprovider.ITimer;
import frc.robot.driver.*;

public abstract class VisionMoveAndTurnTaskBase extends VisionTurningTask
{
    public enum MoveType
    {
        Forward,
        AprilTagStrafe
    }

    private final MoveType translateType;
    private final boolean useFastMode;
    private final boolean verifyAngle;

    private PIDHandler movePIDHandler;

    /**
    * Initializes a new VisionAdvanceAndCenterTaskBase
    */
    protected VisionMoveAndTurnTaskBase(boolean useFastMode, TurnType rotateType, MoveType translateType, boolean bestEffort, boolean verifyAngle)
    {
        super(false, rotateType, bestEffort);

        this.translateType = translateType;

        this.useFastMode = useFastMode;
        this.verifyAngle = verifyAngle;
        this.movePIDHandler = null;
    }

    /**
     * Begin the current task
     */
    @Override
    public void begin()
    {
        super.begin();
        if (this.useFastMode)
        {
            this.movePIDHandler = new PIDHandler(
                TuningConstants.VISION_FAST_MOVING_PID_KP,
                TuningConstants.VISION_FAST_MOVING_PID_KI,
                TuningConstants.VISION_FAST_MOVING_PID_KD,
                TuningConstants.VISION_FAST_MOVING_PID_KF,
                TuningConstants.VISION_FAST_MOVING_PID_KS,
                TuningConstants.VISION_FAST_MOVING_PID_MIN,
                TuningConstants.VISION_FAST_MOVING_PID_MAX,
                this.getInjector().getInstance(ITimer.class));
        }
        else
        {
            this.movePIDHandler = new PIDHandler(
                TuningConstants.VISION_MOVING_PID_KP,
                TuningConstants.VISION_MOVING_PID_KI,
                TuningConstants.VISION_MOVING_PID_KD,
                TuningConstants.VISION_MOVING_PID_KF,
                TuningConstants.VISION_MOVING_PID_KS,
                TuningConstants.VISION_MOVING_PID_MIN,
                TuningConstants.VISION_MOVING_PID_MAX,
                this.getInjector().getInstance(ITimer.class));
        }
    }

    @Override
    public void update()
    {
        super.update();
        Double currentDistance = this.getDistance();
        if (currentDistance != null)
        {
            double desiredDistance = this.getDesiredDistance(currentDistance);
            double desiredSpeed = -this.movePIDHandler.calculatePosition(desiredDistance, currentDistance);
            switch (this.translateType)
            {
                case AprilTagStrafe:
                    this.setAnalogOperationState(AnalogOperation.DriveTrainMoveRight, desiredSpeed);
                    break;

                default:
                case Forward:
                    this.setAnalogOperationState(AnalogOperation.DriveTrainMoveForward, desiredSpeed);
                    break;
            }
        }
    }

    @Override
    public void end()
    {
        super.end();
        this.setAnalogOperationState(AnalogOperation.DriveTrainMoveForward, 0.0);
    }

    @Override
    public boolean hasCompleted()
    {
        Double currentDistance = this.getDistance();
        if (currentDistance == null)
        {
            return false;
        }

        double distanceOffset = Math.abs(this.getDesiredDistance(currentDistance) - currentDistance);
        if (distanceOffset > TuningConstants.MAX_VISION_ACCEPTABLE_FORWARD_DISTANCE)
        {
            return false;
        }

        return !this.verifyAngle || super.hasCompleted();
    }

    protected Double getDistance()
    {
        Double distance;
        if (this.translateType == MoveType.Forward)
        {
            if (this.isAprilTag())
            {
                distance = this.visionManager.getAprilTagXOffset();
            }
            else
            {
                distance = this.visionManager.getVisionTargetDistance();
            }
        }
        else
        {
            distance = this.visionManager.getAprilTagYOffset();
        }

        return distance;
    }

    @Override
    protected PIDHandler createTurnHandler()
    {
        return new PIDHandler(
            TuningConstants.VISION_MOVING_CENTERING_PID_KP,
            TuningConstants.VISION_MOVING_CENTERING_PID_KI,
            TuningConstants.VISION_MOVING_CENTERING_PID_KD,
            TuningConstants.VISION_MOVING_CENTERING_PID_KF,
            TuningConstants.VISION_MOVING_CENTERING_PID_KS,
            TuningConstants.VISION_MOVING_CENTERING_PID_MIN,
            TuningConstants.VISION_MOVING_CENTERING_PID_MAX,
            this.getInjector().getInstance(ITimer.class));
    }

    protected abstract double getDesiredDistance(double currentDistance);
}