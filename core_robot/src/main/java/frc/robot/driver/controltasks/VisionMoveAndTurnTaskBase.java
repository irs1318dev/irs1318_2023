package frc.robot.driver.controltasks;

import frc.robot.TuningConstants;
import frc.robot.common.PIDHandler;
import frc.robot.common.robotprovider.ITimer;
import frc.robot.driver.*;


    // after apriltag vision
    // see lower RRT
    // get horizontal angle and center with it
    // then score (arm task)
public abstract class VisionMoveAndTurnTaskBase extends VisionTurningTask
{
    public enum MoveSpeed
    {
        Normal,
        Fast,
        Slow,
    }

    public enum MoveType
    {
        Forward,
        AprilTagStrafe,
        RetroReflectiveStrafe,
    }

    private final MoveType translateType;
    private final MoveSpeed moveSpeed;
    private final boolean verifyAngle;

    private PIDHandler movePIDHandler;

    /**
    * Initializes a new VisionAdvanceAndCenterTaskBase
    */
    protected VisionMoveAndTurnTaskBase(TurnType rotateType, MoveType translateType, MoveSpeed moveSpeed, boolean bestEffort, boolean verifyAngle)
    {
        super(false, rotateType, bestEffort);

        this.translateType = translateType;

        this.moveSpeed = moveSpeed;
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

        if (this.translateType == MoveType.RetroReflectiveStrafe)
        {
            this.setDigitalOperationState(DigitalOperation.VisionEnableRetroreflectiveProcessing, true);
        }

        this.setDigitalOperationState(DigitalOperation.DriveTrainDisableFieldOrientation, true);
        this.setDigitalOperationState(DigitalOperation.DriveTrainEnableFieldOrientation, false);
        switch (this.moveSpeed)
        {
            case Fast:
                this.movePIDHandler = new PIDHandler(
                    TuningConstants.VISION_FAST_MOVING_PID_KP,
                    TuningConstants.VISION_FAST_MOVING_PID_KI,
                    TuningConstants.VISION_FAST_MOVING_PID_KD,
                    TuningConstants.VISION_FAST_MOVING_PID_KF,
                    TuningConstants.VISION_FAST_MOVING_PID_KS,
                    TuningConstants.VISION_FAST_MOVING_PID_MIN,
                    TuningConstants.VISION_FAST_MOVING_PID_MAX,
                    this.getInjector().getInstance(ITimer.class));
                break;

            case Slow:
                this.movePIDHandler = new PIDHandler(
                    TuningConstants.VISION_SLOW_MOVING_PID_KP,
                    TuningConstants.VISION_SLOW_MOVING_PID_KI,
                    TuningConstants.VISION_SLOW_MOVING_PID_KD,
                    TuningConstants.VISION_SLOW_MOVING_PID_KF,
                    TuningConstants.VISION_SLOW_MOVING_PID_KS,
                    TuningConstants.VISION_SLOW_MOVING_PID_MIN,
                    TuningConstants.VISION_SLOW_MOVING_PID_MAX,
                    this.getInjector().getInstance(ITimer.class));
                break;

            default:
            case Normal:
                this.movePIDHandler = new PIDHandler(
                    TuningConstants.VISION_MOVING_PID_KP,
                    TuningConstants.VISION_MOVING_PID_KI,
                    TuningConstants.VISION_MOVING_PID_KD,
                    TuningConstants.VISION_MOVING_PID_KF,
                    TuningConstants.VISION_MOVING_PID_KS,
                    TuningConstants.VISION_MOVING_PID_MIN,
                    TuningConstants.VISION_MOVING_PID_MAX,
                    this.getInjector().getInstance(ITimer.class));
                break;
        }
    }

    @Override
    public void update()
    {
        super.update();

        Double currentDistance = this.getMoveMeasuredValue();
        if (currentDistance != null)
        {
            double desiredValue = this.getMoveDesiredValue(currentDistance);
            double desiredVelocity = -this.movePIDHandler.calculatePosition(desiredValue, currentDistance);
            switch (this.translateType)
            {
                case RetroReflectiveStrafe:
                    this.setAnalogOperationState(AnalogOperation.DriveTrainMoveRight, -desiredVelocity);
                    break;

                case AprilTagStrafe:
                    this.setAnalogOperationState(AnalogOperation.DriveTrainMoveRight, -desiredVelocity);
                    break;

                default:
                case Forward:
                    this.setAnalogOperationState(AnalogOperation.DriveTrainMoveForward, desiredVelocity);
                    break;
            }
        }
    }

    @Override
    public void end()
    {
        super.end();

        this.setDigitalOperationState(DigitalOperation.DriveTrainDisableFieldOrientation, false);
        this.setDigitalOperationState(DigitalOperation.DriveTrainEnableFieldOrientation, true);
        this.setAnalogOperationState(AnalogOperation.DriveTrainMoveForward, 0.0);
        this.setDigitalOperationState(DigitalOperation.VisionEnableRetroreflectiveProcessing, false);
    }

    @Override
    public boolean hasCompleted()
    {
        Double measuredValue = this.getMoveMeasuredValue();
        if (measuredValue == null)
        {
            return false;
        }

        double offset = Math.abs(this.getMoveDesiredValue(measuredValue) - measuredValue);

        // return false if we have not yet reached an acceptable offset from our goal position
        switch (this.translateType)
        {
            case RetroReflectiveStrafe:
                if (offset > TuningConstants.MAX_VISION_ACCEPTABLE_MOVING_RR_ANGLE_ERROR)
                {
                    return false;
                }

                break;

            case Forward:
                if (offset > TuningConstants.MAX_VISION_ACCEPTABLE_FORWARD_DISTANCE)
                {
                    return false;
                }

                break;
            
            case AprilTagStrafe:
            default:
                if (offset > TuningConstants.MAX_VISION_ACCEPTABLE_STRAFE_DISTANCE)
                {
                    return false;
                }

                break;
        }

        return !this.verifyAngle || super.hasCompleted();
    }

    protected Double getMoveMeasuredValue()
    {
        if (this.translateType == MoveType.Forward)
        {
            if (this.isAprilTag())
            {
                return this.visionManager.getAprilTagXOffset();
            }
            else
            {
                return this.visionManager.getVisionTargetDistance();
            }
        }
        else if (this.translateType == MoveType.RetroReflectiveStrafe)
        {
            return this.visionManager.getVisionTargetHorizontalAngle();
        }
        else // if (this.translateType == MoveType.AprilTagStrafe)
        {
            return this.visionManager.getAprilTagYOffset();
        }
    }

    @Override
    protected PIDHandler createTurnHandler()
    {
        return new PIDHandler(
            TuningConstants.VISION_MOVING_TURNING_PID_KP,
            TuningConstants.VISION_MOVING_TURNING_PID_KI,
            TuningConstants.VISION_MOVING_TURNING_PID_KD,
            TuningConstants.VISION_MOVING_TURNING_PID_KF,
            TuningConstants.VISION_MOVING_TURNING_PID_KS,
            TuningConstants.VISION_MOVING_TURNING_PID_MIN,
            TuningConstants.VISION_MOVING_TURNING_PID_MAX,
            this.getInjector().getInstance(ITimer.class));
    }

    protected abstract double getMoveDesiredValue(double currentDistance);
}