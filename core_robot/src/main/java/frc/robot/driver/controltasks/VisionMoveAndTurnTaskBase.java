package frc.robot.driver.controltasks;

import frc.robot.TuningConstants;
import frc.robot.common.PIDHandler;
import frc.robot.common.robotprovider.ITimer;
import frc.robot.driver.*;
import frc.robot.mechanisms.OffboardVisionManager;


    // after apriltag vision
    // see lower RRT
    // get horizontal angle and center with it
    // then score (arm task)

    
public abstract class VisionMoveAndTurnTaskBase extends VisionTurningTask
{
    public enum MoveType
    {
        Forward,
        AprilTagStrafe,
        RetroReflectiveStrafe,
    }

    private final MoveType translateType;
    private final boolean useFastMode;
    private final boolean useSlowMode;
    private final boolean verifyAngle;

    private double horizontalAngle;
    protected OffboardVisionManager visionManager;

    private PIDHandler movePIDHandler;

    /**
    * Initializes a new VisionAdvanceAndCenterTaskBase
    */
    protected VisionMoveAndTurnTaskBase(boolean useFastMode, TurnType rotateType, MoveType translateType, boolean bestEffort, boolean verifyAngle)
    {
        super(false, rotateType, bestEffort);

        this.translateType = translateType;

        this.useFastMode = useFastMode;
        this.useSlowMode = false;
        this.verifyAngle = verifyAngle;
        this.movePIDHandler = null;
    }

    public VisionMoveAndTurnTaskBase(boolean useSlowMode, MoveType translateType)
    {
        super(false, null, true);
        this.useSlowMode = useSlowMode;
        this.translateType = translateType;
        this.verifyAngle = true;
        this.movePIDHandler = null;
        this.useFastMode = false;
    }

    /**
     * Begin the current task
     */
    @Override
    public void begin()
    {
        super.begin();

        horizontalAngle = visionManager.getVisionTargetHorizontalAngle();
        
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

        else if(useSlowMode)
        {
            this.movePIDHandler = new PIDHandler(
                TuningConstants.VISION_MOVING_CENTERING_PID_KP,
                TuningConstants.VISION_MOVING_CENTERING_PID_KI,
                TuningConstants.VISION_MOVING_CENTERING_PID_KD,
                TuningConstants.VISION_MOVING_CENTERING_PID_KF,
                TuningConstants.VISION_MOVING_CENTERING_PID_KS,
                TuningConstants.VISION_MOVING_CENTERING_PID_MIN,
                TuningConstants.VISION_MOVING_CENTERING_PID_MAX,
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
        horizontalAngle = visionManager.getVisionTargetHorizontalAngle();
        super.update();
        Double currentDistance = this.getDistance();
        if (currentDistance != null)
        {
            double desiredDistance = this.getDesiredDistance(currentDistance);
            double desiredSpeed = -this.movePIDHandler.calculatePosition(desiredDistance, currentDistance);
            switch (this.translateType)
            {
                case RetroReflectiveStrafe:

                    // if to the right of the RRT
                    if (horizontalAngle > TuningConstants.ACCEPTABLE_ANGLE_RR_ERROR)
                    {
                        this.setAnalogOperationState(AnalogOperation.DriveTrainMoveRight, TuningConstants.RR_CENTERING_MOVEMENT_SPEED_TO_CENTER_WITH_REFLECTIVE_TAPE);
                    }

                    //If to the left of the RRT
                    else if(horizontalAngle < -TuningConstants.ACCEPTABLE_ANGLE_RR_ERROR)
                    {
                        this.setAnalogOperationState(AnalogOperation.DriveTrainMoveRight, -TuningConstants.RR_CENTERING_MOVEMENT_SPEED_TO_CENTER_WITH_REFLECTIVE_TAPE);
                    }

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