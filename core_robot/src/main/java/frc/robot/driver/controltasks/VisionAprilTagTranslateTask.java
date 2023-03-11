package frc.robot.driver.controltasks;

import com.google.inject.Inject;
import com.google.inject.Singleton;

import frc.robot.TuningConstants;
import frc.robot.common.PIDHandler;
import frc.robot.common.robotprovider.ITimer;
import frc.robot.driver.AnalogOperation;
import frc.robot.driver.DigitalOperation;
import frc.robot.mechanisms.DriveTrainMechanism;
import frc.robot.mechanisms.OffboardVisionManager;

@Singleton
public class VisionAprilTagTranslateTask extends ControlTaskBase
{
    public enum GridScoringPosition
    {
        LeftCone,
        MiddleCube,
        RightCone,
    }

    private enum TranslateState
    {
        FindAprilTags,
        Translate,
        Completed,
        Stop
    };

    private TranslateState currentState;

    private double[] xAprilTagDistanceSamples;
    private double[] yAprilTagDistanceSamples;

    private int tagsFound;
    private int tagsMissed;

    private double startingDriveTrainX;
    private double startingDriveTrainY;

    private double desiredXPosition;
    private double desiredYPosition;

    private double visionOffset;

    private OffboardVisionManager vision;
    private DriveTrainMechanism driveTrain;
    private PIDHandler xHandler;
    private PIDHandler yHandler;

    @Inject
    public VisionAprilTagTranslateTask(GridScoringPosition position)
    {
        switch (position)
        {
            case LeftCone:
                this.visionOffset = TuningConstants.APRILTAG_TO_CONE_NODE_HORIZONTAL_DISTANCE;
                break;

            case RightCone:
                this.visionOffset = -TuningConstants.APRILTAG_TO_CONE_NODE_HORIZONTAL_DISTANCE;
                break;

            default:
            case MiddleCube:
                this.visionOffset = 0.0;
                break;
        }
    }

    @Override
    public void begin()
    {
        ITimer timer = this.getInjector().getInstance(ITimer.class);
        this.xHandler = new PIDHandler(
            TuningConstants.VISION_FAST_MOVING_PID_KP,
            TuningConstants.VISION_FAST_MOVING_PID_KI,
            TuningConstants.VISION_FAST_MOVING_PID_KD,
            TuningConstants.VISION_FAST_MOVING_PID_KF,
            TuningConstants.VISION_FAST_MOVING_PID_KS,
            TuningConstants.VISION_FAST_MOVING_PID_MIN,
            TuningConstants.VISION_FAST_MOVING_PID_MAX,
            timer);

        this.yHandler = new PIDHandler(
            TuningConstants.VISION_FAST_MOVING_PID_KP,
            TuningConstants.VISION_FAST_MOVING_PID_KI,
            TuningConstants.VISION_FAST_MOVING_PID_KD,
            TuningConstants.VISION_FAST_MOVING_PID_KF,
            TuningConstants.VISION_FAST_MOVING_PID_KS,
            TuningConstants.VISION_FAST_MOVING_PID_MIN,
            TuningConstants.VISION_FAST_MOVING_PID_MAX,
            timer);

        this.currentState = TranslateState.FindAprilTags;
        this.tagsFound = 0;
        this.tagsMissed = 0;
        this.startingDriveTrainX = this.driveTrain.getPositionX();
        this.startingDriveTrainY = this.driveTrain.getPositionY();
        this.xAprilTagDistanceSamples = new double[TuningConstants.TAGS_FOUND_THRESHOLD];
        this.yAprilTagDistanceSamples = new double[TuningConstants.TAGS_FOUND_THRESHOLD];

        this.setDigitalOperationState(DigitalOperation.VisionEnableAprilTagProcessing, true);
    }

    @Override
    public void update()
    {
        double currXPosition = this.driveTrain.getPositionX();
        double currYPosition = this.driveTrain.getPositionY();
        if (this.currentState == TranslateState.FindAprilTags)
        {
            Double xOffset = this.vision.getAprilTagXOffset();
            Double yOffset = this.vision.getAprilTagYOffset();
            if (xOffset != null && yOffset != null)
            {
                this.xAprilTagDistanceSamples[this.tagsFound] = xOffset;
                this.yAprilTagDistanceSamples[this.tagsFound] = yOffset;
                this.tagsFound++;
            }
            else
            {
                this.tagsMissed++;
            }

            if (this.tagsFound >= TuningConstants.TAGS_FOUND_THRESHOLD)
            {
                double xAprilTagSumDistance = 0;
                double yAprilTagSumDistance = 0;
                for (int i = 0; i < TuningConstants.TAGS_FOUND_THRESHOLD; i++)
                {
                    xAprilTagSumDistance += this.xAprilTagDistanceSamples[i];
                    yAprilTagSumDistance += this.yAprilTagDistanceSamples[i];
                }

                double xAprilTagDistanceAverage = xAprilTagSumDistance / TuningConstants.TAGS_FOUND_THRESHOLD;
                double yAprilTagDistanceAverage = yAprilTagSumDistance / TuningConstants.TAGS_FOUND_THRESHOLD;
                this.desiredXPosition = this.startingDriveTrainX + xAprilTagDistanceAverage - TuningConstants.APRILTAG_TO_DESIRED_SCORING_X_POSITION_DISTANCE;
                this.desiredYPosition = this.startingDriveTrainY + yAprilTagDistanceAverage + this.visionOffset;
                this.currentState = TranslateState.Translate;
            }
            else if (this.tagsMissed >= TuningConstants.TAGS_MISSED_THRESHOLD)
            {
                this.currentState = TranslateState.Stop;
            }
        }
        else if (this.currentState == TranslateState.Translate)
        {
            if (Math.abs(currXPosition - this.desiredXPosition) < TuningConstants.ACCEPTABLE_RANGE_IN_X_AND_Y_FOR_ALIGNMENT_TRANSLATE && 
                Math.abs(currYPosition - this.desiredYPosition) < TuningConstants.ACCEPTABLE_RANGE_IN_X_AND_Y_FOR_ALIGNMENT_TRANSLATE)
            {
                this.currentState = TranslateState.Completed;
            }
        }

        switch (this.currentState)
        {
            case FindAprilTags:
                this.setAnalogOperationState(AnalogOperation.DriveTrainMoveForward, 0.0);
                this.setAnalogOperationState(AnalogOperation.DriveTrainMoveRight, 0.0);
                this.setDigitalOperationState(DigitalOperation.VisionEnableAprilTagProcessing, true);
                break;

            case Translate:
                double xDesiredVelocity = this.xHandler.calculatePosition(desiredXPosition, currXPosition);
                double yDesiredVelocity = this.yHandler.calculatePosition(desiredYPosition, currYPosition);
                this.setAnalogOperationState(AnalogOperation.DriveTrainMoveForward, xDesiredVelocity);
                this.setAnalogOperationState(AnalogOperation.DriveTrainMoveRight, yDesiredVelocity);
                this.setDigitalOperationState(DigitalOperation.VisionEnableAprilTagProcessing, false);
                break;

            default:
            case Completed:
            case Stop:
                this.setAnalogOperationState(AnalogOperation.DriveTrainMoveForward, 0.0);
                this.setAnalogOperationState(AnalogOperation.DriveTrainMoveRight, 0.0);
                this.setDigitalOperationState(DigitalOperation.VisionEnableAprilTagProcessing, false);
                break;
        }
    }

    @Override
    public void end()
    {
        this.setAnalogOperationState(AnalogOperation.DriveTrainMoveForward, 0.0);
        this.setAnalogOperationState(AnalogOperation.DriveTrainMoveRight, 0.0);
        this.setDigitalOperationState(DigitalOperation.VisionEnableAprilTagProcessing, false);
    }

    @Override
    public boolean shouldCancel()
    {
        return this.currentState == TranslateState.Stop;
    }

    @Override
    public boolean hasCompleted()
    {
        return this.currentState == TranslateState.Completed;
    }
}
