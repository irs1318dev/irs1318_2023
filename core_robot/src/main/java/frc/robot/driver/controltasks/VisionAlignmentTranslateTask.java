package frc.robot.driver.controltasks;

import com.google.inject.Inject;
import com.google.inject.Singleton;

import frc.robot.TuningConstants;
import frc.robot.common.PIDHandler;
import frc.robot.common.robotprovider.ITimer;
import frc.robot.driver.AnalogOperation;
import frc.robot.mechanisms.DriveTrainMechanism;
import frc.robot.mechanisms.OffboardVisionManager;

@Singleton
public class VisionAlignmentTranslateTask extends ControlTaskBase
{
    private enum TranslateState
    {
        FindAprilTags,
        Translate,
        Completed,
        Stop
    };
    private TranslateState currentState;

    // public enum GridScoringPosition
    // {
    //     LeftCone,
    //     MiddleCube,
    //     RightCone,
    // }
    // private GridScoringPosition currentScorePositionState;

    private double[] xAprilTagDistance;
    private double[] yAprilTagDistance;
    private double xAprilTagDistanceAverage;
    private double yAprilTagDistanceAverage;
    private int tagsFound;
    private int tagsMissed;
    private double startingDriveTrainX;
    private double startingDriveTrainY;

    private double desiredDriveTrainX;
    private double desiredDriveTrainY;

    private double xDesiredVelocity;
    private double yDesiredVelocity;
    private double visionOffset;

    private OffboardVisionManager vision;
    private DriveTrainMechanism driveTrain;
    private PIDHandler XPidHandler;
    private PIDHandler yPIDHandler;

    @Inject
    public VisionAlignmentTranslateTask(double visionOffset)
    {
        this.visionOffset = visionOffset;
    }
    @Override
    public void begin()
    {
        this.XPidHandler = new PIDHandler(
            TuningConstants.VISION_FAST_MOVING_PID_KP,
            TuningConstants.VISION_FAST_MOVING_PID_KI,
            TuningConstants.VISION_FAST_MOVING_PID_KD,
            TuningConstants.VISION_FAST_MOVING_PID_KF,
            TuningConstants.VISION_FAST_MOVING_PID_KS,
            TuningConstants.VISION_FAST_MOVING_PID_MIN,
            TuningConstants.VISION_FAST_MOVING_PID_MAX,
            this.getInjector().getInstance(ITimer.class));
        
        this.yPIDHandler = new PIDHandler(
            TuningConstants.VISION_FAST_MOVING_PID_KP,
            TuningConstants.VISION_FAST_MOVING_PID_KI,
            TuningConstants.VISION_FAST_MOVING_PID_KD,
            TuningConstants.VISION_FAST_MOVING_PID_KF,
            TuningConstants.VISION_FAST_MOVING_PID_KS,
            TuningConstants.VISION_FAST_MOVING_PID_MIN,
            TuningConstants.VISION_FAST_MOVING_PID_MAX,
            this.getInjector().getInstance(ITimer.class));

        this.currentState = TranslateState.FindAprilTags;
        tagsFound = 0;
        tagsMissed = 0;
        startingDriveTrainX = this.driveTrain.getPositionX();
        startingDriveTrainY = this.driveTrain.getPositionY();
        xAprilTagDistance = new double[TuningConstants.TAGS_FOUND_THRESHOLD];
        yAprilTagDistance = new double[TuningConstants.TAGS_FOUND_THRESHOLD];
    }

    @Override
    public void update() 
    {
        switch(this.currentState)
        {
            case FindAprilTags:
            if (tagsFound >= TuningConstants.TAGS_FOUND_THRESHOLD)
            {
                this.currentState = TranslateState.Translate;
                double xAprilTagSumDistance = 0;
                double yAprilTagSumDistance = 0;
                for (int i = 0; i < TuningConstants.APRIL_TAG_SAMPLES_DESIRED; i++)
                {
                    xAprilTagSumDistance += xAprilTagDistance[i];
                    yAprilTagSumDistance += yAprilTagDistance[i];
                }
                this.xAprilTagDistanceAverage = xAprilTagSumDistance/TuningConstants.APRIL_TAG_SAMPLES_DESIRED;
                this.yAprilTagDistanceAverage = yAprilTagSumDistance/TuningConstants.APRIL_TAG_SAMPLES_DESIRED;
                this.desiredDriveTrainX = this.startingDriveTrainX + this.xAprilTagDistanceAverage - TuningConstants.APRILTAG_TO_DESIRED_SCORING_X_POSITION_DISTANCE;
                this.desiredDriveTrainY = this.startingDriveTrainY + this.yAprilTagDistanceAverage + this.visionOffset;
                this.currentState = TranslateState.Translate;
                break;
            }

            else if (tagsMissed >= TuningConstants.TAGS_MISSED_THRESHOLD)
            {
                this.currentState = TranslateState.Stop;
                break;
            }

            if (this.vision.getAprilTagXOffset() != null && this.vision.getAprilTagYOffset() != null) //Not sure if should be && or || conditional
            {
                this.xAprilTagDistance[tagsFound] = this.vision.getAprilTagXOffset();
                this.yAprilTagDistance[tagsFound] = this.vision.getAprilTagYOffset(); 
                this.tagsFound++;
            }
            else {
                this.tagsMissed++;
            }
 
            case Translate:
                if (driveTrain.getPositionX() < TuningConstants.ACCEPTABLE_RANGE_IN_X_AND_Y_FOR_ALIGNMENT_TRANSLATE && 
                driveTrain.getPositionY() < TuningConstants.ACCEPTABLE_RANGE_IN_X_AND_Y_FOR_ALIGNMENT_TRANSLATE)
                {
                this.currentState = TranslateState.Completed;
                }
                xDesiredVelocity = this.yPIDHandler.calculatePosition(desiredDriveTrainY, startingDriveTrainY);
                yDesiredVelocity = this.XPidHandler.calculatePosition(desiredDriveTrainX, startingDriveTrainX);
                this.setAnalogOperationState(AnalogOperation.DriveTrainMoveForward, xDesiredVelocity);
                this.setAnalogOperationState(AnalogOperation.DriveTrainMoveRight, yDesiredVelocity);
                this.currentState = TranslateState.Completed;
                break;
            
            case Completed:
                break;
            
            case Stop:
                break;
        }

    }

    @Override
    public void end() {
        
    }

    @Override
    public boolean hasCompleted() {
        return this.currentState == TranslateState.Completed;
    }
}
