package frc.robot.driver.controltasks;

import javax.lang.model.util.ElementScanner6;

import frc.robot.TuningConstants;
import frc.robot.common.PIDHandler;
import frc.robot.common.robotprovider.ITimer;
import frc.robot.mechanisms.DriveTrainMechanism;
import frc.robot.mechanisms.OffboardVisionManager;

public class VisionAlignmentTranslateTask extends ControlTaskBase
{
    private enum TranslateState
    {
        FindAprilTags,
        Translate,
        Completed,
        Stop
    };
    private TranslateState currentTranslateState;

    public enum GridScoringPosition
    {
        LeftCone,
        MiddleCube,
        RightCone,
    }
    private GridScoringPosition currentScorePositionState;

    private double[] xAprilTagDistance;
    private double[] yAprilTagDistance;
    private double xAprilTagSumDistance;
    private double yAprilTagSumDistance;
    private double xAprilTagDistanceAverage;
    private double yAprilTagDistanceAverage;
    private int tagsFound;
    //private int tagsMissed;
    private double driveTrainX;
    private double driveTrainY;

    private OffboardVisionManager vision;
    private DriveTrainMechanism driveTrain;
    private PIDHandler pid;
    private ITimer timer;

    public VisionAlignmentTranslateTask(GridScoringPosition currentScorePositionState)
    {
        
    }

    @Override
    public void begin() {
        this.currentTranslateState = TranslateState.FindAprilTags;
        tagsFound = 0;
        //tagsMissed = 0;
        driveTrainX = this.driveTrain.getPositionX();
        driveTrainY = this.driveTrain.getPositionY();
    }

    @Override
    public void update() 
    {
        //TranslateState t = TranslateState.FindAprilTags;
        switch(this.currentTranslateState)
        {
            case FindAprilTags:
            if (this.vision.getAprilTagXOffset() != null && this.vision.getAprilTagYOffset() != null) //Not sure if should be && or || conditional
            {
                tagsFound++;
                // TO DO: put the apriltags Founds X & Y to xDist and yDist array
                // TO DO: then take average x and average y of those two arrays
            }

            if (tagsFound > TuningConstants.TAGS_FOUND_THRESHOLD)
            {
                this.currentTranslateState = TranslateState.Translate;
                for (int i = 0; i < TuningConstants.APRIL_TAG_SAMPLES_DESIRED; i++)
                {
                    xAprilTagDistance[i] = this.vision.getAprilTagXOffset();
                    yAprilTagDistance[i] = this.vision.getAprilTagYOffset();    
                    xAprilTagSumDistance = xAprilTagSumDistance + xAprilTagDistance[i]; 
                    yAprilTagSumDistance = yAprilTagSumDistance + yAprilTagDistance[i];            
                }
                xAprilTagDistanceAverage = xAprilTagSumDistance/TuningConstants.APRIL_TAG_SAMPLES_DESIRED;
                yAprilTagDistanceAverage = yAprilTagSumDistance/TuningConstants.APRIL_TAG_SAMPLES_DESIRED;

                this.currentTranslateState = TranslateState.Translate;
            }

            else if(tagsFound < TuningConstants.TAGS_FOUND_THRESHOLD)
            {
                this.currentTranslateState = TranslateState.Stop;
            }
 
            case Translate:
                // if (driveTrain.getPositionX() < TuningConstants.ACCEPTABLE_RANGE_IN_X_AND_Y_FOR_ALIGNMENT_TRANSLATE && 
                //  driveTrain.getPositionY() < TuningConstants.ACCEPTABLE_RANGE_IN_X_AND_Y_FOR_ALIGNMENT_TRANSLATE)
                //  {
                //     this.currentState = TranslateState.Completed;
                //  }
                switch(this.currentScorePositionState)
                {
                    case LeftCone:
                       
                        if (yAprilTagDistanceAverage - driveTrainY - TuningConstants.APRILTAG_TO_NODE_HORIZONTAL_DISTANCE > TuningConstants.ACCEPTABLE_RANGE_IN_X_AND_Y_FOR_ALIGNMENT_TRANSLATE) // Check if Left of
                        {
                            this.pid.calculateVelocity(TuningConstants.VELOCITY_TO_SETPOINT, yAprilTagDistanceAverage - driveTrainY - TuningConstants.APRILTAG_TO_NODE_HORIZONTAL_DISTANCE); //Y Movement (Right)
                            this.pid.calculateVelocity(TuningConstants.VELOCITY_TO_SETPOINT, xAprilTagDistanceAverage - driveTrainX - TuningConstants.APRILTAG_TO_DESIRED_SCORING_X_POSITION_DISTANCE); //X Movement, X of aprilTag and X of left and right are assumed the same, so same X Code for all three

                        }
                        else if (yAprilTagDistanceAverage - driveTrainY - TuningConstants.APRILTAG_TO_NODE_HORIZONTAL_DISTANCE < -TuningConstants.ACCEPTABLE_RANGE_IN_X_AND_Y_FOR_ALIGNMENT_TRANSLATE) //Check if Right of
                        {
                            this.pid.calculateVelocity(TuningConstants.VELOCITY_TO_SETPOINT, yAprilTagDistanceAverage - driveTrainY - TuningConstants.APRILTAG_TO_NODE_HORIZONTAL_DISTANCE); //Y Movement (Left)
                            this.pid.calculateVelocity(TuningConstants.VELOCITY_TO_SETPOINT, xAprilTagDistanceAverage - driveTrainX - TuningConstants.APRILTAG_TO_DESIRED_SCORING_X_POSITION_DISTANCE); //X Movement (Forwards)

                        }   
                        else 
                        {
                            this.currentTranslateState = TranslateState.Completed;
                            break;
                        }
                    
                    case MiddleCube:
                        if (yAprilTagDistanceAverage - driveTrainY > TuningConstants.ACCEPTABLE_RANGE_IN_X_AND_Y_FOR_ALIGNMENT_TRANSLATE) //Check if Left or Right Of
                        {
                            this.pid.calculateVelocity(TuningConstants.VELOCITY_TO_SETPOINT, yAprilTagDistanceAverage - driveTrainY ); //Y Movement Left
                            this.pid.calculateVelocity(TuningConstants.VELOCITY_TO_SETPOINT, xAprilTagDistanceAverage - driveTrainX - TuningConstants.APRILTAG_TO_DESIRED_SCORING_X_POSITION_DISTANCE); //X Movement (Forwards)
                        }
                        else if(yAprilTagDistanceAverage - driveTrainY < -TuningConstants.ACCEPTABLE_RANGE_IN_X_AND_Y_FOR_ALIGNMENT_TRANSLATE)
                        {
                            this.pid.calculateVelocity(TuningConstants.VELOCITY_TO_SETPOINT, yAprilTagDistanceAverage - driveTrainY ); //Y Movement Right
                            this.pid.calculateVelocity(TuningConstants.VELOCITY_TO_SETPOINT, xAprilTagDistanceAverage - driveTrainX - TuningConstants.APRILTAG_TO_DESIRED_SCORING_X_POSITION_DISTANCE); //X Movement (Forwards)
                        }
                        else
                        {
                            this.currentTranslateState = TranslateState.Completed;
                            break;
                        }

                    case RightCone:
                        if (TuningConstants.APRILTAG_TO_DESIRED_SCORING_X_POSITION_DISTANCE - driveTrainY - yAprilTagDistanceAverage > TuningConstants.ACCEPTABLE_RANGE_IN_X_AND_Y_FOR_ALIGNMENT_TRANSLATE) // Check if Left of
                            {
                                this.pid.calculateVelocity(TuningConstants.VELOCITY_TO_SETPOINT, TuningConstants.APRILTAG_TO_DESIRED_SCORING_X_POSITION_DISTANCE - driveTrainY - yAprilTagDistanceAverage); //Y Movement (Right)
                                this.pid.calculateVelocity(TuningConstants.VELOCITY_TO_SETPOINT, xAprilTagDistanceAverage - driveTrainX - TuningConstants.APRILTAG_TO_DESIRED_SCORING_X_POSITION_DISTANCE); //X Movement

                            }
                            else if (TuningConstants.APRILTAG_TO_DESIRED_SCORING_X_POSITION_DISTANCE - driveTrainY - yAprilTagDistanceAverage < -TuningConstants.ACCEPTABLE_RANGE_IN_X_AND_Y_FOR_ALIGNMENT_TRANSLATE) // Check if Right of
                            {
                                this.pid.calculateVelocity(TuningConstants.VELOCITY_TO_SETPOINT, TuningConstants.APRILTAG_TO_DESIRED_SCORING_X_POSITION_DISTANCE - driveTrainY - yAprilTagDistanceAverage); //Y Movement (Left) *Can make one long conditional and combine as negative distance would set negative velocity and go left
                                this.pid.calculateVelocity(TuningConstants.VELOCITY_TO_SETPOINT, xAprilTagDistanceAverage - driveTrainX -TuningConstants.APRILTAG_TO_DESIRED_SCORING_X_POSITION_DISTANCE); //X Movement (Forwards)
                            }   
                            else 
                            {
                                this.currentTranslateState = TranslateState.Completed;
                                break;
                            }
                }
            
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
        return this.currentTranslateState == TranslateState.Completed;
    }
}
