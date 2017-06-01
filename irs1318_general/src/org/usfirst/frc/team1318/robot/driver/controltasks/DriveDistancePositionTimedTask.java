package org.usfirst.frc.team1318.robot.driver.controltasks;

import org.usfirst.frc.team1318.robot.driver.Operation;
import org.usfirst.frc.team1318.robot.drivetrain.DriveTrainComponent;

public class DriveDistancePositionTimedTask extends TimedTask
{
    private final double distance;
    private final double velocity;

    private DriveTrainComponent driveTrain;

    private double startLeftDistance;
    private double startRightDistance;
    private double endLeftDistance;
    private double endRightDistance;

    public DriveDistancePositionTimedTask(double speed, double distance, double duration)
    {
        super(duration);

        this.distance = distance;
        this.velocity = speed;
    }

    @Override
    public void begin()
    {
        super.begin();

        this.driveTrain = this.getInjector().getInstance(DriveTrainComponent.class);

        this.startLeftDistance = this.driveTrain.getLeftEncoderDistance();
        this.startRightDistance = this.driveTrain.getRightEncoderDistance();

        this.endLeftDistance = this.startLeftDistance + this.distance;
        this.endRightDistance = this.startRightDistance + this.distance;

        this.setDigitalOperationState(Operation.DriveTrainUsePositionalMode, false);
        this.setAnalogOperationState(Operation.DriveTrainTurn, 0.0);
        this.setAnalogOperationState(Operation.DriveTrainMoveForward, this.velocity);
    }

    @Override
    public void update()
    {
        this.setDigitalOperationState(Operation.DriveTrainUsePositionalMode, false);
        this.setAnalogOperationState(Operation.DriveTrainTurn, 0.0);
        this.setAnalogOperationState(Operation.DriveTrainMoveForward, this.velocity);
    }

    @Override
    public void stop()
    {
        super.stop();

        this.setDigitalOperationState(Operation.DriveTrainUsePositionalMode, false);
        this.setAnalogOperationState(Operation.DriveTrainTurn, 0.0);
        this.setAnalogOperationState(Operation.DriveTrainMoveForward, 0.0);
    }

    @Override
    public void end()
    {
        super.end();

        this.setDigitalOperationState(Operation.DriveTrainUsePositionalMode, false);
        this.setAnalogOperationState(Operation.DriveTrainTurn, 0.0);
        this.setAnalogOperationState(Operation.DriveTrainMoveForward, 0.0);
    }

    @Override
    public boolean hasCompleted()
    {
        if (super.hasCompleted())
        {
            return true;
        }

        double leftEncoderDistance = this.driveTrain.getLeftEncoderDistance();
        double rightEncoderDistance = this.driveTrain.getRightEncoderDistance();

        if (this.distance >= 0.0)
        {
            return leftEncoderDistance >= this.endLeftDistance ||
                rightEncoderDistance >= this.endRightDistance;
        }
        else
        {
            return leftEncoderDistance <= this.endLeftDistance ||
                rightEncoderDistance <= this.endRightDistance;
        }
    }

    @Override
    public boolean shouldCancel()
    {
        return super.shouldCancel();
    }
}
