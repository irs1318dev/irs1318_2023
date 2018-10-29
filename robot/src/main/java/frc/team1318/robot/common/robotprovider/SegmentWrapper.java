package frc.team1318.robot.common.robotprovider;

import jaci.pathfinder.Trajectory.Segment;

public class SegmentWrapper implements ISegment
{
    private final Segment wrappedObject;
    public SegmentWrapper(Segment wrappedObject)
    {
        this.wrappedObject = wrappedObject;
    }

    @Override
    public double getPosition()
    {
        return this.wrappedObject.position;
    }

    @Override
    public double getVelocity()
    {
        return this.wrappedObject.velocity;
    }

    @Override
    public double getAcceleration()
    {
        return this.wrappedObject.acceleration;
    }
}