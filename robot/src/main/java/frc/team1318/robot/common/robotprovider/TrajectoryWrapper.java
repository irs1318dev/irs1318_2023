package frc.team1318.robot.common.robotprovider;

import jaci.pathfinder.Trajectory;

public class TrajectoryWrapper implements ITrajectory
{
    private final Trajectory wrappedObject;
    public TrajectoryWrapper(Trajectory wrappedObject)
    {
        this.wrappedObject = wrappedObject;
    }

    @Override
    public int length()
    {
        return this.wrappedObject.length();
    }

    @Override
    public ISegment get(int currentSegmentIndex)
    {
        return new SegmentWrapper(this.wrappedObject.get(currentSegmentIndex));
    }
}