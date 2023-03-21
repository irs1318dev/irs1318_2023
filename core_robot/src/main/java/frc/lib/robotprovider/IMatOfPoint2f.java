package frc.lib.robotprovider;

import java.util.List;

public interface IMatOfPoint2f
{
    void alloc(int elemNumber);
    void fromArray(IPoint...a);
    IPoint[] toArray();
    void fromList(List<IPoint> lp);
    List<IPoint> toList();
    void release();
}