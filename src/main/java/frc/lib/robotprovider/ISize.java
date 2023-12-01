package frc.lib.robotprovider;

public interface ISize
{
    double getWidth();
    double getHeight();
    void set(double[] vals);
    double area();
    ISize clone();
}