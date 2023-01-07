package frc.robot.common.robotprovider;

public interface INetworkTableProvider
{
    IDoubleSubscriber getNumberSlider(String title, double initialValue);
    <V> ISendableChooser<V> getSendableChooser();
    <V> void addChooser(String name, ISendableChooser<V> chooser);
    IDoubleSubscriber getDoubleSubscriber(String key);
    IBooleanSubscriber getBooleanSubscriber(String key);
    IIntegerSubscriber getIntegerSubscriber(String key);
    IStringSubscriber getStringSubscriber(String key);
}
