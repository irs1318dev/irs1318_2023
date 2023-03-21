package frc.lib.robotprovider;

import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class NetworkTableProvider implements INetworkTableProvider
{
    private static NetworkTableInstance instance;
    private static NetworkTable smartDashboard;

    private static NetworkTableInstance getInstance()
    {
        if (NetworkTableProvider.instance == null)
        {
            NetworkTableInstance inst = NetworkTableInstance.getDefault();
            NetworkTableProvider.instance = inst;
        }

        return NetworkTableProvider.instance;
    }

    private static NetworkTable getSmartDashboard()
    {
        if (NetworkTableProvider.smartDashboard == null)
        {
            NetworkTableInstance inst = NetworkTableProvider.getInstance();
            NetworkTableProvider.smartDashboard = inst.getTable("SmartDashboard");
        }

        return NetworkTableProvider.smartDashboard;
    }

    @Override
    public void startShuffleboardRecording()
    {
        Shuffleboard.startRecording();
    }

    @Override
    public void stopShuffleboardRecording()
    {
        Shuffleboard.stopRecording();
    }

    @Override
    public IDoubleSubscriber getNumberSlider(String title, double initialValue)
    {
        DoubleTopic number = NetworkTableProvider.getSmartDashboard().getDoubleTopic(title);
        number.publish().set(initialValue);
        return new DoubleSubscriberWrapper(number.subscribe(initialValue));
    }

    @Override
    public <V> ISendableChooser<V> getSendableChooser()
    {
        return new SendableChooserWrapper<V>();
    }

    @Override
    public <V> void addChooser(String name, ISendableChooser<V> chooser)
    {
        SendableChooserWrapper<V> wrappedChooser = (SendableChooserWrapper<V>)chooser;
        SmartDashboard.putData(name, wrappedChooser.wrappedObject);
    }

    @Override
    public IDoubleSubscriber getDoubleSubscriber(String key)
    {
        return this.getDoubleSubscriber(key, 0.0);
    }

    @Override
    public IDoubleSubscriber getDoubleSubscriber(String key, double defaultValue)
    {
        return new DoubleSubscriberWrapper(NetworkTableProvider.getSmartDashboard().getDoubleTopic(key).subscribe(defaultValue));
    }

    @Override
    public IBooleanSubscriber getBooleanSubscriber(String key)
    {
        return this.getBooleanSubscriber(key, false);
    }

    @Override
    public IBooleanSubscriber getBooleanSubscriber(String key, boolean defaultValue)
    {
        return new BooleanSubscriberWrapper(NetworkTableProvider.getSmartDashboard().getBooleanTopic(key).subscribe(defaultValue));
    }

    @Override
    public IIntegerSubscriber getIntegerSubscriber(String key)
    {
        return this.getIntegerSubscriber(key, 0);
    }

    @Override
    public IIntegerSubscriber getIntegerSubscriber(String key, int defaultValue)
    {
        return new IntegerSubscriberWrapper(NetworkTableProvider.getSmartDashboard().getIntegerTopic(key).subscribe(defaultValue));
    }

    @Override
    public IStringSubscriber getStringSubscriber(String key)
    {
        return this.getStringSubscriber(key, null);
    }

    @Override
    public IStringSubscriber getStringSubscriber(String key, String defaultValue)
    {
        return new StringSubscriberWrapper(NetworkTableProvider.getSmartDashboard().getStringTopic(key).subscribe(defaultValue));
    }
}