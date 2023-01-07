package frc.robot.common.robotprovider;

public class FauxbotNetworkTableProvider implements INetworkTableProvider
{
    @Override
    public IDoubleSubscriber getNumberSlider(String title, double initialValue)
    {
        return null;
    }

    /**
     * Add a sendable chooser to the smart dashboard
     */
    @Override
    public <V> void addChooser(String name, ISendableChooser<V> chooser)
    {
    }

    @Override
    public <V> ISendableChooser<V> getSendableChooser()
    {
        return new FauxbotSendableChooser<V>();
    }

    @Override
    public IDoubleSubscriber getDoubleSubscriber(String key)
    {
        return null;
    }

    @Override
    public IBooleanSubscriber getBooleanSubscriber(String key)
    {
        return null;
    }

    @Override
    public IIntegerSubscriber getIntegerSubscriber(String key)
    {
        return null;
    }

    @Override
    public IStringSubscriber getStringSubscriber(String key)
    {
        return null;
    }
}
