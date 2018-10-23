package frc.team1318.robot;

import frc.team1318.robot.common.robotprovider.*;
import javafx.scene.canvas.Canvas;

public interface IRealWorldSimulator
{
    public String getSensorName(FauxbotSensorConnection connection);
    public double getEncoderMin(FauxbotSensorConnection connection);
    public double getEncoderMax(FauxbotSensorConnection connection);
    public String getActuatorName(FauxbotActuatorConnection connection);
    public void update();
    public void draw(Canvas canvas);
}
