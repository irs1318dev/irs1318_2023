package frc.team1318.robot.simulation;

import java.io.FileInputStream;
import java.util.HashMap;
import java.util.Map;

import frc.team1318.robot.ElectronicsConstants;
import frc.team1318.robot.IRealWorldSimulator;
import frc.team1318.robot.common.robotprovider.*;

import com.google.inject.Inject;
import com.google.inject.Singleton;

import javafx.scene.canvas.Canvas;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.image.Image;
import javafx.scene.paint.Color;

@Singleton
public class ForkliftSimulator implements IRealWorldSimulator
{
    private static final int LeftMotorChannel = 0;
    private static final int RightMotorChannel = 1;
    private static final int LifterForwardChannel = 2;

    @SuppressWarnings("serial")
    private final Map<Integer, String> actuatorNameMap = new HashMap<Integer, String>()
    {
        {
            this.put(ForkliftSimulator.LeftMotorChannel, "F Left Drive Motor");
            this.put(ForkliftSimulator.RightMotorChannel, "Right Drive Motor");
            this.put(ForkliftSimulator.LifterForwardChannel, "Lifter solenoid");
        }
    };

    private double leftPower;
    private double rightPower;

    private boolean forkliftUp;

    private Image forkliftDownImage;
    private Image forkliftUpImage;

    @Inject
    public ForkliftSimulator()
    {
        this.leftPower = 0.0;
        this.rightPower = 0.0;

        // start with it either up or down
        this.forkliftUp = Math.random() >= 0.5;

        try
        {
            FileInputStream forkliftDownInputStream = new FileInputStream(this.getClass().getResource("/images/forklift_down.png").getPath());
            this.forkliftDownImage = new Image(forkliftDownInputStream);

            FileInputStream forkliftUpInputStream = new FileInputStream(this.getClass().getResource("/images/forklift_up.png").getPath());
            this.forkliftUpImage = new Image(forkliftUpInputStream);
        }
        catch (Exception e)
        {
            System.out.println("ERROR: INVALID IMAGE");             
        }
    }

    @Override
    public String getSensorName(int channel)
    {
        return "Sensor " + channel;
    }

    @Override
    public double getEncoderMin(int channel)
    {
        return -1.0;
    }

    @Override
    public double getEncoderMax(int channel)
    {
        return 1.0;
    }

    @Override
    public String getActuatorName(int channel)
    {
        if (this.actuatorNameMap.containsKey(channel))
        {
            return this.actuatorNameMap.get(channel);
        }

        return "Motor " + channel;
    }

    @Override
    public void update()
    {
        FauxbotActuatorBase leftDriveActuator = FauxbotActuatorManager.get(ForkliftSimulator.LeftMotorChannel);
        FauxbotActuatorBase rightDriveActuator = FauxbotActuatorManager.get(ForkliftSimulator.RightMotorChannel);
        FauxbotActuatorBase lifterActuator = FauxbotActuatorManager.get(ForkliftSimulator.LifterForwardChannel);

        if (leftDriveActuator != null && leftDriveActuator instanceof FauxbotMotorBase && rightDriveActuator != null && rightDriveActuator instanceof FauxbotMotorBase)
        {
            FauxbotMotorBase leftDriveMotor = (FauxbotMotorBase)leftDriveActuator;
            FauxbotMotorBase rightDriveMotor = (FauxbotMotorBase)rightDriveActuator;
            this.leftPower = leftDriveMotor.get();
            this.rightPower = rightDriveMotor.get();
        }

        if (lifterActuator != null && lifterActuator instanceof FauxbotDoubleSolenoid)
        {
            FauxbotDoubleSolenoid lifterSolenoid = (FauxbotDoubleSolenoid)lifterActuator;
            this.forkliftUp = lifterSolenoid.get() == DoubleSolenoidValue.kForward;
        }
    }

    @Override
    public void draw(Canvas canvas)
    {
        double canvasHeight = canvas.getHeight();
        double canvasWidth = canvas.getWidth();

        GraphicsContext gc = canvas.getGraphicsContext2D();
        gc.clearRect(0, 0, canvasWidth, canvasHeight);

        double halfHeight = canvasHeight / 2.0;
        double powerIndicatorWidth = canvasWidth / 10.0;

        double leftHeight = Math.abs(leftPower * halfHeight);
        double leftTop;
        if (leftPower > 0.0)
        {
            leftTop = halfHeight - leftHeight;
        }
        else
        {
            leftTop = halfHeight;
        }

        gc.setFill(Color.BLUE); 
        gc.fillRect(0, leftTop, powerIndicatorWidth, leftHeight);

        double rightHeight = Math.abs(rightPower * halfHeight);
        double rightTop;
        if (rightPower > 0.0)
        {
            rightTop = halfHeight - rightHeight;
        }
        else
        {
            rightTop = halfHeight;
        }

        gc.setFill(Color.RED); 
        gc.fillRect(powerIndicatorWidth * 2, rightTop, powerIndicatorWidth, rightHeight);

        // draw the forklift in its current state
        Image forkliftToDraw = null;
        if (this.forkliftUp)
        {
            forkliftToDraw = this.forkliftUpImage;
        }
        else
        {
            forkliftToDraw = this.forkliftDownImage;
        }

        gc.drawImage(
            forkliftToDraw,
            canvasWidth / 2.0,
            canvasHeight / 4.0, 
            canvasWidth / 2.0,
            canvasHeight / 2.0);
    } 
}
