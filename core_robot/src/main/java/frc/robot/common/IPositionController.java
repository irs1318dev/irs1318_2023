package frc.robot.common;

/**
 * This class is a PID handler with a feed-forward handler and a complementary filter.
 * 
 * To use PID control:
 *      set the kp/ki/kd/kf tuning values
 *      calculate output based on the setpoint and measured value regularly
 * 
 * for reference:
 *      http://en.wikipedia.org/wiki/PID_controller
 *      http://en.wikipedia.org/wiki/Feed_forward_(control)
 * 
 * @author Will (adapted from old code)
 */
public interface IPositionController
{
    /**
     * Calculate the desired output value based on the history, setpoint, and measured value.
     * measuredValue should be in the same unit as the setpoint - if they are not that should be accounted-for in the ks value.
     * This method should be called in a loop and fed feedback data and setpoint changes
     * 
     * @param setpoint describes the goal value
     * @param measuredValue describes the measured value
     * @return output value to be used
     */
    double calculatePosition(double setpoint, double measuredValue);

    /**
     * The current output from the 
     * @return
     */
    double getOutput();

    double getError();

    void reset();
}
