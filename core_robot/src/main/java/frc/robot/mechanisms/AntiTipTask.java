package frc.robot.mechanisms;

import frc.robot.common.IMechanism;

public class AntiTipTask implements IMechanism{

    private PigeonManager imuManager;
    private double pitch;
    private double roll;

    @Override
    public void readSensors() {
        // TODO Auto-generated method stub
        this.imuManager.getPitch();
    }

    @Override
    public void update() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void stop() {
        // TODO Auto-generated method stub
        
    }
    
}
