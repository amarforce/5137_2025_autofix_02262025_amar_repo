package frc.robot.other;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RolloverEncoder extends DutyCycleEncoder{
    private double last=-1;
    private double shift=0;
    private double fullRange;
    private int invertedVal;
    private DutyCycleEncoderSim sim;

    // fullRange = meters/rotation
    public RolloverEncoder(int channel,double fullRange,double offset,boolean inverted){
        super(channel,fullRange,RobotUtils.mod((inverted?1:-1)*offset,fullRange));
        invertedVal=inverted?-1:1;
        this.fullRange=fullRange;
        sim=new DutyCycleEncoderSim(this);
    }

    @Override
    public double get(){
        return invertedVal*(super.get()+shift);
    }

    public void periodic(){
        double pos=super.get();
        if(last>-1 && Math.abs(last-pos)>fullRange/2){
            if(last<pos){
                shift-=fullRange;
            }else{
                shift+=fullRange;
            }
        }
        last=pos;
    }

    public void set(double val){
        double encoderVal=RobotUtils.mod(val,fullRange);
        sim.set(invertedVal*encoderVal);
        shift=invertedVal*(val-encoderVal);
    }

    public void log(String path){
        SmartDashboard.putBoolean(path+"/connected", isConnected());
        SmartDashboard.putNumber(path+"/rawPosition", super.get());
        SmartDashboard.putNumber(path+"/position", get());
    }
}
