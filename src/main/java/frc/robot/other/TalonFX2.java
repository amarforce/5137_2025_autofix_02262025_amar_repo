package frc.robot.other;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TalonFX2 extends TalonFX{
    private double mul;
    private double offset;
    private TalonFXSimState sim;

    public TalonFX2(int port,double mul,double offset,InvertedValue value,String canBus){
        super(port,canBus);
        var currentConfigs = new MotorOutputConfigs();
        currentConfigs.NeutralMode = NeutralModeValue.Brake;
        currentConfigs.Inverted = value;
        this.getConfigurator().apply(currentConfigs);
        this.mul=mul;
        this.offset=offset;
        sim=new TalonFXSimState(this,value==InvertedValue.Clockwise_Positive?ChassisReference.Clockwise_Positive:ChassisReference.CounterClockwise_Positive);
    }

    public double getPos(){
        return super.getPosition().getValueAsDouble()*mul+offset;
    }

    public double getVel(){
        return super.getVelocity().getValueAsDouble()*mul;
    }

    public double getAcc(){
        return super.getAcceleration().getValueAsDouble()*mul;
    }

    public void setPos(double pos){
        sim.setRawRotorPosition((pos-offset)/mul);
    }

    public void setVel(double vel){
        sim.setRotorVelocity(vel/mul);
    }

    public void setAcc(double acc){
        sim.setRotorAcceleration(acc/mul);
    }

    public void refreshSupplyVoltage(){
        sim.setSupplyVoltage(RobotController.getBatteryVoltage());
    }

    public double getSimVoltage(){
        return sim.getMotorVoltage();
    }

    public void log(String path){
        SmartDashboard.putNumber(path+"/output",get());
        SmartDashboard.putNumber(path+"/rawMeasurement", getPosition().getValueAsDouble());
        SmartDashboard.putNumber(path+"/measurement", getPos());
        SmartDashboard.putNumber(path+"/rawVelocity", getVelocity().getValueAsDouble());
        SmartDashboard.putNumber(path+"/velocity", getVel());
        SmartDashboard.putNumber(path+"/rawAcceleration", getAcceleration().getValueAsDouble());
        SmartDashboard.putNumber(path+"/acceleration", getAcc());
        SmartDashboard.putNumber(path+"/temp", getDeviceTemp().getValueAsDouble());
        SmartDashboard.putNumber(path+"/fault", getFaultField().asSupplier().get());
        SmartDashboard.putNumber(path+"/current", getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber(path+"/voltage", getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber(path+"/supplyVoltage", getSupplyVoltage().getValueAsDouble());
    }
}
