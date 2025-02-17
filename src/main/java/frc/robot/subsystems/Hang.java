package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.HangConstants;
import frc.robot.other.RobotUtils;
import frc.robot.other.TalonFX2;

import edu.wpi.first.wpilibj.DataLogManager;

/**
 * The Hang subsystem controls the pneumatics for the robot's hanging mechanism.
 * This includes the clamp and climb solenoids, as well as the compressor that
 * provides the necessary air pressure.
 */
public class Hang extends SubsystemBase {
    
    private TalonFX2 hangMotor=new TalonFX2(HangConstants.motorId,1,0,true,"rio");

    /**
     * Constructs a new Hang subsystem.
     */
    public Hang() {
        
    }
    
    public void setSpeed(double speed){
        hangMotor.set(speed);
    }

    /**
     * Logs relevant data from the Hang subsystem.
     * This method is currently a placeholder and should be implemented to log specific data.
     */
    private void telemetry(){
        hangMotor.log("hang/motor");
    }

    @Override
    public void periodic() {
        try{
            telemetry();
        }catch(Exception e){
            DataLogManager.log("Periodic error: "+RobotUtils.getError(e));
        }
    }
}