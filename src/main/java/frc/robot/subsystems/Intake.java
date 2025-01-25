package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;

public class Intake extends SubsystemBase {
    private SparkMax intakeMotor;
    private DigitalInput limitSwitch;

    // Data log
    private DataLog dataLog;

    public Intake(DataLog dataLog){
        intakeMotor = new SparkMax(IntakeConstants.motorId, MotorType.kBrushless);
        limitSwitch = new DigitalInput(IntakeConstants.switchChannel);

        this.dataLog = dataLog;
    }

    public void setSpeed(double speed){
        intakeMotor.set(speed);
    }

    public void stop(){
        intakeMotor.stopMotor();
    }

    public boolean isSwitched(){
        return limitSwitch.get();
    }

    public void log(){
        // TODO add logs
    }
}