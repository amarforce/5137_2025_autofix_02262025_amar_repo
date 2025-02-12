package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;
import frc.robot.other.RobotUtils;

/**
 * The Intake subsystem controls the intake mechanism of the robot.
 * It includes a motor to control the intake mechanism and a limit switch
 * to detect when the intake is in a specific position.
 */
public class Intake extends SubsystemBase {
    // Motor controller for the intake mechanism
    private TalonFX intakeMotor;

    // Limit switch to detect the position of the intake
    //sprivate DigitalInput limitSwitch;

    /**
     * Constructs an Intake subsystem.
     */
    public Intake() {
        // Initialize the motor controller with the ID and type from constants
        intakeMotor = new TalonFX(IntakeConstants.motorId, "rio");

        // Initialize the limit switch with the channel from constants
        //limitSwitch = new DigitalInput(IntakeConstants.switchChannel);
        var currentConfigs = new MotorOutputConfigs();
        currentConfigs.NeutralMode = NeutralModeValue.Coast;
        currentConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
        intakeMotor.getConfigurator().apply(currentConfigs);
    }

    /**
     * Sets the speed of the intake motor.
     *
     * @param speed The speed to set the motor to, in the range [-1.0, 1.0].
     */
    public void setSpeed(double speed) {
        intakeMotor.set(speed);
    }

    /**
     * Stops the intake motor.
     */
    public void stop() {
        intakeMotor.stopMotor();
    }

    /**
     * Checks if the limit switch is triggered.
     *
     * @return True if the limit switch is triggered, false otherwise.
     */
    public boolean isSwitched() {
        //return limitSwitch.get();
        return false;
    }

    /**
     * Logs relevant data from the Intake subsystem.
     * This method is intended to be called periodically to record data.
     */
    public void telemetry(){
        SmartDashboard.putNumber("intake/motor/rawAngle", intakeMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("intake/motor/output", intakeMotor.get());
        SmartDashboard.putNumber("intake/motor/temp", intakeMotor.getDeviceTemp().getValueAsDouble());
        SmartDashboard.putNumber("intake/motor/fault", intakeMotor.getFaultField().asSupplier().get());
        SmartDashboard.putNumber("intake/motor/current", intakeMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("intake/motor/voltage", intakeMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("intake/motor/supplyVoltage", intakeMotor.getSupplyVoltage().getValueAsDouble());
    }

    @Override
    public void periodic(){
        try{
            telemetry();
        }catch(Exception e){
            DataLogManager.log("Periodic error: "+RobotUtils.getError(e));
        }
    }
}