package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.WristConstants;
import frc.robot.constants.GeneralConstants;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

/**
 * The Wrist subsystem controls the wrist joint of the robot arm.
 * It uses a PID controller to manage the position of the wrist and includes
 * simulation support for testing and tuning.
 */
public class Wrist extends SubsystemBase {
    
    // Motor controller for the Wrist
    private TalonFX wristMotor = new TalonFX(WristConstants.motorId, "rhino");
    
    // PID controller for Wrist position control
    private PIDController wristController = new PIDController(WristConstants.kP, WristConstants.kI, WristConstants.kD);
    
    // Goal position for the Wrist in radians
    private double goal = WristConstants.pos1;
    
    // Simulation model for the Wrist
    private SingleJointedArmSim wristSim = new SingleJointedArmSim(
        WristConstants.motorSim, 
        WristConstants.gearRatio, 
        WristConstants.momentOfInertia, 
        WristConstants.wristLength, 
        WristConstants.minAngle, 
        WristConstants.maxAngle, 
        true, 
        WristConstants.pos1
    );

    // System Identification routine for characterizing the Wrist
    public final SysIdRoutine sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                this::setVoltage,
                log -> {
                    log.motor("Wrist")
                        .voltage(Volts.of(wristMotor.get() * RobotController.getBatteryVoltage()))
                        .angularPosition(Radians.of(getMeasurement()))
                        .angularVelocity(RadiansPerSecond.of(getVelocity()));
                },
                this
            )
        );
    
    // Simulation state for the motor
    private TalonFXSimState WristMotorSim = wristMotor.getSimState();

    private StringLogEntry log;
        
    /**
     * Constructor for the Wrist subsystem.
     */
    public Wrist(StringLogEntry log) {
        // Configure the motor to coast when neutral
        var currentConfigs = new MotorOutputConfigs();
        currentConfigs.NeutralMode = NeutralModeValue.Coast;
        wristMotor.getConfigurator().apply(currentConfigs);

        // Set the tolerance for the PID controller
        wristController.setTolerance(WristConstants.wristTolerance);
        
        // Display the PID controller on SmartDashboard for tuning
        SmartDashboard.putData("Wrist Controller", wristController);

        this.log=log;
    }

    /**
     * Gets the current Wrist position in radians.
     * 
     * @return The current position of the wrist in radians.
     */
    public double getMeasurement() {
        return Units.rotationsToRadians(wristMotor.getPosition().getValueAsDouble() / WristConstants.gearRatio) - WristConstants.wristOffset;
    }
    
    /**
     * Sets the goal position for the Wrist, clamping it within the allowed range.
     * 
     * @param newGoal The desired goal position in radians.
     */
    public void setGoal(double newGoal) {
        if (newGoal < WristConstants.minAngle) {
            newGoal = WristConstants.minAngle;
        }
        if (newGoal > WristConstants.maxAngle) {
            newGoal = WristConstants.maxAngle;
        }
        goal = newGoal;
    }

    /**
     * Gets the current goal position.
     * 
     * @return The current goal position in radians.
     */
    public double getGoal() {
        return goal;
    }

    /**
     * Sets the voltage applied to the Wrist motor.
     * 
     * @param v The voltage to apply to the motor.
     */
    public void setVoltage(Voltage v) {
        wristMotor.setVoltage(v.magnitude());
    }

    /**
     * Gets the current Wrist velocity in radians per second.
     * 
     * @return The current velocity of the wrist in radians per second.
     */
    public double getVelocity() {
        return Units.rotationsToRadians(wristMotor.getVelocity().getValueAsDouble() / WristConstants.gearRatio);
    }

    /**
     * Displays telemetry data on SmartDashboard.
     */
    public void telemetry() {
        SmartDashboard.putNumber("Wrist Angle", wristSim.getAngleRads());
        SmartDashboard.putNumber("Wrist Velocity", wristSim.getVelocityRadPerSec());
        SmartDashboard.putNumber("Wrist Input", wristMotor.get());
    }

    /**
     * Periodic method called every loop iteration.
     * Updates telemetry and calculates PID control outputs.
     */
    @Override
    public void periodic() {
        // Update telemetry
        telemetry();
        
        // Calculate PID control outputs
        double voltage = wristController.calculate(getMeasurement(), goal);
        
        // Apply the calculated voltage to the motor
        setVoltage(Volts.of(voltage));
    }

    /**
     * Simulation periodic method called every loop iteration in simulation.
     * Updates the motor simulation state and the Wrist simulation.
     */
    @Override
    public void simulationPeriodic() {
        // Update the motor simulation state with the current battery voltage
        WristMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        
        // Get the current motor input voltage and update the Wrist simulation
        double WristInput = WristMotorSim.getMotorVoltage();
        wristSim.setInputVoltage(WristInput);
        wristSim.update(GeneralConstants.simPeriod);
        
        // Update the motor simulation state with the new Wrist position and velocity
        double angle = wristSim.getAngleRads();
        WristMotorSim.setRawRotorPosition(Units.radiansToRotations((angle + WristConstants.wristOffset) * WristConstants.gearRatio));
        double vel = wristSim.getVelocityRadPerSec();
        WristMotorSim.setRotorVelocity(Units.radiansToRotations(vel * WristConstants.gearRatio));
        
        // Update the RoboRIO simulation state with the new battery voltage
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(wristSim.getCurrentDrawAmps()));
    }

    /**
     * Logs subsystem data.
     * TODO: Implement logging functionality.
     */
    public void log() {
        // TODO: Add logs
        log.append("Angle: " + getMeasurement());
        log.append("Goal: " + getGoal());
        log.append("Error: " + wristController.getError());
        log.append("Temp: " + wristMotor.getDeviceTemp().getValueAsDouble());
        int motorFault = wristMotor.getFaultField().asSupplier().get();
        if (motorFault != 0){
            log.append("Fault: " + motorFault);
        }
        log.append("Current: " + wristMotor.getSupplyCurrent());
        log.append("Voltage: " + wristMotor.getMotorVoltage().getValueAsDouble());
        log.append("Supply Voltage: " + wristMotor.getSupplyVoltage());
    }
}