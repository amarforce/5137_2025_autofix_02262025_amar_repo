package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.WristConstants;
import frc.robot.other.RobotUtils;
import frc.robot.constants.ArmConstants;
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
    private TalonFX wristMotor = new TalonFX(WristConstants.motorId, "rio");
    
    // PID controller for Wrist position control
    private PIDController controller = new PIDController(WristConstants.kP, WristConstants.kI, WristConstants.kD);
    
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
    private final SysIdRoutine sysIdRoutine =
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
    private TalonFXSimState wristMotorSim = wristMotor.getSimState();

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
        controller.setTolerance(WristConstants.wristTolerance);
        
        // Display the PID controller on SmartDashboard for tuning
        SmartDashboard.putData("Wrist Controller", controller);

        this.log=log;
    }

    /**
     * Gets the current Wrist position in radians.
     * 
     * @return The current position of the wrist in radians.
     */
    public double getMeasurement() {
        return WristConstants.transform.transformPos(wristMotor.getPosition().getValueAsDouble());
    }
    
    /**
     * Sets the goal position for the Wrist, clamping it within the allowed range.
     * 
     * @param newGoal The desired goal position in radians.
     */
    public void setGoal(double newGoal) {
        goal = RobotUtils.clamp(newGoal,WristConstants.minAngle,WristConstants.maxAngle);
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
        return WristConstants.transform.transformVel(wristMotor.getVelocity().getValueAsDouble());
    }

    public SysIdRoutine getRoutine(){
        return sysIdRoutine;
    }

    /**
     * Displays telemetry data on SmartDashboard.
     */
    public void telemetry() {
        SmartDashboard.putNumber("wrist/angle",getMeasurement());
        SmartDashboard.putNumber("wrist/goal",getGoal());
        SmartDashboard.putNumber("wrist/velocity",getVelocity());
        SmartDashboard.putNumber("wrist/error",controller.getError());
        SmartDashboard.putNumber("wrist/motorTemp",wristMotor.getDeviceTemp().getValueAsDouble());
        int motorFault = wristMotor.getFaultField().asSupplier().get();
        if (motorFault != 0){
            SmartDashboard.putNumber("wrist/motorFault",motorFault);
        }
        SmartDashboard.putNumber("wrist/current",wristMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("wrist/voltage",wristMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("wrist/current",wristMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("wrist/supplyVoltage",wristMotor.getSupplyVoltage().getValueAsDouble());
    }

    /**
     * Periodic method called every loop iteration.
     * Updates telemetry and calculates PID control outputs.
     */
    @Override
    public void periodic() {
        try {
            // Update telemetry
            telemetry();
        
            // Calculate PID control outputs
            double voltage = controller.calculate(getMeasurement(), goal);
            
            // Apply the calculated voltage to the motor
            setVoltage(Volts.of(voltage));
            
        } catch (Exception e) {
            log.append("Periodic error: " + RobotUtils.getError(e));
        }

    }

    /**
     * Simulation periodic method called every loop iteration in simulation.
     * Updates the motor simulation state and the Wrist simulation.
     */
    @Override
    public void simulationPeriodic() {
        // Update the motor simulation state with the current battery voltage
        wristMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        
        // Get the current motor input voltage and update the Wrist simulation
        double WristInput = wristMotorSim.getMotorVoltage();
        wristSim.setInputVoltage(WristInput);
        wristSim.update(GeneralConstants.simPeriod);
        
        // Update the motor simulation state with the new Wrist position and velocity
        wristMotorSim.setRawRotorPosition(WristConstants.transform.transformPosInv(wristSim.getAngleRads()));
        wristMotorSim.setRotorVelocity(WristConstants.transform.transformVelInv(wristSim.getVelocityRadPerSec()));
        
        // Update the RoboRIO simulation state with the new battery voltage
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(wristSim.getCurrentDrawAmps()));
    }
}