package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.GeneralConstants;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

/**
 * The Arm subsystem controls the robotic arm using a TalonFX motor controller.
 * It includes PID control, feedforward control, and simulation capabilities.
 */
public class Arm extends SubsystemBase {
    
    // Motor controller for the arm
    private TalonFX armMotor = new TalonFX(ArmConstants.motorId, "rhino");
    
    // PID controller for arm position control
    private PIDController armController = new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);
    
    // Feedforward controller for arm dynamics
    private ArmFeedforward feedforward = new ArmFeedforward(ArmConstants.kS, ArmConstants.kG, ArmConstants.kV);
    
    // Goal position for the arm in radians
    private double goal = ArmConstants.defaultGoal;
    
    // Simulation model for the arm
    private SingleJointedArmSim armSim = new SingleJointedArmSim(
        ArmConstants.motorSim, 
        ArmConstants.gearRatio, 
        ArmConstants.momentOfInertia, 
        ArmConstants.armLength, 
        ArmConstants.minAngle, 
        ArmConstants.maxAngle, 
        true, 
        ArmConstants.defaultGoal
    );

    // System Identification routine for characterizing the arm
    public final SysIdRoutine sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                this::setVoltage,
                log -> {
                    log.motor("arm")
                        .voltage(Volts.of(armMotor.get() * RobotController.getBatteryVoltage()))
                        .angularPosition(Radians.of(getMeasurement()))
                        .angularVelocity(RadiansPerSecond.of(getVelocity()));
                },
                this
            )
        );
    
    // Simulation state for the motor
    private TalonFXSimState armMotorSim = armMotor.getSimState();

    // Data log for recording arm data
    private DataLog dataLog;
        
    /**
     * Constructor for the Arm subsystem.
     * 
     * @param dataLog The data log to record arm data.
     */
    public Arm(DataLog dataLog) {
        // Configure the motor to coast when neutral
        var currentConfigs = new MotorOutputConfigs();
        currentConfigs.NeutralMode = NeutralModeValue.Coast;
        armMotor.getConfigurator().apply(currentConfigs);

        // Set the tolerance for the PID controller
        armController.setTolerance(ArmConstants.armTolerance);
        
        // Display the PID controller on SmartDashboard for tuning
        SmartDashboard.putData("Arm Controller", armController);

        this.dataLog = dataLog;
    }

    /**
     * Get the current arm position in radians.
     * 
     * @return The current arm position in radians.
     */
    public double getMeasurement() {
        return Units.rotationsToRadians(armMotor.getPosition().getValueAsDouble() / ArmConstants.gearRatio) - ArmConstants.armOffset;
    }
    
    /**
     * Set the goal position for the arm, clamping it within the allowed range.
     * 
     * @param newGoal The desired goal position in radians.
     */
    public void setGoal(double newGoal) {
        if (newGoal < ArmConstants.minAngle) {
            newGoal = ArmConstants.minAngle;
        }
        if (newGoal > ArmConstants.maxAngle) {
            newGoal = ArmConstants.maxAngle;
        }
        goal = newGoal;
    }

    /**
     * Get the current goal position.
     * 
     * @return The current goal position in radians.
     */
    public double getGoal() {
        return goal;
    }

    /**
     * Set the voltage applied to the arm motor.
     * 
     * @param v The voltage to apply.
     */
    public void setVoltage(Voltage v) {
        armMotor.setVoltage(v.magnitude());
    }

    /**
     * Get the current arm velocity in radians per second.
     * 
     * @return The current arm velocity in radians per second.
     */
    public double getVelocity() {
        return Units.rotationsToRadians(armMotor.getVelocity().getValueAsDouble() / ArmConstants.gearRatio);
    }

    /**
     * Display telemetry data on SmartDashboard.
     */
    public void telemetry() {
        SmartDashboard.putNumber("Arm Angle", armSim.getAngleRads());
        SmartDashboard.putNumber("Arm Velocity", armSim.getVelocityRadPerSec());
        SmartDashboard.putNumber("Arm Input", armMotor.get());
    }

    /**
     * Periodic method called every loop iteration.
     */
    @Override
    public void periodic() {
        // Update telemetry
        telemetry();
        
        // Calculate feedforward and PID control outputs
        double extra = feedforward.calculate(getMeasurement(), getVelocity());
        double voltage = armController.calculate(getMeasurement(), goal) + extra;
        
        // Apply the calculated voltage to the motor
        setVoltage(Volts.of(voltage));
    }

    /**
     * Simulation periodic method called every loop iteration in simulation.
     */
    @Override
    public void simulationPeriodic() {
        // Update the motor simulation state with the current battery voltage
        armMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        
        // Get the current motor input voltage and update the arm simulation
        double armInput = armMotorSim.getMotorVoltage();
        armSim.setInputVoltage(armInput);
        armSim.update(GeneralConstants.simPeriod);
        
        // Update the motor simulation state with the new arm position and velocity
        double angle = armSim.getAngleRads();
        armMotorSim.setRawRotorPosition(Units.radiansToRotations((angle + ArmConstants.armOffset) * ArmConstants.gearRatio));
        double vel = armSim.getVelocityRadPerSec();
        armMotorSim.setRotorVelocity(Units.radiansToRotations(vel * ArmConstants.gearRatio));
        
        // Update the RoboRIO simulation state with the new battery voltage
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(armSim.getCurrentDrawAmps()));
    }

    /**
     * Log arm data.
     * TODO: Implement logging functionality.
     */
    public void log() {

    }
}