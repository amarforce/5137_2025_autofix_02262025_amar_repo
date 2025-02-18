package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.SwerveSystemConstants;
import frc.robot.motorSystem.EnhancedTalonFX;
import frc.robot.motorSystem.EnhancedEncoder;
import frc.robot.motorSystem.MotorSystem;
import frc.robot.motorSystem.ArmMechanismSim;
import frc.robot.constants.GeneralConstants;
import frc.robot.other.RobotUtils;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.List;

/**
 * The Arm subsystem controls a single-jointed robotic arm using a TalonFX motor controller.
 * It provides position control through a combination of PID and feedforward control.
 * 
 * Features:
 * - Position control using ProfiledPIDController for smooth motion
 * - Gravity compensation and feedforward control for accurate positioning
 * - Simulation support using SingleJointedArmSim
 * - System identification capabilities for tuning
 * - Telemetry output for debugging and monitoring
 * 
 * The arm's position is measured in radians, where:
 * - 0 radians = vertical position
 * - Positive angles = forward from vertical
 * - Negative angles = backward from vertical
 */
public class Arm extends SubsystemBase {
    
    /** Motor system that handles both the motor and encoder */
    private final MotorSystem motorSystem;
    
    /** PID controller for arm position control */
    private final ProfiledPIDController controller;
    
    /** Feedforward controller for gravity compensation and dynamics */
    private final ArmFeedforward feedforward;
    
    /** Current goal position for the arm in radians */
    private double goal;

    /** Simulation model for the arm's physics */
    private final SingleJointedArmSim armSim;

    /** Adapter to make SingleJointedArmSim compatible with MotorSystem */
    private final ArmMechanismSim mechanismSim;

    /** System identification routine for parameter tuning */
    private final SysIdRoutine sysIdRoutine;
        
    /**
     * Constructor for the Arm subsystem.
     * Initializes all control systems, motors, encoders, and simulation components.
     */
    public Arm() {
        // Create motor and encoder
        EnhancedTalonFX armMotor = new EnhancedTalonFX(
            ArmConstants.motorId, 
            "rio", 
            (2*Math.PI)/ArmConstants.gearRatio, 
            false, 
            false
        );
        EnhancedEncoder armEncoder = new EnhancedEncoder(
            ArmConstants.encoderId, 
            (2*Math.PI)/ArmConstants.encoderRatio, 
            ArmConstants.encoderOffset
        );
        
        // Create motor system
        motorSystem = new MotorSystem(List.of(armMotor), armEncoder);
        
        // Initialize PID controller
        controller = new ProfiledPIDController(
            ArmConstants.kP, 
            ArmConstants.kI, 
            ArmConstants.kD, 
            ArmConstants.pidConstraints
        );
        controller.setTolerance(ArmConstants.armTolerance);
        
        // Initialize feedforward controller
        feedforward = new ArmFeedforward(
            ArmConstants.kS, 
            ArmConstants.kG, 
            ArmConstants.kV
        );
        
        // Set initial goal position
        goal = SwerveSystemConstants.getDefaultState().armPosition;

        // Initialize simulation components
        armSim = new SingleJointedArmSim(
            ArmConstants.motorSim, 
            ArmConstants.gearRatio, 
            ArmConstants.momentOfInertia,
            ArmConstants.armLength,
            ArmConstants.minAngle,
            ArmConstants.maxAngle,
            false,
            SwerveSystemConstants.getDefaultState().armPosition
        );
        mechanismSim = new ArmMechanismSim(armSim);

        // Initialize system identification routine
        sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                null,        // Use default ramp rate (1 V/s)
                Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
                null        // Use default timeout (10 s)
            ),
            new SysIdRoutine.Mechanism(
                this::setVoltage,
                log -> {
                    log.motor("arm")
                        .voltage(getVolts())
                        .angularPosition(Radians.of(getMeasurement()+ArmConstants.feedOffset)) // Offset so that 0 = horizontal
                        .angularVelocity(RadiansPerSecond.of(getVelocity()))
                        .angularAcceleration(RadiansPerSecondPerSecond.of(getAcceleration()));
                },
                this
            )
        );
    }

    /**
     * Get the current arm position in radians.
     * 
     * @return The current arm position in radians, where 0 is vertical,
     *         positive values are forward, and negative values are backward.
     */
    public double getMeasurement() {
        return motorSystem.getMeasurement();
    }
    
    /**
     * Set the goal position for the arm, clamping it within the allowed range.
     * 
     * @param newGoal The desired goal position in radians. Will be clamped between
     *                ArmConstants.minAngle and ArmConstants.maxAngle.
     */
    public void setGoal(double newGoal) {
        goal = RobotUtils.clamp(newGoal, ArmConstants.minAngle, ArmConstants.maxAngle);
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
     * @param v The voltage to apply to the motor.
     */
    public void setVoltage(Voltage v) {
        motorSystem.setVoltage(v);
    }

    /**
     * Gets the current voltage applied to the arm motor.
     * 
     * @return The voltage currently being applied to the motor.
     */
    public Voltage getVolts() {
        return motorSystem.getVolts();
    }

    /**
     * Get the current arm velocity in radians per second.
     * 
     * @return The current angular velocity in radians per second.
     */
    public double getVelocity() {
        return motorSystem.getVelocity();
    }

    /**
     * Get the current arm acceleration in radians per second squared.
     * 
     * @return The current angular acceleration in radians per second squared.
     */
    public double getAcceleration() {
        return motorSystem.getAcceleration();
    }

    /**
     * Check if the arm is at its target position.
     * 
     * @return true if the arm is within the tolerance of its goal position,
     *         false otherwise.
     */
    public boolean atSetpoint() {
        return controller.atSetpoint();
    }

    /**
     * Get the system identification routine for tuning.
     * 
     * @return The SysId routine configured for this arm.
     */
    public SysIdRoutine getRoutine() {
        return sysIdRoutine;
    }

    /**
     * Display telemetry data on SmartDashboard.
     * Outputs current position, goal, velocity, and error information
     * for debugging and monitoring.
     */
    public void telemetry() {
        SmartDashboard.putNumber("arm/angle", getMeasurement());
        SmartDashboard.putNumber("arm/degrees", Units.radiansToDegrees(getMeasurement()));
        SmartDashboard.putNumber("arm/goal", getGoal());
        SmartDashboard.putNumber("arm/setpoint", controller.getSetpoint().position);
        SmartDashboard.putNumber("arm/setpointVelocity",controller.getSetpoint().velocity);
        SmartDashboard.putNumber("arm/velocity", getVelocity());
        SmartDashboard.putNumber("arm/error", controller.getPositionError());
        SmartDashboard.putData("arm/controller", controller);
        motorSystem.log("arm");
    }

    /**
     * Periodic method called every loop iteration.
     * Updates the arm's position using PID and feedforward control.
     */
    @Override
    public void periodic() {
        try {
            motorSystem.periodic();

            // Update telemetry
            telemetry();
            
            // Calculate feedforward and PID control outputs
            double feed = feedforward.calculate(
                controller.getSetpoint().position + ArmConstants.feedOffset, // Offset so that 0 = horizontal
                controller.getSetpoint().velocity
            );
            double voltage = controller.calculate(getMeasurement(), goal) + feed;
            
            // Apply the calculated voltage to the motor
            setVoltage(Volts.of(voltage));
        } catch (Exception e) {
            DataLogManager.log("Periodic error: " + RobotUtils.getError(e));
        }
    }

    /**
     * Simulation periodic method called every loop iteration in simulation.
     * Updates the simulated arm physics using the MotorSystem and ArmMechanismSim.
     */
    @Override
    public void simulationPeriodic() {
        motorSystem.simulationPeriodic(mechanismSim, GeneralConstants.simPeriod);
    }
}