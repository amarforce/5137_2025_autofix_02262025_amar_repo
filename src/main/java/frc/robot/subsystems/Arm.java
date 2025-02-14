package frc.robot.subsystems;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.SwerveSystemConstants;
import frc.robot.constants.GeneralConstants;
import frc.robot.other.RobotUtils;
import frc.robot.other.RolloverEncoder;
import frc.robot.other.TalonFX2;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

/**
 * The Arm subsystem controls the robotic arm using a TalonFX motor controller.
 * It includes PID control, feedforward control, and simulation capabilities.
 */
public class Arm extends SubsystemBase {
    
    // Motor controller for the arm
    private TalonFX2 armMotor = new TalonFX2(ArmConstants.motorId,(2*Math.PI)/ArmConstants.gearRatio,0,InvertedValue.CounterClockwise_Positive,"rio");
    private RolloverEncoder armEncoder = new RolloverEncoder(ArmConstants.encoderId, (2*Math.PI)/ArmConstants.encoderRatio, ArmConstants.encoderOffset);
    
    // PID controller for arm position control
    private ProfiledPIDController controller = new ProfiledPIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD, ArmConstants.pidConstraints);
    
    // Feedforward controller for arm dynamics
    private ArmFeedforward feedforward = new ArmFeedforward(ArmConstants.kS, ArmConstants.kG, ArmConstants.kV);
    
    // Goal position for the arm in radians
    private double goal = SwerveSystemConstants.getDefaultState().armPosition;

    // Simulation model for the arm
    private SingleJointedArmSim armSim = new SingleJointedArmSim(
        ArmConstants.motorSim, 
        ArmConstants.gearRatio, 
        ArmConstants.momentOfInertia, 
        ArmConstants.armLength, 
        ArmConstants.minAngle, 
        ArmConstants.maxAngle, 
        false, 
        SwerveSystemConstants.getDefaultState().armPosition
    );

    // SysId routine for system identification
    private final SysIdRoutine sysIdRoutine = 
        new SysIdRoutine(
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
                        .angularPosition(Radians.of(getMeasurement()+Math.PI/2))
                        .angularVelocity(RadiansPerSecond.of(getVelocity()))
                        .angularAcceleration(RadiansPerSecondPerSecond.of(getAcceleration()));
                },
                this));
        
    /**
     * Constructor for the Arm subsystem.
     */
    public Arm() {
        // Set the tolerance for the PID controller
        controller.setTolerance(ArmConstants.armTolerance);
        
        // Display the PID controller on SmartDashboard for tuning
        SmartDashboard.putData("arm/controller", controller);
    }

    /**
     * Get the current arm position in radians.
     * 
     * @return The current arm position in radians.
     */
    public double getMeasurement() {
        return armEncoder.get();
    }
    
    /**
     * Set the goal position for the arm, clamping it within the allowed range.
     * 
     * @param newGoal The desired goal position in radians.
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
     * @param v The voltage to apply.
     */
    public void setVoltage(Voltage v) {
        armMotor.setVoltage(v.magnitude());
    }

    /**
     * Gets the current voltage applied to the arm motor.
     * 
     * @return The voltage applied to the motors.
     */
    public Voltage getVolts() {
        return armMotor.getMotorVoltage().getValue();
    }

    /**
     * Get the current arm velocity in radians per second.
     * 
     * @return The current arm velocity in radians per second.
     */
    public double getVelocity() {
        return armMotor.getVel();
    }

    /**
     * Get the current arm acceleration in radians per second^2.
     * 
     * @return The current arm acceleration in radians per second^2.
     */
    public double getAcceleration() {
        return armMotor.getAcc();
    }

    public boolean atSetpoint() {
        return controller.atSetpoint();
    }

    public SysIdRoutine getRoutine() {
        return sysIdRoutine;
    }

    /**
     * Display telemetry data on SmartDashboard.
     */
    public void telemetry() {
        SmartDashboard.putNumber("arm/angle", getMeasurement());
        SmartDashboard.putNumber("arm/degrees", Units.radiansToDegrees(getMeasurement()));
        SmartDashboard.putNumber("arm/goal", getGoal());
        SmartDashboard.putNumber("arm/setpoint", controller.getSetpoint().position);
        SmartDashboard.putNumber("arm/setpointVelocity",controller.getSetpoint().velocity);
        SmartDashboard.putNumber("arm/velocity", getVelocity());
        SmartDashboard.putNumber("arm/error", controller.getPositionError());
        SmartDashboard.putNumber("arm/motor/output",armMotor.get());
        SmartDashboard.putNumber("arm/motor/rawAngle", armMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("arm/motor/temp", armMotor.getDeviceTemp().getValueAsDouble());
        SmartDashboard.putNumber("arm/motor/fault", armMotor.getFaultField().asSupplier().get());
        SmartDashboard.putNumber("arm/motor/current", armMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("arm/motor/voltage", armMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("arm/motor/supplyVoltage", armMotor.getSupplyVoltage().getValueAsDouble());
    }

    /**
     * Periodic method called every loop iteration.
     */
    @Override
    public void periodic() {
        try {
            // Update telemetry
            telemetry();
            
            // Calculate feedforward and PID control outputs
            double feed = feedforward.calculate(controller.getSetpoint().position+ArmConstants.feedOffset, controller.getSetpoint().velocity); // Offset so that 0 = horizontal
            double voltage = controller.calculate(getMeasurement(), goal) + feed;
            
            // Apply the calculated voltage to the motor
            setVoltage(Volts.of(voltage));

            armEncoder.periodic();
        } catch (Exception e) {
            DataLogManager.log("Periodic error: " + RobotUtils.getError(e));
        }
    }

    /**
     * Simulation periodic method called every loop iteration in simulation.
     */
    @Override
    public void simulationPeriodic() {
        // Update the motor simulation state with the current battery voltage
        armMotor.refreshSupplyVoltage();
        
        // Get the current motor input voltage and update the Wrist simulation
        double armInput = armMotor.getSimVoltage();
        armSim.setInputVoltage(armInput);
        armSim.update(GeneralConstants.simPeriod);
        
        // Update the motor simulation state with the new Wrist position and velocity
        armMotor.setPos(armSim.getAngleRads());
        armMotor.setVel(armSim.getVelocityRadPerSec());

        armEncoder.set(armSim.getAngleRads());

        // Update the RoboRIO simulation state with the new battery voltage
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(armSim.getCurrentDrawAmps()));
    }
}