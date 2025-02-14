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
import frc.robot.constants.WristConstants;
import frc.robot.other.RobotUtils;
import frc.robot.other.RolloverEncoder;
import frc.robot.other.TalonFX2;
import frc.robot.constants.GeneralConstants;
import frc.robot.constants.SwerveSystemConstants;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

/**k
 * The Wrist subsystem controls the wrist joint of the robot arm.
 * It uses a PID controller to manage the position of the wrist and includes
 * simulation support for testing and tuning.
 */
public class Wrist extends SubsystemBase {
    
    // Motor controller for the Wrist
    private TalonFX2 wristMotor = new TalonFX2(WristConstants.motorId,(2*Math.PI)/WristConstants.gearRatio,0,InvertedValue.CounterClockwise_Positive,"rio");
    private RolloverEncoder wristEncoder = new RolloverEncoder(WristConstants.encoderId, (2*Math.PI)/WristConstants.encoderRatio, WristConstants.encoderOffset);

    // PID controller for Wrist position control
    private ProfiledPIDController controller = new ProfiledPIDController(WristConstants.kP, WristConstants.kI, WristConstants.kD,WristConstants.pidConstraints);

    // Feedforward controller for arm dynamics
    private ArmFeedforward feedforward = new ArmFeedforward(WristConstants.kS, WristConstants.kG, WristConstants.kV);
    
    // Goal position for the Wrist in radians
    private double goal = SwerveSystemConstants.getDefaultState().wristPosition;

    // Simulation model for the Wrist
    private SingleJointedArmSim wristSim = new SingleJointedArmSim(
        WristConstants.motorSim,
        WristConstants.gearRatio,
        WristConstants.momentOfInertia,
        WristConstants.wristLength,
        WristConstants.minAngle,
        WristConstants.maxAngle,
        false,
        SwerveSystemConstants.getDefaultState().wristPosition
    );

    // SysId routine for system identification
    private final SysIdRoutine sysIdRoutine = 
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.2).div(Seconds.of(1.0)),        // Use default ramp rate (1 V/s)
                Volts.of(1), // Reduce dynamic step voltage to 4 V to prevent brownout
                null        // Use default timeout (10 s)
            ),
            new SysIdRoutine.Mechanism(
                this::setVoltage,
                log -> {
                    log.motor("wrist")
                        .voltage(getVolts())
                        .angularPosition(Radians.of(getMeasurement()+Math.PI/2))
                        .angularVelocity(RadiansPerSecond.of(getVelocity()))
                        .angularAcceleration(RadiansPerSecondPerSecond.of(getAcceleration()));
                },
                this));

    private Arm arm;
        
    /**
     * Constructor for the Wrist subsystem.
     * @param arm The arm subsystem that this wrist is attached to
     */
    public Wrist(Arm arm) {
        // Set the tolerance for the PID controller
        controller.setTolerance(WristConstants.wristTolerance);
        
        // Display the PID controller on SmartDashboard for tuning
        SmartDashboard.putData("wrist/controller", controller);

        this.arm=arm;
    }
    /**
     * Gets the current Wrist position in radians.
     * 
     * @return The current position of the wrist in radians.
     */
    public double getMeasurement() {
        return wristEncoder.get();
    }

    /**
     * Gets the true Wrist position in radians to use with feedforward.
     * 
     * @return The current true position of the wrist in radians.
     */
    public double getAdjustedMeasurement() {
        return this.getMeasurement() + arm.getMeasurement();
    }
    
    /**
     * Sets the goal position for the Wrist, clamping it within the allowed range.
     * 
     * @param newGoal The desired goal position in radians.
     */
    public void setGoal(double newGoal) {
        goal = RobotUtils.clamp(newGoal, WristConstants.minAngle, WristConstants.maxAngle);
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
     * Gets the true goal in radians to use with feedforward.
     * 
     * @return The current true goal in radians.
     */
    public double getAdjustedGoal() {
        return this.getGoal() + arm.getGoal();
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
        return wristMotor.getVel();
    }

    /**
     * Gets the current voltage applied to the arm motor.
     * 
     * @return The voltage applied to the motors.
     */
    public Voltage getVolts() {
        return wristMotor.getMotorVoltage().getValue();
    }

    /**
     * Gets the current Wrist acceleration in radians per second^2.
     * 
     * @return The current acceleration of the wrist in radians per second^2.
     */
    public double getAcceleration() {
        return wristMotor.getAcc();
    }

    public boolean atSetpoint() {
        return controller.atSetpoint();
    }

    public SysIdRoutine getRoutine() {
        return sysIdRoutine;
    }

    /**
     * Displays telemetry data on SmartDashboard.
     */
    public void telemetry() {
        SmartDashboard.putNumber("wrist/angle",getMeasurement());
        SmartDashboard.putNumber("wrist/degrees", Units.radiansToDegrees(getMeasurement()));
        SmartDashboard.putNumber("wrist/goal",getGoal());
        SmartDashboard.putNumber("wrist/setpoint",controller.getSetpoint().position);
        SmartDashboard.putNumber("wrist/setpointVelocity",controller.getSetpoint().velocity);
        SmartDashboard.putNumber("wrist/velocity",getVelocity());
        SmartDashboard.putNumber("wrist/error",controller.getPositionError());
        wristMotor.log("wrist/motor");
        wristEncoder.log("wrist/encoder");
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
            double armRotation=arm!=null?arm.getMeasurement():0;
            double feed = feedforward.calculate(controller.getSetpoint().position+armRotation+WristConstants.feedOffset, controller.getSetpoint().velocity);
            double voltage = controller.calculate(getMeasurement(), goal) + feed;
            
            // Apply the calculated voltage to the motor
            setVoltage(Volts.of(voltage));

            wristEncoder.periodic();

        } catch (Exception e) {
            DataLogManager.log("Periodic error: " + RobotUtils.getError(e));
        }
    }

    /**
     * Simulation periodic method called every loop iteration in simulation.
     * Updates the motor simulation state and the Wrist simulation.
     */
    @Override
    public void simulationPeriodic() {
        // Update the motor simulation state with the current battery voltage
        wristMotor.refreshSupplyVoltage();
        
        // Get the current motor input voltage and update the Wrist simulation
        double wristInput = wristMotor.getSimVoltage();
        wristSim.setInputVoltage(wristInput);
        wristSim.update(GeneralConstants.simPeriod);
        
        // Update the motor simulation state with the new Wrist position and velocity
        wristMotor.setPos(wristSim.getAngleRads());
        wristMotor.setVel(wristSim.getVelocityRadPerSec());

        wristEncoder.set(wristSim.getAngleRads());

        // Update the RoboRIO simulation state with the new battery voltage
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(wristSim.getCurrentDrawAmps()));
    }
}