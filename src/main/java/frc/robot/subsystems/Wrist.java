package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.WristConstants;
import frc.robot.other.RobotUtils;
import frc.robot.constants.GeneralConstants;
import frc.robot.constants.ArmSystemConstants;

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
    private TalonFX wristMotor = new TalonFX(WristConstants.motorId, "rio");
    
    // PID controller for Wrist position control
    private ProfiledPIDController controller = new ProfiledPIDController(WristConstants.kP, WristConstants.kI, WristConstants.kD, new Constraints(WristConstants.maxVelocity, WristConstants.maxAcceleration));

    // Feedforward controller for arm dynamics
    private ArmFeedforward feedforward = new ArmFeedforward(WristConstants.kS, WristConstants.kG, WristConstants.kV);
    
    // Goal position for the Wrist in radians
    private double goal = ArmSystemConstants.defaultState.wristPosition;
    
    // Simulation model for the Wrist
    private SingleJointedArmSim wristSim = new SingleJointedArmSim(
        WristConstants.motorSim,
        WristConstants.gearRatio,
        WristConstants.momentOfInertia,
        WristConstants.wristLength,
        WristConstants.minAngle,
        WristConstants.maxAngle,
        true,
        ArmSystemConstants.defaultState.wristPosition
    );

    // SysId routine for system identification
    private final SysIdRoutine sysIdRoutine = 
        new SysIdRoutine(
            // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
            new SysIdRoutine.Config(
                Volts.of(0.2).div(Seconds.of(1.0)),        // Use default ramp rate (1 V/s)
                Volts.of(1), // Reduce dynamic step voltage to 4 V to prevent brownout
                null        // Use default timeout (10 s)
            ),
            new SysIdRoutine.Mechanism(
                // Tell SysId how to plumb the driving voltage to the motor(s).
                this::setVoltage,
                // Tell SysId how to record a frame of data for each motor on the mechanism being characterized.
                log -> {
                    // Record a frame for the elevator motor.
                    log.motor("wrist")
                        .voltage(getVolts())
                        .angularPosition(Radians.of(getMeasurement()))
                        .angularVelocity(RadiansPerSecond.of(getVelocity()))
                        .angularAcceleration(RadiansPerSecondPerSecond.of(getAcceleration()));
                },
                // Tell SysId to make generated commands require this subsystem, suffix test state in WPILog with this subsystem's name ("elevator")
                this));
    
    // Simulation state for the motor
    private TalonFXSimState wristMotorSim = wristMotor.getSimState();

    private Arm arm;

    private StringLogEntry log;

    // Used for calculating acceleration
    private double lastSpeed;
    private double lastTime;
        
    /**
     * Constructor for the Wrist subsystem.
     */
    public Wrist(Arm arm, StringLogEntry log) {
        // Configure the motor to brake when neutral
        var currentConfigs = new MotorOutputConfigs();
        currentConfigs.NeutralMode = NeutralModeValue.Brake;
        wristMotor.getConfigurator().apply(currentConfigs);

        // Set the tolerance for the PID controller
        controller.setTolerance(WristConstants.wristTolerance);
        
        // Display the PID controller on SmartDashboard for tuning
        SmartDashboard.putData("wrist/controller", controller);

        this.arm=arm;
        this.log=log;

        lastSpeed = 0.0;
        lastTime = 0.0;

        //wristMotor.setPosition(0.0);
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
     * Gets the true Wrist position in radians to use with feedforward.
     * 
     * @return The current true position of the wrist in radians.
     */
    public double getAdjustedMeasurement() {
        return arm.getMeasurement() + this.getMeasurement() - WristConstants.feedOffset;
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
     * Gets the true goal in radians to use with feedforward.
     * 
     * @return The current true goal in radians.
     */
    public double getAdjustedGoal() {
        return arm.getGoal() + this.getGoal();
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

    /**
     * Gets the current voltage applied to the arm motor.
     * 
     * @return The voltage applied to the motors.
     */
    public Voltage getVolts() {
        return (wristMotor.getMotorVoltage().getValue());
    }

    /**
     * Gets the current Wrist acceleration in radians per second^2.
     * 
     * @return The current acceleration of the wrist in radians per second^2.
     */
    public double getAcceleration() {
        return WristConstants.transform.transformVel(wristMotor.getAcceleration().getValueAsDouble());
    }

    public boolean atSetpoint(){
        return controller.atSetpoint();
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
        SmartDashboard.putNumber("wrist/positionError",controller.getPositionError());
        SmartDashboard.putNumber("wrist/velocityError",controller.getVelocityError());
        SmartDashboard.putNumber("wrist/motor/rawAngle",wristMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("wrist/motor/temp",wristMotor.getDeviceTemp().getValueAsDouble());
        SmartDashboard.putNumber("wrist/motor/fault",wristMotor.getFaultField().asSupplier().get());
        SmartDashboard.putNumber("wrist/motor/current",wristMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("wrist/motor/voltage",wristMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("wrist/motor/supplyVoltage",wristMotor.getSupplyVoltage().getValueAsDouble());
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
            double time = Timer.getFPGATimestamp();
            State state = controller.getSetpoint();
            double feed = feedforward.calculate(state.position, state.velocity, (state.velocity-lastSpeed)/(time-lastTime));
            double voltage = controller.calculate(getMeasurement(), goal) + feed;
            
            // Apply the calculated voltage to the motor
            setVoltage(Volts.of(voltage));

            // Keep track of previous values to calculate acceleration
            lastSpeed = state.velocity;
            lastSpeed = time;
            
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
        double wristInput = wristMotorSim.getMotorVoltage();
        wristSim.setInputVoltage(wristInput);
        wristSim.update(GeneralConstants.simPeriod);
        
        // Update the motor simulation state with the new Wrist position and velocity
        wristMotorSim.setRawRotorPosition(WristConstants.transform.transformPosInv(wristSim.getAngleRads()));
        wristMotorSim.setRotorVelocity(WristConstants.transform.transformVelInv(wristSim.getVelocityRadPerSec()));
        
        // Update the RoboRIO simulation state with the new battery voltage
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(wristSim.getCurrentDrawAmps()));
    }
}