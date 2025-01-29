package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.GeneralConstants;

/**
 * The Elevator subsystem controls the elevator mechanism of the robot.
 * It uses two TalonFX motors for movement, a PID controller for position control,
 * and a feedforward controller for compensating for gravity and friction.
 * The subsystem also supports simulation for testing and tuning.
 */
public class Elevator extends SubsystemBase {

    // Define the motors for the elevator
    private TalonFX leftMotor = new TalonFX(ElevatorConstants.leftMotorId, "rhino");
    private TalonFX rightMotor = new TalonFX(ElevatorConstants.rightMotorId, "rhino");

    // PID controller and feedforward controller for elevator control
    private PIDController controller = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);
    private ElevatorFeedforward feedforward = new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV);

    // Simulation objects for the elevator
    private ElevatorSim elevatorSim = new ElevatorSim(ElevatorConstants.motorSim, ElevatorConstants.gearRatio, ElevatorConstants.carriageMass, ElevatorConstants.drumRadius, ElevatorConstants.minHeight, ElevatorConstants.maxHeight, true, ElevatorConstants.defaultGoal);
    private TalonFXSimState leftMotorSim = leftMotor.getSimState();
    private TalonFXSimState rightMotorSim = rightMotor.getSimState();

    // Goal position for the elevator
    private double goal = ElevatorConstants.defaultGoal;

    // SysId routine for system identification
    public final SysIdRoutine sysIdRoutine = 
        new SysIdRoutine(
            // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                // Tell SysId how to plumb the driving voltage to the motor(s).
                this::setVoltage,
                // Tell SysId how to record a frame of data for each motor on the mechanism being characterized.
                log -> {
                    // Record a frame for the elevator motor.
                    log.motor("elevator")
                        .voltage(Volts.of(getInput() * RobotController.getBatteryVoltage()))
                        .linearPosition(Meters.of(getMeasurement()))
                        .linearVelocity(MetersPerSecond.of(getVelocity()));
                },
                // Tell SysId to make generated commands require this subsystem, suffix test state in WPILog with this subsystem's name ("elevator")
                this));

    private StringLogEntry log;

    /**
     * Constructor for the Elevator subsystem.
     */
    public Elevator(StringLogEntry log) {
        // Configure the motors to coast when neutral
        var currentConfigs = new MotorOutputConfigs();
        currentConfigs.NeutralMode = NeutralModeValue.Coast;
        leftMotor.getConfigurator().apply(currentConfigs);
        rightMotor.getConfigurator().apply(currentConfigs);

        // Set the tolerance for the PID controller
        controller.setTolerance(ElevatorConstants.elevatorTol);

        // Add the PID controller to SmartDashboard for tuning
        SmartDashboard.putData("Elevator Controller", controller);

        this.log=log;
    }

    /**
     * Gets the current goal position of the elevator.
     * 
     * @return The current goal position in meters.
     */
    public double getGoal() {
        return goal;
    }

    /**
     * Sets the goal position for the elevator.
     * The goal is clamped between the minimum and maximum height limits.
     * 
     * @param newGoal The new goal position in meters.
     */
    public void setGoal(double newGoal) {
        if(newGoal < ElevatorConstants.minHeight){
            newGoal = ElevatorConstants.minHeight;
        }
        if(newGoal > ElevatorConstants.maxHeight){
            newGoal = ElevatorConstants.maxHeight;
        }
        goal = newGoal;
    }

    /**
     * Gets the current position of the elevator.
     * 
     * @return The current position in meters.
     */
    public double getMeasurement() {
        return (leftMotor.getPosition().getValueAsDouble() - rightMotor.getPosition().getValueAsDouble()) / 2 * ElevatorConstants.metersPerRotation - ElevatorConstants.elevatorOffset;
    }

    /**
     * Gets the current velocity of the elevator.
     * 
     * @return The current velocity in meters per second.
     */
    public double getVelocity() {
        return (leftMotor.getVelocity().getValueAsDouble() - rightMotor.getVelocity().getValueAsDouble()) / 2 * ElevatorConstants.metersPerRotation;
    }

    /**
     * Checks if the elevator is at the setpoint.
     * 
     * @return True if the elevator is at the setpoint, false otherwise.
     */
    public boolean atSetpoint() {
        return controller.atSetpoint();
    }

    /**
     * Gets the current input to the elevator motors.
     * 
     * @return The average input to the motors.
     */
    public double getInput() {
        return (leftMotor.get() - rightMotor.get()) / 2;
    }

    /**
     * Sets the voltage to the elevator motors.
     * 
     * @param v The voltage to apply to the motors.
     */
    public void setVoltage(Voltage v) {
        leftMotor.setVoltage(v.magnitude());
        rightMotor.setVoltage(-v.magnitude());
    }

    /**
     * Updates telemetry data on SmartDashboard.
     * This includes the elevator height, velocity, and input.
     */
    private void telemetry() {
        SmartDashboard.putNumber("Elevator Height", getMeasurement());
        SmartDashboard.putNumber("Elevator Velocity", getVelocity());
        SmartDashboard.putNumber("Elevator Input", getInput());
    }

    /**
     * Periodic method called every robot loop.
     * Updates the elevator control and telemetry.
     */
    @Override
    public void periodic() {
        telemetry();
        // Calculate the feedforward and PID output
        double extra = feedforward.calculate(getVelocity());
        double voltage = controller.calculate(getMeasurement(), goal) + extra;
        setVoltage(Volts.of(voltage));
    }

    /**
     * Simulation periodic method called every simulation loop.
     * Updates the motor simulation states and the RoboRIO simulation.
     */
    @Override
    public void simulationPeriodic() {
        // Update the motor simulation states with the current battery voltage
        leftMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        rightMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        // Calculate the input voltage to the elevator simulation
        double elevatorInput = (leftMotorSim.getMotorVoltage() - rightMotorSim.getMotorVoltage()) / 2;
        elevatorSim.setInputVoltage(elevatorInput);
        elevatorSim.update(GeneralConstants.simPeriod);

        // Update the motor positions and velocities based on the simulation
        double pos = elevatorSim.getPositionMeters();
        leftMotorSim.setRawRotorPosition((pos + ElevatorConstants.elevatorOffset) / ElevatorConstants.metersPerRotation);
        rightMotorSim.setRawRotorPosition(-(pos + ElevatorConstants.elevatorOffset) / ElevatorConstants.metersPerRotation);
        double vel = elevatorSim.getVelocityMetersPerSecond();
        leftMotorSim.setRotorVelocity(vel / ElevatorConstants.metersPerRotation);
        rightMotorSim.setRotorVelocity(-vel / ElevatorConstants.metersPerRotation);

        // Update the RoboRIO simulation with the current battery voltage
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));
    }

    /**
     * Logs elevator data.
     */
    public void log(){
        log.append("Height: "+getMeasurement());
        log.append("Velocity: "+getVelocity());
        log.append("Goal: "+getGoal());
        log.append("Input: "+getInput());
        log.append("Error: "+controller.getError());
        log.append("Left Motor Temp: "+leftMotor.getDeviceTemp().getValueAsDouble());
        log.append("Right Motor Temp: "+rightMotor.getDeviceTemp().getValueAsDouble());
        log.append("Left Motor Voltage: "+leftMotor.getMotorVoltage().getValueAsDouble());
        log.append("Right Motor Voltage: "+rightMotor.getMotorVoltage().getValueAsDouble());
        // Fault flag = 0 means nothing bad happened, fault flag > 0 means something bad happened
        int leftMotorFault=leftMotor.getFaultField().asSupplier().get();
        if(leftMotorFault!=0){
            log.append("Left Motor Error: "+leftMotorFault);
        }
        int rightMotorFault=leftMotor.getFaultField().asSupplier().get();
        if(rightMotorFault!=0){
            log.append("Right Motor Error: "+rightMotorFault);
        }
        log.append("Left Motor Supply Current: "+leftMotor.getSupplyCurrent());
        log.append("Right Motor Supply Current: "+rightMotor.getSupplyCurrent());
        log.append("Left Motor Supply Voltage: "+leftMotor.getSupplyVoltage());
        log.append("Right Motor Supply Voltage: "+rightMotor.getSupplyVoltage());
    }
}