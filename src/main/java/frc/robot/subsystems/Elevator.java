package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
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
import frc.robot.other.RobotUtils;
import frc.robot.constants.ArmSystemConstants;

/**
 * The Elevator subsystem controls the elevator mechanism of the robot.
 * It uses two TalonFX motors for movement, a PID controller for position control,
 * and a feedforward controller for compensating for gravity and friction.
 * The subsystem also supports simulation for testing and tuning.
 */
public class Elevator extends SubsystemBase {

    // Define the motors for the elevator
    private TalonFX leftMotor = new TalonFX(ElevatorConstants.leftMotorId, "rio");
    private TalonFX rightMotor = new TalonFX(ElevatorConstants.rightMotorId, "rio");

    // PID controller and feedforward controller for elevator control
    private PIDController controller = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);
    private ElevatorFeedforward feedforward = new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV, ElevatorConstants.kA);

    // Simulation objects for the elevator
    private ElevatorSim elevatorSim = new ElevatorSim(
        ElevatorConstants.motorSim,
        ElevatorConstants.gearRatio,
        ElevatorConstants.carriageMass,
        ElevatorConstants.drumRadius,
        ElevatorConstants.minHeight,
        ElevatorConstants.maxHeight,
        true,
        ArmSystemConstants.defaultState.elevatorPosition
    );
    private TalonFXSimState leftMotorSim = new TalonFXSimState(leftMotor, ChassisReference.CounterClockwise_Positive);
    private TalonFXSimState rightMotorSim = new TalonFXSimState(rightMotor, ChassisReference.Clockwise_Positive);

    // Goal position for the elevator
    private double goal = ArmSystemConstants.defaultState.elevatorPosition;

    // SysId routine for system identification
    private final SysIdRoutine sysIdRoutine = 
        new SysIdRoutine(
            // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
            new SysIdRoutine.Config(
                null,        // Use default ramp rate (1 V/s)
                Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
                null        // Use default timeout (10 s)
            ),
            new SysIdRoutine.Mechanism(
                // Tell SysId how to plumb the driving voltage to the motor(s).
                this::setVoltage,
                // Tell SysId how to record a frame of data for each motor on the mechanism being characterized.
                log -> {
                    // Record a frame for the elevator motor.
                    log.motor("elevator")
                        .voltage(getVolts())
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
        currentConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
        leftMotor.getConfigurator().apply(currentConfigs);
        currentConfigs.Inverted = InvertedValue.Clockwise_Positive;
        rightMotor.getConfigurator().apply(currentConfigs);

        // Set the tolerance for the PID controller
        controller.setTolerance(ElevatorConstants.elevatorTolerance);

        // Add the PID controller to SmartDashboard for tuning
        SmartDashboard.putData("elevator/controller", controller);

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
        goal = RobotUtils.clamp(newGoal,ElevatorConstants.minHeight,ElevatorConstants.maxHeight);
    }

    /**
     * Gets the current position of the elevator.
     * 
     * @return The current position in meters.
     */
    public double getMeasurement() {
        return ElevatorConstants.transform.transformPos((leftMotor.getPosition().getValueAsDouble() + rightMotor.getPosition().getValueAsDouble()) / 2);
    }

    /**
     * Gets the current velocity of the elevator.
     * 
     * @return The current velocity in meters per second.
     */
    public double getVelocity() {
        return ElevatorConstants.transform.transformVel((leftMotor.getVelocity().getValueAsDouble() + rightMotor.getVelocity().getValueAsDouble()) / 2);
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
     * Gets the current voltage applied to the elevator motors.
     * 
     * @return The average voltage applied to the motors.
     */
    public Voltage getVolts() {
        return (leftMotor.getMotorVoltage().getValue().plus(rightMotor.getMotorVoltage().getValue())).div(2.0);
    }

    /**
     * Sets the voltage to the elevator motors.
     * 
     * @param v The voltage to apply to the motors.
     */
    public void setVoltage(Voltage v) {
        leftMotor.setVoltage(v.magnitude());
        rightMotor.setVoltage(v.magnitude());
    }

    public SysIdRoutine getRoutine(){
        return sysIdRoutine;
    }

    /**
     * Updates telemetry data on SmartDashboard.
     * This includes the elevator height, velocity, and input.
     */
    private void telemetry() {
        SmartDashboard.putNumber("elevator/height",getMeasurement());
        SmartDashboard.putNumber("elevator/goal",getGoal());
        SmartDashboard.putNumber("elevator/velocity",getVelocity());
        SmartDashboard.putNumber("elevator/error",controller.getError());
        SmartDashboard.putNumber("elevator/leftMotor/rawHeight",leftMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("elevator/rightMotor/rawHeight",rightMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("elevator/leftMotor/temp",leftMotor.getDeviceTemp().getValueAsDouble());
        SmartDashboard.putNumber("elevator/rightMotor/temp",rightMotor.getDeviceTemp().getValueAsDouble());
        SmartDashboard.putNumber("elevator/leftMotor/fault",leftMotor.getFaultField().asSupplier().get());
        SmartDashboard.putNumber("elevator/rightMotor/fault",rightMotor.getFaultField().asSupplier().get());
        SmartDashboard.putNumber("elevator/leftMotor/current",leftMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("elevator/rightMotor/current",rightMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("elevator/leftMotor/voltage",leftMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("elevator/rightMotor/voltage",rightMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("elevator/leftMotor/supplyVoltage",leftMotor.getSupplyVoltage().getValueAsDouble());
        SmartDashboard.putNumber("elevator/rightMotor/supplyVoltage",rightMotor.getSupplyVoltage().getValueAsDouble());
    }

    /**
     * Periodic method called every robot loop.
     * Updates the elevator control and telemetry.
     */
    @Override
    public void periodic() {
        try{
            telemetry();
            // Calculate the feedforward and PID output
            double feed = feedforward.calculate(getVelocity());
            double voltage = controller.calculate(getMeasurement(), goal) + feed;
            setVoltage(Volts.of(voltage));
        }catch(Exception e){
            log.append("Periodic error: "+RobotUtils.getError(e));
        }
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
        double elevatorInput = (leftMotorSim.getMotorVoltage() + rightMotorSim.getMotorVoltage()) / 2;
        elevatorSim.setInputVoltage(elevatorInput);
        elevatorSim.update(GeneralConstants.simPeriod);

        // Update the motor positions and velocities based on the simulation
        double pos = ElevatorConstants.transform.transformPosInv(elevatorSim.getPositionMeters());
        leftMotorSim.setRawRotorPosition(pos);
        rightMotorSim.setRawRotorPosition(pos);
        double vel = ElevatorConstants.transform.transformPosInv(elevatorSim.getVelocityMetersPerSecond());
        leftMotorSim.setRotorVelocity(vel);
        rightMotorSim.setRotorVelocity(vel);

        // Update the RoboRIO simulation with the current battery voltage
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));
    }
}