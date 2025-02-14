package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.GeneralConstants;
import frc.robot.other.RobotUtils;
import frc.robot.other.RolloverEncoder;
import frc.robot.other.TalonFX2;
import frc.robot.constants.SwerveSystemConstants;


/**
 * The Elevator subsystem controls the elevator mechanism of the robot.
 * It uses two TalonFX motors for movement, a PID controller for position control,
 * and a feedforward controller for compensating for gravity and friction.
 * The subsystem also supports simulation for testing and tuning.
 */
public class Elevator extends SubsystemBase {

    // Define the motors for the elevator
    private TalonFX2 leftMotor = new TalonFX2(ElevatorConstants.leftMotorId,ElevatorConstants.drumRadius*2*Math.PI/ElevatorConstants.gearRatio,0,InvertedValue.CounterClockwise_Positive,"rio");
    private TalonFX2 rightMotor = new TalonFX2(ElevatorConstants.rightMotorId,ElevatorConstants.drumRadius*2*Math.PI/ElevatorConstants.gearRatio,0,InvertedValue.Clockwise_Positive,"rio");

    private RolloverEncoder elevatorEncoder = new RolloverEncoder(ElevatorConstants.encoderId, ElevatorConstants.drumRadius*2*Math.PI/ElevatorConstants.encoderRatio, ElevatorConstants.encoderOffset);

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
        SwerveSystemConstants.getDefaultState().elevatorPosition
    );

    // Goal position for the elevator
    private double goal = SwerveSystemConstants.getDefaultState().elevatorPosition;

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

    /**
     * Constructor for the Elevator subsystem.
     */
    public Elevator() {
        // Set the tolerance for the PID controller
        controller.setTolerance(ElevatorConstants.elevatorTolerance);

        // Add the PID controller to SmartDashboard for tuning
        SmartDashboard.putData("elevator/controller", controller);
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
        return elevatorEncoder.get();
    }

    /**
     * Gets the current velocity of the elevator.
     * 
     * @return The current velocity in meters per second.
     */
    public double getVelocity() {
        return (leftMotor.getVel()+rightMotor.getVel())/2;
    }

    /**
     * Gets the current acceleration of the elevator.
     * 
     * @return The current acceleration in meters per second^2.
     */
    public double getAcceleration() {
        return (leftMotor.getAcc()+rightMotor.getAcc())/2;
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
        SmartDashboard.putNumber("elevator/positionError",controller.getError());
        SmartDashboard.putNumber("elevator/velocityError",controller.getErrorDerivative());
        SmartDashboard.putNumber("elevator/leftMotor/output",leftMotor.get());
        SmartDashboard.putNumber("elevator/rightMotor/output",rightMotor.get());
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

            double feed = feedforward.calculate(0);
            double voltage = controller.calculate(getMeasurement(), goal) + feed;
            
            // Apply the calculated voltage to the motor
            setVoltage(Volts.of(voltage));

            elevatorEncoder.periodic();
        }catch(Exception e){
            DataLogManager.log("Periodic error: "+RobotUtils.getError(e));
        }
    }

    /**
     * Simulation periodic method called every simulation loop.
     * Updates the motor simulation states and the RoboRIO simulation.
     */
    @Override
    public void simulationPeriodic() {
        // Update the motor simulation state with the current battery voltage
        leftMotor.refreshSupplyVoltage();
        rightMotor.refreshSupplyVoltage();
        
        // Get the current motor input voltage and update the Wrist simulation
        double elevatorInput = (leftMotor.getSimVoltage()+rightMotor.getSimVoltage())/2;
        elevatorSim.setInputVoltage(elevatorInput);
        elevatorSim.update(GeneralConstants.simPeriod);
        
        // Update the motor simulation state with the new Wrist position and velocity
        leftMotor.setPos(elevatorSim.getPositionMeters());
        rightMotor.setPos(elevatorSim.getPositionMeters());
        leftMotor.setVel(elevatorSim.getVelocityMetersPerSecond());
        rightMotor.setVel(elevatorSim.getVelocityMetersPerSecond());

        elevatorEncoder.set(elevatorSim.getPositionMeters());

        // Update the RoboRIO simulation state with the new battery voltage
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));
    }
}