package frc.robot.constants;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Robot;

public class ElevatorConstants {
    // Motor IDs
    public static final int leftMotorId = Robot.isSimulation()?20:0;
    public static final int rightMotorId = Robot.isSimulation()?21:1;

    // Encoder ID
    public static final int encoderId = 2;
    public static final double encoderRatio = 0.5;
    public static final double encoderOffset = -0.273;

    // PID constants
    public static final double kP = 60;
    public static final double kI = 0;
    public static final double kD = 0;

    // Feedforward constants
    public static final double kS = Robot.isSimulation() ? 0.0 : 0.0;
    public static final double kG = 0.18;
    public static final double kV = 0.88;
    public static final double kA = 0.02;

    // Elevator tolerance
    public static final double elevatorTolerance = 0.01;

    // Simulation constants
    public static final double gearRatio = 30.0; // gear ratio
    public static final double carriageMass = 13.0; // in kg
    public static final double drumRadius = 0.0254; // in meters
    public static final double minHeight = 0; // in meters
    public static final double maxHeight = 1.3; // in meters // actually 1.4
    public static final DCMotor motorSim = DCMotor.getFalcon500(2);

    // Motion Profile constants
    public static final double maxVelocity = 1; // meters/sec
    public static final double maxAcceleration = 2; // meters/sec^2
}
