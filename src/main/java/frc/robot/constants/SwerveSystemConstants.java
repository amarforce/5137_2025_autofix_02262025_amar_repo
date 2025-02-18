package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.SwerveSystem;
import frc.robot.elastic.Reef;

/**
 * Constants for coordinated positions of the arm, elevator, and wrist mechanisms.
 * This class centralizes all the goal positions for the entire arm system.
 */
public final class SwerveSystemConstants {
    public static final double timeout = 5; // seconds
    
    // Weight for how much to consider rotation vs translation when finding closest state
    // Higher values mean rotation differences matter more
    public static final double rotationWeight = 1.0; // meters per radian

    private static final double wristStraight = Units.degreesToRadians(0);    // 90 - 90 = 0 (vertical)
    private static final double wristDown = Units.degreesToRadians(-90);      // 0 - 90 = -90 (horizontal)

    // Basic states without specific robot positions
    private static final SwerveSystem.SwerveSystemState baseGroundIntake = new SwerveSystem.SwerveSystemState(
        Units.degreesToRadians(-120),  // From ArmConstants.groundIntakeGoal
        0.5,                       // From ElevatorConstants.groundIntakeGoal
        Units.degreesToRadians(-45),                  // From WristConstants.pos1 (down)
        null                        // Robot position determined at runtime
    );

    private static final SwerveSystem.SwerveSystemState baseDefaultState = new SwerveSystem.SwerveSystemState(
        Units.degreesToRadians(0),  // From ArmConstants.defaultGoal (vertical)
        0.05,                         // From ElevatorConstants.defaultGoal
        wristStraight,                   // From WristConstants.pos1 (down)
        null                         // Robot position determined at runtime
    );

    // Offset for the arm pivot in AdvantageScope simulation
    public static final Translation3d armTransOffset = new Translation3d(0.11,-0.18,0.26);

    public static final double intakeDistance = 1;

    public static SwerveSystem.SwerveSystemState getGroundIntake() {
        return baseGroundIntake;
    }

    public static SwerveSystem.SwerveSystemState getDefaultState() {
        return baseDefaultState;
    }

    public static SwerveSystem.SwerveSystemState getProcessor() {
        return new SwerveSystem.SwerveSystemState(
            Units.degreesToRadians(-95),  // 75 - 90 = -15 degrees
            0.4,
            wristDown,
            RobotPositions.processor.alliancePos()
        );
    }

    public static SwerveSystem.SwerveSystemState[] getSourceStates() {
        SwerveSystem.SwerveSystemState[] states = new SwerveSystem.SwerveSystemState[RobotPositions.stations.length];
        for (int i = 0; i < states.length; i++) {
            states[i] = new SwerveSystem.SwerveSystemState(
                Units.degreesToRadians(-25.1),  // 45 - 90 = -45 degrees
                0.1,                        // From ElevatorConstants.sourceGoal
                Units.degreesToRadians(-113.6),              // From WristConstants.pos2 (straight)
                RobotPositions.stations[i].alliancePos()
            );
        }
        return states;
    }

    public static SwerveSystem.SwerveSystemState[] getAlgaeStates() {
        SwerveSystem.SwerveSystemState[] states = new SwerveSystem.SwerveSystemState[FieldGeometry.reefSides];
        for (int i = 0; i < states.length; i++) {
            boolean isLow = Reef.isAlgaeLow(i);
            states[i] = new SwerveSystem.SwerveSystemState(
                Units.degreesToRadians(30),  // 30 - 90 = -60, 120 - 90 = 30
                0.43,                                       // From ElevatorConstants.algaeGoal
                wristStraight,                             // From WristConstants.pos2 (straight)
                RobotPositions.centerReef[i].alliancePos()
            );
        }
        return states;
    }

    public static SwerveSystem.SwerveSystemState[][] getScoringStates() {
        SwerveSystem.SwerveSystemState[][] states = new SwerveSystem.SwerveSystemState[4][FieldGeometry.reefSides*2];
        double[] armAngles = {
            Units.degreesToRadians(45),   // L1 (135 - 90)
            Units.degreesToRadians(35),   // L2 (135 - 90)
            Units.degreesToRadians(35),   // L3 (135 - 90)
            Units.degreesToRadians(35)    // L4 (120 - 90)
        };
        double[] elevatorHeights = {
            0.06,  // L1
            0.0803,  // L2
            0.41,  // L3
            1.21   // L4
        };
        double[] wristAngles = {
            Units.degreesToRadians(0),   // L1 (135 - 90)
            Units.degreesToRadians(0),   // L2 (135 - 90)
            Units.degreesToRadians(0),   // L3 (135 - 90)
            Units.degreesToRadians(24.2)    // L4 (120 - 90)
        };

        for (int level = 0; level < states.length; level++) {
            for (int pos = 0; pos < states[level].length; pos++) {
                states[level][pos] = new SwerveSystem.SwerveSystemState(
                    armAngles[level],
                    elevatorHeights[level],
                    wristAngles[level],
                    RobotPositions.branchReef[pos].alliancePos()
                );
            }
        }
        return states;
    }
} 