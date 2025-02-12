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
        0.26,                       // From ElevatorConstants.groundIntakeGoal
        wristDown,                  // From WristConstants.pos1 (down)
        null                        // Robot position determined at runtime
    );

    private static final SwerveSystem.SwerveSystemState baseDefaultState = new SwerveSystem.SwerveSystemState(
        Units.degreesToRadians(0),  // From ArmConstants.defaultGoal (vertical)
        0.1,                         // From ElevatorConstants.defaultGoal
        wristStraight,                   // From WristConstants.pos1 (down)
        null                         // Robot position determined at runtime
    );

    // Base states (without alliance inversion)
    private static final SwerveSystem.SwerveSystemState baseProcessor;
    private static final SwerveSystem.SwerveSystemState[] baseSourceStates;
    private static final SwerveSystem.SwerveSystemState[] baseAlgaeStates;
    private static final SwerveSystem.SwerveSystemState[][] baseScoringStates;

    // Offset for the arm pivot in AdvantageScope simulation
    public static final Translation3d armTransOffset = new Translation3d(0.11,-0.18,0.26);

    static {
        // Initialize processor state
        baseProcessor = new SwerveSystem.SwerveSystemState(
            Units.degreesToRadians(-95),  // 75 - 90 = -15 degrees
            0.26,
            wristDown,
            GeneralConstants.getProcessor()
        );

        // Initialize source states
        baseSourceStates = new SwerveSystem.SwerveSystemState[GeneralConstants.sides * 2 / 3];
        for (int i = 0; i < baseSourceStates.length; i++) {
            baseSourceStates[i] = new SwerveSystem.SwerveSystemState(
                Units.degreesToRadians(35),  // 45 - 90 = -45 degrees
                0.22,                        // From ElevatorConstants.sourceGoal
                wristStraight,              // From WristConstants.pos2 (straight)
                GeneralConstants.getStations()[i]
            );
        }

        // Initialize algae states
        baseAlgaeStates = new SwerveSystem.SwerveSystemState[GeneralConstants.sides];
        for (int i = 0; i < baseAlgaeStates.length; i++) {
            boolean isLow = Reef.isAlgaeLow(i);
            baseAlgaeStates[i] = new SwerveSystem.SwerveSystemState(
                Units.degreesToRadians(isLow ? -60 : 30),  // 30 - 90 = -60, 120 - 90 = 30
                0.35,                                       // From ElevatorConstants.algaeGoal
                wristStraight,                             // From WristConstants.pos2 (straight)
                GeneralConstants.getCenterReef()[i]
            );
        }

        // Initialize scoring states
        baseScoringStates = new SwerveSystem.SwerveSystemState[4][GeneralConstants.sides*2];
        double[] armAngles = {
            Units.degreesToRadians(45),   // L1 (135 - 90)
            Units.degreesToRadians(45),   // L2 (135 - 90)
            Units.degreesToRadians(45),   // L3 (135 - 90)
            Units.degreesToRadians(30)    // L4 (120 - 90)
        };
        double[] elevatorHeights = {
            0.06,  // L1
            0.26,  // L2
            0.56,  // L3
            1.26   // L4
        };

        for (int level = 0; level < baseScoringStates.length; level++) {
            for (int pos = 0; pos < baseScoringStates[level].length; pos++) {
                baseScoringStates[level][pos] = new SwerveSystem.SwerveSystemState(
                    armAngles[level],
                    elevatorHeights[level],
                    wristStraight,
                    GeneralConstants.getBranchReef()[pos]
                );
            }
        }
    }

    public static SwerveSystem.SwerveSystemState getGroundIntake() {
        return baseGroundIntake;
    }

    public static SwerveSystem.SwerveSystemState getDefaultState() {
        return baseDefaultState;
    }

    public static SwerveSystem.SwerveSystemState getProcessor() {
        return new SwerveSystem.SwerveSystemState(
            baseProcessor.armPosition,
            baseProcessor.elevatorPosition,
            baseProcessor.wristPosition,
            GeneralConstants.getProcessor()
        );
    }

    public static SwerveSystem.SwerveSystemState[] getSourceStates() {
        SwerveSystem.SwerveSystemState[] states = new SwerveSystem.SwerveSystemState[baseSourceStates.length];
        var stations = GeneralConstants.getStations();
        for (int i = 0; i < states.length; i++) {
            states[i] = new SwerveSystem.SwerveSystemState(
                baseSourceStates[i].armPosition,
                baseSourceStates[i].elevatorPosition,
                baseSourceStates[i].wristPosition,
                stations[i]
            );
        }
        return states;
    }

    public static SwerveSystem.SwerveSystemState[] getAlgaeStates() {
        SwerveSystem.SwerveSystemState[] states = new SwerveSystem.SwerveSystemState[baseAlgaeStates.length];
        var centerReef = GeneralConstants.getCenterReef();
        for (int i = 0; i < states.length; i++) {
            states[i] = new SwerveSystem.SwerveSystemState(
                baseAlgaeStates[i].armPosition,
                baseAlgaeStates[i].elevatorPosition,
                baseAlgaeStates[i].wristPosition,
                centerReef[i]
            );
        }
        return states;
    }

    public static SwerveSystem.SwerveSystemState[][] getScoringStates() {
        SwerveSystem.SwerveSystemState[][] states = new SwerveSystem.SwerveSystemState[baseScoringStates.length][baseScoringStates[0].length];
        var branchReef = GeneralConstants.getBranchReef();
        for (int level = 0; level < states.length; level++) {
            for (int pos = 0; pos < states[level].length; pos++) {
                states[level][pos] = new SwerveSystem.SwerveSystemState(
                    baseScoringStates[level][pos].armPosition,
                    baseScoringStates[level][pos].elevatorPosition,
                    baseScoringStates[level][pos].wristPosition,
                    branchReef[pos]
                );
            }
        }
        return states;
    }
} 