package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.SwerveSystem;

/**
 * Constants for coordinated positions of the arm, elevator, and wrist mechanisms.
 * This class centralizes all the goal positions for the entire arm system.
 */
public final class SwerveSystemConstants {
    public static final double timeout = 5; // seconds
    
    // Weight for how much to consider rotation vs translation when finding closest state
    // Higher values mean rotation differences matter more
    public static final double rotationWeight = 1.0; // meters per radian

    private static final double wristStraight = Units.degreesToRadians(90);
    private static final double wristDown = Units.degreesToRadians(0);

    // Basic states without specific robot positions
    public static SwerveSystem.SwerveSystemState groundIntake(){
        return new SwerveSystem.SwerveSystemState(
            Units.degreesToRadians(0),  // From ArmConstants.groundIntakeGoal
            0.26,                       // From ElevatorConstants.groundIntakeGoal
            wristDown,                  // From WristConstants.pos1 (down)
            null
        );
    }

    public static SwerveSystem.SwerveSystemState defaultState(){
        return new SwerveSystem.SwerveSystemState(
            Units.degreesToRadians(90),  // From ArmConstants.defaultGoal
            0.0,                         // From ElevatorConstants.defaultGoal
            wristDown,                   // From WristConstants.pos1 (down)
            null                         // Robot position determined at runtime
        );
    }

    public static SwerveSystem.SwerveSystemState processor(){
        return new SwerveSystem.SwerveSystemState(
            Units.degreesToRadians(75),
            1.26,
            wristStraight,
            GeneralConstants.processor()    // Processor position
        );
    }

    // Source states - one for each station position
    public static SwerveSystem.SwerveSystemState[] sourceStates(){
        Pose2d[] stations=GeneralConstants.stations();
        SwerveSystem.SwerveSystemState[] sourceStates=new SwerveSystem.SwerveSystemState[stations.length];
        for (int i = 0; i < stations.length; i++) {
            sourceStates[i] = new SwerveSystem.SwerveSystemState(
                Units.degreesToRadians(45),  // From ArmConstants.sourceGoal
                0.76,                        // From ElevatorConstants.sourceGoal
                wristStraight,              // From WristConstants.pos2 (straight)
                stations[i] // Station position
            );
        }
        return sourceStates;
    }

    // Algae states - one for each reef position, alternating between low and high
    public static SwerveSystem.SwerveSystemState[] algaeStates(){
        Pose2d[] algae=GeneralConstants.centerReef();
        SwerveSystem.SwerveSystemState[] algaeStates=new SwerveSystem.SwerveSystemState[algae.length];
        for (int i = 0; i < algae.length; i++) {
            algaeStates[i] = new SwerveSystem.SwerveSystemState(
                Units.degreesToRadians(45),  // From ArmConstants.sourceGoal
                0.76,                        // From ElevatorConstants.sourceGoal
                wristStraight,              // From WristConstants.pos2 (straight)
                algae[i] // Station position
            );
        }
        return algaeStates;
    }

    // Algae states - one for each reef position, alternating between low and high
    public static SwerveSystem.SwerveSystemState[][] scoringStates(){
        SwerveSystem.SwerveSystemState[][] scoringStates = new SwerveSystem.SwerveSystemState[4][GeneralConstants.sides * 2];
        // Initialize scoring states
        double[] armAngles = {
            Units.degreesToRadians(135),  // L1
            Units.degreesToRadians(135),  // L2
            Units.degreesToRadians(135),  // L3
            Units.degreesToRadians(120)   // L4
        };
        double[] elevatorHeights = {
            0.06,  // L1
            0.26,  // L2
            0.56,  // L3
            1.26   // L4
        };

        for (int level = 0; level < 4; level++) {
            for (int pos = 0; pos < GeneralConstants.sides * 2; pos++) {
                scoringStates[level][pos] = new SwerveSystem.SwerveSystemState(
                    armAngles[level],
                    elevatorHeights[level],
                    wristStraight,               // Scoring wrist position (straight)
                    GeneralConstants.allReef()[pos] // All reef positions (left and right)
                );
            }
        }
        return scoringStates;
    }

    

    // Offset for the arm pivot in AdvantageScope simulation
    public static final Translation3d armTransOffset = new Translation3d(0.11,-0.18,0.26);
} 