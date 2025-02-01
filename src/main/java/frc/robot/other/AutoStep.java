package frc.robot.other;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.MultiCommands;
import frc.robot.constants.GeneralConstants;
import frc.robot.constants.SwerveConstants;

/**
 * The `AutoStep` class is responsible for creating and managing three {@link SendableChooser}s
 * for level selection, reef position, and pickup position. These choosers are displayed on the
 * SmartDashboard, allowing users to select different options for autonomous routines.
 */
public class AutoStep {

    private int id; // Unique identifier for this AutoStep instance
    private MultiCommands multiCommands; // Reference to the MultiCommands class for executing commands

    // Choosers for selecting level, reef position, and pickup position
    private SendableChooser<Integer> levelChooser;
    private SendableChooser<Pose2d> reefChooser;
    private SendableChooser<Pose2d> pickupChooser;

    /**
     * Constructor for the AutoStep class.
     *
     * @param id The unique identifier for this AutoStep instance.
     * @param multiCommands Reference to the MultiCommands class for executing commands.
     */
    public AutoStep(int id, MultiCommands multiCommands) {
        this.id = id;
        this.multiCommands = multiCommands;

        // Initialize the level chooser with default and additional options
        levelChooser = new SendableChooser<Integer>();
        levelChooser.setDefaultOption("L4", 4);
        levelChooser.addOption("L3", 3);
        levelChooser.addOption("L2", 2);
        levelChooser.addOption("L1", 1);
        levelChooser.addOption("Algae", 0);
        levelChooser.addOption("No Auto",-1);
        SmartDashboard.putData("Level Choice " + id, levelChooser);

        // Initialize the reef and pickup choosers with coral poses
        switchToCoralPoses();

        // Add a listener to the level chooser to switch between coral and algae poses
        levelChooser.onChange((Integer choice) -> {
            if (choice.equals(0)) {
                switchToAlgaePoses();
            } else if(choice.equals(-1)){
                switchToNoAuto();
            }else{
                switchToCoralPoses();
            }
        });
    }

    /**
     * Initializes the reef and pickup choosers with coral poses.
     */
    private void switchToCoralPoses() {
        // Initialize the reef chooser with default and additional options
        reefChooser = new SendableChooser<Pose2d>();
        reefChooser.setDefaultOption("A", SwerveConstants.allReef[0]);
        for (int i = 1; i < GeneralConstants.sides * 2; i++) {
            reefChooser.addOption(Character.toString('A' + i), SwerveConstants.allReef[i]);
        }
        SmartDashboard.putData("Reef Choice " + id, reefChooser);

        // Initialize the pickup chooser with default and additional options
        pickupChooser = new SendableChooser<Pose2d>();
        pickupChooser.setDefaultOption("RightStation Right", SwerveConstants.stationA);
        pickupChooser.addOption("RightStation Left", SwerveConstants.stationB);
        pickupChooser.addOption("LeftStation Right", SwerveConstants.stationC);
        pickupChooser.addOption("LeftStation Left", SwerveConstants.stationD);
        pickupChooser.addOption("Left Ground", SwerveConstants.leftPickup);
        pickupChooser.addOption("Center Ground", SwerveConstants.centerPickup);
        pickupChooser.addOption("Right Ground", SwerveConstants.rightPickup);
        SmartDashboard.putData("Pickup Choice " + id, pickupChooser);
    }

    /**
     * Initializes the reef and pickup choosers with algae poses.
     */
    private void switchToAlgaePoses() {
        // Initialize the reef chooser with default and additional options for algae poses
        reefChooser = new SendableChooser<Pose2d>();
        reefChooser.setDefaultOption("AB", SwerveConstants.centerReef[0]);
        for (int i = 1; i < GeneralConstants.sides; i++) {
            reefChooser.addOption(Character.toString('A' + (2 * i)) + Character.toString('A' + (2 * i + 1)), SwerveConstants.centerReef[i]);
        }
        SmartDashboard.putData("Reef Choice " + id, reefChooser);

        // Initialize the pickup chooser (no options added for algae poses)
        pickupChooser = new SendableChooser<Pose2d>();
        SmartDashboard.putData("Pickup Choice " + id, pickupChooser);
    }

    private void switchToNoAuto(){
        pickupChooser = new SendableChooser<Pose2d>();
        SmartDashboard.putData("Pickup Choice " + id, pickupChooser);

        reefChooser = new SendableChooser<Pose2d>();
        SmartDashboard.putData("Reef Choice " + id, reefChooser);
    }

    /**
     * Generates a command sequence based on the selected options from the choosers.
     *
     * @return A {@link Command} representing the sequence of actions to be executed.
     */
    public Command getCommand() {
        int level=levelChooser.getSelected();
        if(level==-1){
            return new InstantCommand();
        }else if(level==0){
            return new ParallelCommandGroup(
                multiCommands.moveToAlgae(),
                multiCommands.getSwerveCommands().driveToPose(()->RobotUtils.invertPoseToAlliance(reefChooser.getSelected()))
            );
        }else{
            return new SequentialCommandGroup(
                multiCommands.getCoral(RobotUtils.invertPoseToAlliance(pickupChooser.getSelected())),
                new ParallelCommandGroup(
                    multiCommands.moveToGoal(levelChooser.getSelected()),
                    multiCommands.getSwerveCommands().driveToPose(() -> RobotUtils.invertPoseToAlliance(reefChooser.getSelected()))
                ),
                multiCommands.getIntakeCommands().outtake()
            );
        }
        
    }

    /**
     * Retrieves the selected reef pose.
     *
     * @return The selected {@link Pose2d} for the reef position.
     */
    public Pose2d getPose() {
        return RobotUtils.invertPoseToAlliance(reefChooser.getSelected());
    }

    /**
     * Retrieves the selected pickup pose.
     *
     * @return The selected {@link Pose2d} for the pickup position.
     */
    public Pose2d getPickup() {
        return RobotUtils.invertPoseToAlliance(pickupChooser.getSelected());
    }
}