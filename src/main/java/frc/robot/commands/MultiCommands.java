package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Hang;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;

public class MultiCommands {
    private Arm arm;
    private Elevator elevator;
    private Wrist wrist;
    private Swerve swerve;
    private Intake intake;
    private Hang hang;
    private ArmCommands armCommands;
    private ElevatorCommands elevatorCommands;
    private WristCommands wristCommands;
    private SwerveCommands swerveCommands;
    private IntakeCommands intakeCommands;
    private HangCommand hangCommand;

    public MultiCommands(Arm arm,Elevator elevator,Wrist wrist,Swerve swerve,Intake intake,Hang hang,ArmCommands armCommands,ElevatorCommands elevatorCommands,WristCommands wristCommands,SwerveCommands swerveCommands,IntakeCommands intakeCommands,HangCommand hangCommand) {
        this.arm=arm;
        this.elevator=elevator;
        this.wrist=wrist;
        this.swerve=swerve;
        this.intake=intake;
        this.hang=hang;
        this.armCommands=armCommands;
        this.elevatorCommands=elevatorCommands;
        this.wristCommands=wristCommands;
        this.intakeCommands=intakeCommands;
        this.swerveCommands=swerveCommands;
        this.hangCommand=hangCommand;
    }

    public Command moveToGroundIntake(){
        return new ParallelCommandGroup(armCommands.moveToGroundIntake(),elevatorCommands.moveToGroundIntake());
    }

    public Command moveToDefault(){
        return new ParallelCommandGroup(armCommands.moveToDefault(),elevatorCommands.moveToDefault());
    }

    public Command moveToSource(){
        return new ParallelCommandGroup(armCommands.moveToSource(),elevatorCommands.moveToSource());
    }

    public Command moveToAlgae(){
        return new ParallelCommandGroup(armCommands.moveToAlgae(),elevatorCommands.moveToAlgae());
    }

    public Command moveToGoal(int goal){
        return new ParallelCommandGroup(armCommands.moveToGoal(goal),elevatorCommands.moveToGoal(goal));
    }

    public Command getCoral(Pose2d pose) {
        if (pose == null) {
            return new InstantCommand();
        } else {
            if (pose.getY() > 1.75 && pose.getY() < 6.3) {
                return new ParallelCommandGroup(
                    AutoBuilder.pathfindToPose(pose, SwerveConstants.constraints),
                    moveToGroundIntake()
                );
            } else {
                return new ParallelCommandGroup(
                    AutoBuilder.pathfindToPose(pose, SwerveConstants.constraints),
                    moveToSource()
                );
            }
        }
    }
}