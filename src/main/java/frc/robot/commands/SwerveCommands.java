package frc.robot.commands;

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.GeneralConstants;
import frc.robot.constants.SwerveConstants;
import frc.robot.other.DetectedObject;
import frc.robot.subsystems.Swerve;

/**
 * This class encapsulates all the commands related to the Swerve subsystem.
 * It provides methods to control the swerve drive, including driving to specific poses,
 * locking the swerve, resetting the gyro, and running system identification routines.
 */
public class SwerveCommands {
    private Swerve swerve;

    /**
     * Constructor for SwerveCommands.
     *
     * @param swerve The Swerve subsystem that this class will control.
     */
    public SwerveCommands(Swerve swerve) {
        this.swerve = swerve;
    }

    /**
     * Creates a command to drive the swerve subsystem based on the provided inputs.
     *
     * @param dx The supplier for the x-axis (forward/backward) movement.
     * @param dy The supplier for the y-axis (left/right) movement.
     * @param dtheta The supplier for the rotational movement.
     * @param fieldOriented The supplier for whether the drive should be field-oriented.
     * @return A command that drives the swerve subsystem.
     */
    public Command drive(DoubleSupplier dx, DoubleSupplier dy, DoubleSupplier dtheta, BooleanSupplier fieldOriented) {
        return new InstantCommand(
            () -> swerve.setPercentDrive(dx.getAsDouble(), dy.getAsDouble(), dtheta.getAsDouble(), fieldOriented.getAsBoolean()),
            swerve
        );
    }

    /**
     * Creates a command to drive the swerve subsystem to a specific pose.
     *
     * @param pose The supplier for the target pose.
     * @return A command that drives the swerve subsystem to the specified pose.
     */
    public Command driveToPose(Supplier<Pose2d> pose){
        Command auto = AutoBuilder.pathfindToPose(pose.get(), SwerveConstants.constraints);
        auto.addRequirements(swerve);
        return auto;
    }

    /**
     * Creates a command to drive the swerve subsystem to the closest station.
     *
     * @return A command that drives the swerve subsystem to the closest station.
     */
    public Command driveToStation() {
        return driveToPose(() -> swerve.getClosest(GeneralConstants.stations));
    }

    /**
     * Creates a command to drive the swerve subsystem to the closest left reef.
     *
     * @return A command that drives the swerve subsystem to the closest left reef.
     */
    public Command driveToReefLeft() {
        return driveToPose(() -> swerve.getClosest(GeneralConstants.leftReef));
    }

    /**
     * Creates a command to drive the swerve subsystem to the closest center reef.
     *
     * @return A command that drives the swerve subsystem to the closest center reef.
     */
    public Command driveToReefCenter() {
        return driveToPose(() -> swerve.getClosest(GeneralConstants.centerReef));
    }

    /**
     * Creates a command to drive the swerve subsystem to the closest right reef.
     *
     * @return A command that drives the swerve subsystem to the closest right reef.
     */
    public Command driveToReefRight() {
        return driveToPose(() -> swerve.getClosest(GeneralConstants.rightReef));
    }

    /**
     * Creates a command to drive the swerve subsystem to the processor.
     *
     * @return A command that drives the swerve subsystem to the processor.
     */
    public Command driveToProcessor() {
        return driveToPose(() -> GeneralConstants.processor);
    }

    /**
     * Creates a command to lock the swerve subsystem in place.
     *
     * @return A command that locks the swerve subsystem.
     */
    public Command lock() {
        return new InstantCommand(() -> swerve.lock(), swerve);
    }

    /**
     * Creates a command to reset the gyro of the swerve subsystem.
     *
     * @return A command that resets the gyro.
     */
    public Command resetGyro() {
        return new InstantCommand(() -> swerve.resetGyro(), swerve);
    }

    public Command driveToCoral(){
        return driveToPose(()->{
            List<DetectedObject> objects=swerve.getGroundCoral();
            Pose2d[] poses=new Pose2d[objects.size()];
            for(int i=0;i<poses.length;i++){
                poses[i]=objects.get(i).getPose().toPose2d();
            }
            return swerve.getClosest(poses);
        });
    }

    /**
     * Creates a command to run a quasistatic system identification routine.
     *
     * @param dir The direction of the quasistatic test.
     * @return A command that runs the quasistatic system identification routine.
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction dir) {
        return swerve.getRoutine().quasistatic(dir);
    }

    /**
     * Creates a command to run a dynamic system identification routine.
     *
     * @param dir The direction of the dynamic test.
     * @return A command that runs the dynamic system identification routine.
     */
    public Command sysIdDynamic(SysIdRoutine.Direction dir) {
        return swerve.getRoutine().dynamic(dir);
    }
}