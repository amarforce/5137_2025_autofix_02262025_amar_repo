package frc.robot;

import java.io.File;
import java.util.function.Supplier;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.elastic.*;
import frc.robot.other.AutoFactory;
import frc.robot.other.CageChoice;
import frc.robot.other.RobotUtils;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

@SuppressWarnings("unused")
public class RobotContainer {
	// Controllers for driver and operator
	private CommandPS5Controller driver;
	private CommandPS5Controller operator;

	// Subsystems
	private Vision vision;
	private Swerve swerve;
	private Elevator elevator;
	private Arm arm;
	private Wrist wrist;
	private Intake intake;
	private Hang hang;
	private ArmSystem armSystem;


	// Commands for each subsystem
	private SwerveCommands swerveCommands;
	private ElevatorCommands elevatorCommands;
	private ArmCommands armCommands;
	private WristCommands wristCommands;
	private IntakeCommands intakeCommands;
	private HangCommand hangCommand;
	private ArmSystemCommands armSystemCommands;
	private MultiCommands multiCommands;

	// Additional components
	private Reef reef;
	private ReefScoring reefScoring;

	private CageChoice cageChoice;

	// Factory for autonomous commands
	private AutoFactory autoFactory;

	private StringLogEntry log;

	/**
	 * Constructor for RobotContainer.
	 * Initializes all subsystems, commands, and binds controls.
	 */
	public RobotContainer() {
		// Start data log
		DataLogManager.start();
		DataLog dataLog=DataLogManager.getLog();
		DriverStation.startDataLog(dataLog);
		log = new StringLogEntry(dataLog, "container");
		StringLogEntry swerveLog = new StringLogEntry(dataLog, "swerve");

		try{
			// Initialize controllers
			driver = new CommandPS5Controller(0);
			operator = new CommandPS5Controller(1);

			// Initialize Reef and ReefScoring components
			reef = new Reef();
			reefScoring = new ReefScoring(reef);
			SmartDashboard.putData("Reef", reef);
			SmartDashboard.putData("ReefScoring", reefScoring);

			// // Initialize subsystems with data log
			vision = new Vision(reef,new StringLogEntry(dataLog, "vision"));
			swerve = new Swerve(new File(Filesystem.getDeployDirectory(), "swerve.json"), vision,swerveLog);
			elevator = new Elevator(new StringLogEntry(dataLog, "elevator"));
			arm = new Arm(new StringLogEntry(dataLog, "arm"));
			wrist = new Wrist(new StringLogEntry(dataLog, "wrist"));
			intake = new Intake(new StringLogEntry(dataLog, "intake"));
			hang = new Hang(new StringLogEntry(dataLog, "hang"));
			armSystem = new ArmSystem(arm, elevator, wrist);

			// Initialize commands for each subsystem
			swerveCommands = new SwerveCommands(swerve);
			elevatorCommands = new ElevatorCommands(elevator);
			armCommands = new ArmCommands(arm);
			wristCommands = new WristCommands(wrist);
			intakeCommands = new IntakeCommands(intake);
			hangCommand = new HangCommand(hang);
			armSystemCommands = new ArmSystemCommands(armSystem);
			multiCommands = new MultiCommands(armSystemCommands, swerveCommands, intakeCommands, hangCommand, reef);

			// Initialize cage choice
			cageChoice = new CageChoice();

			// Configure button bindings
			configureBindings();

			// Initialize autonomous command factory
			autoFactory = new AutoFactory(multiCommands);
		}catch(Exception e){
			log.append("Error while initializing: "+RobotUtils.getError(e));
		}
	}

	/**
	 * Configures button bindings for driver and operator controllers.
	 */
	private void configureBindings() {
		// Driver Bindings
		// Set default command for swerve to drive with joystick inputs
		swerve.setDefaultCommand(
			swerveCommands.drive(
				() -> -driver.getLeftY(),
				() -> -driver.getLeftX(),
				() -> -driver.getRightX(),
				() -> driver.R1().negate().getAsBoolean())
		);

		// Bind cross button to lock swerve
		driver.cross().whileTrue(swerveCommands.lock());

		// Bind buttons to drive to specific locations
		driver.triangle().onTrue(swerveCommands.driveToStation());
		driver.square().onTrue(swerveCommands.driveToPose(()->cageChoice.getCage()));
		driver.circle().onTrue(swerveCommands.driveToProcessor());

		// Bind D-pad buttons to drive to specific reef positions
		driver.povLeft().onTrue(swerveCommands.driveToReefLeft());
		driver.povUp().onTrue(swerveCommands.driveToReefCenter());
		driver.povRight().onTrue(swerveCommands.driveToReefRight());

		// Bind options button to reset gyro
		driver.options().onTrue(swerveCommands.resetGyro());

		/*
		// Example of SysId bindings (commented out)
		driver.povUp().onTrue(new InstantCommand(() -> swerve.setRoutine(swerve.m_sysIdRoutineTranslation)));
		driver.povLeft().onTrue(new InstantCommand(() -> swerve.setRoutine(swerve.m_sysIdRoutineSteer)));
		driver.povRight().onTrue(new InstantCommand(() -> swerve.setRoutine(swerve.m_sysIdRoutineRotation)));
		driver.options().and(driver.povDown().negate()).whileTrue(swerveCommands.sysIdDynamic(Direction.kForward));
		driver.options().and(driver.povDown()).whileTrue(swerveCommands.sysIdDynamic(Direction.kForward));
		driver.create().and(driver.povDown().negate()).whileTrue(swerveCommands.sysIdQuasistatic(Direction.kReverse));
		driver.create().and(driver.povDown()).whileTrue(swerveCommands.sysIdQuasistatic(Direction.kReverse));
		*/

		// Bind touchpad to cancel all commands
		driver.touchpad().onTrue(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));

		// For testing: Set default commands for elevator and arm with joystick inputs
		elevator.setDefaultCommand(elevatorCommands.changeGoal(() -> -operator.getLeftY() / 50));
		arm.setDefaultCommand(armCommands.changeGoal(() -> -operator.getLeftX() / 50));

		//operator.axisLessThan(0,0).onTrue(multiCommands.placeCoral(0));

		// Operator Bindings

		// Bind buttons to move to specific goals
		operator.triangle().onTrue(armSystemCommands.moveToGoal(()->4));
		operator.circle().onTrue(armSystemCommands.moveToGoal(()->3));
		operator.square().onTrue(armSystemCommands.moveToGoal(()->2));
		operator.cross().onTrue(armSystemCommands.moveToGoal(()->1));


		// Bind L2 button to outtake and stop intake
		operator.L2()
			.onTrue(intakeCommands.outtake())
			.onFalse(intakeCommands.stop());

		// Bind R2 button to intake until switched and stop intake
		operator.R2()
			.onTrue(intakeCommands.intakeUntilSwitched())
			.onFalse(intakeCommands.stop());

		// Bind touchpad to execute hang command
		operator.touchpad().onTrue(hangCommand);
	}

	/**
	 * Returns the autonomous command to be executed.
	 *
	 * @return The autonomous command
	 */
	public Command getAutonomousCommand() {
		return autoFactory.getAuto();
	}
}