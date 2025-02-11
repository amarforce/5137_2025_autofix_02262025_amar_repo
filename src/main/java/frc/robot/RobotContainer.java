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
import frc.robot.constants.SwerveConstants;

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
	private SwerveSystem swerveSystem;
	private LED led;


	// Commands for each subsystem
	private SwerveCommands swerveCommands;
	private ElevatorCommands elevatorCommands;
	private ArmCommands armCommands;
	private WristCommands wristCommands;
	private IntakeCommands intakeCommands;
	private HangCommand hangCommand;
	private SwerveSystemCommands swerveSystemCommands;
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

		try{
			// Initialize controllers
			driver = new CommandPS5Controller(0);
			operator = new CommandPS5Controller(1);

			// Initialize Reef and ReefScoring components
			reef = new Reef();
			reefScoring = new ReefScoring(reef);
			SmartDashboard.putData("reef", reef);
			SmartDashboard.putData("reefScoring", reefScoring);

			// // Initialize subsystems with data log
			vision = new Vision(reef);
			swerve = new Swerve(new File(Filesystem.getDeployDirectory(), "swerve.json"), vision);
			elevator = new Elevator();
			arm = new Arm();
			wrist = new Wrist();
			intake = new Intake();
			hang = new Hang();
			swerveSystem = new SwerveSystem(arm, elevator, wrist, swerve);
			led = new LED();


			// Initialize commands for each subsystem
			swerveCommands = new SwerveCommands(swerve);
			elevatorCommands = new ElevatorCommands(elevator);
			armCommands = new ArmCommands(arm);
			wristCommands = new WristCommands(wrist);
			intakeCommands = new IntakeCommands(intake);
			hangCommand = new HangCommand(hang);
			swerveSystemCommands = new SwerveSystemCommands(swerveSystem);
			multiCommands = new MultiCommands(swerveSystemCommands, swerveCommands, intakeCommands, hangCommand);


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
		//Driver Bindings
		// Set default command for swerve to drive with joystick inputs
		swerve.setDefaultCommand(swerveCommands.drive(
			() -> -driver.getLeftY(), 
			() -> -driver.getLeftX(), 
			() -> driver.getRightX(), 
			() -> true)
		);

		// Bind cross button to lock swerve
		driver.cross().whileTrue(swerveCommands.lock());

		// Bind buttons to drive to specific locations
		driver.triangle().onTrue(swerveSystemCommands.moveToSource());
		//driver.square().onTrue(swerveCommands.driveToPoseStaticFixed(()->cageChoice.getCage()));
		driver.circle().onTrue(swerveSystemCommands.moveToProcessor());

		// Bind D-pad buttons to drive to specific reef positions
		// driver.povLeft().onTrue(swerveCommands.driveToReefLeft());
		// driver.povUp().onTrue(swerveCommands.driveToReefCenter());
		// driver.povRight().onTrue(swerveCommands.driveToReefRight());

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
		arm.setDefaultCommand(armCommands.changeGoal(() -> -operator.getRightX() / 50));
		elevator.setDefaultCommand(elevatorCommands.changeGoal(() -> -operator.getLeftY() / 50));
		operator.L1().onTrue(wristCommands.changeGoal(() -> Math.toRadians(-5)));
		operator.R1().onTrue(wristCommands.changeGoal(() -> Math.toRadians(5)));

		//operator.axisLessThan(0,0).onTrue(multiCommands.placeCoral(0));

		// Operator Bindings

		// Bind buttons to move to specific goals
		
		operator.triangle().onTrue(swerveSystemCommands.moveToLevel(3)); // L1
		operator.circle().onTrue(swerveSystemCommands.moveToLevel(2)); // L2
		operator.square().onTrue(swerveSystemCommands.moveToLevel(1)); // L3
		operator.cross().onTrue(swerveSystemCommands.moveToLevel(0)); // L4



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