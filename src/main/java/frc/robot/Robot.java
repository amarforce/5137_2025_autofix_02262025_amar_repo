package frc.robot;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.GeneralConstants;

public class Robot extends TimedRobot {
	private Command autonomousCommand;

	private final RobotContainer robotContainer;
	private Timer logTimer;

	public Robot() {
		robotContainer = new RobotContainer();
		logTimer=new Timer();
		logTimer.start();
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
		if(logTimer.hasElapsed(GeneralConstants.logPeriod)){
			logTimer.restart();
			//robotContainer.log();
		}
	}

	@Override
	public void disabledInit() {}

	@Override
	public void disabledPeriodic() {}

	@Override
	public void disabledExit() {}

	@Override
	public void autonomousInit() {
		autonomousCommand = robotContainer.getAutonomousCommand();

		if (autonomousCommand != null) {
			autonomousCommand.schedule();
		}
	}

	@Override
	public void autonomousPeriodic() {}

	@Override
	public void autonomousExit() {}

	@Override
	public void teleopInit() {
		if (autonomousCommand != null) {
		autonomousCommand.cancel();
		}
	}

	@Override
	public void teleopPeriodic() {}

	@Override
	public void teleopExit() {}

	@Override
	public void testInit() {
		CommandScheduler.getInstance().cancelAll();
	}

	@Override
	public void testPeriodic() {}

	@Override
	public void testExit() {}
}
