package first.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.wpilib.epilogue.Epilogue;
import org.wpilib.epilogue.Logged;

@Logged
public class Robot extends LoggedRobot {
	private RobotContainer robotContainer;

	public Robot() {
		// Configure AdvantageKit logging
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        Logger.recordMetadata(
                "GitDirty",
                switch (BuildConstants.DIRTY) {
                    case 0 -> "All changes committed";
                    case 1 -> "Uncommitted changes";
                    default -> "Unknown";
                });
		switch(Constants.ROBOT_TYPE) {
			case SIM_2027:
				Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
				break;
			case IMAGINARY_PHYSICAL_ROBOT_2027:
				Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
				Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
				break;
		}
		Logger.start();

		robotContainer = new RobotContainer();
	}

	@Override
	public void robotPeriodic() {
		robotContainer.periodic();
		Epilogue.update(this);
	}
}
