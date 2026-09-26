package first.robot;

import org.wpilib.command3.button.CommandXboxController;

public class RobotContainer {
    private final CommandXboxController driverController = new CommandXboxController(Constants.DRIVER_CONTROLLER_PORT_ID);

    public RobotContainer() {
        System.out.println("Hello, I am the robot container and I am here to initialize!");

        configureBindings();

        System.out.println("Hello, I am the robot container, and I am done initializing!");
    }

    public void configureBindings() {

    }
}
