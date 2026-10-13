package first.robot;

import org.wpilib.command3.Scheduler;
import org.wpilib.command3.button.CommandXboxController;

import first.robot.mechs.swerve.Drivetrain;
import first.robot.mechs.swerve.GyroIO;
import first.robot.mechs.swerve.SwerveModuleIOSim;
import first.robot.mechs.swerve.SwerveModuleIOTalonFX;

public class RobotContainer {
    //Controller
    private final CommandXboxController driverController = new CommandXboxController(Constants.DRIVER_CONTROLLER_PORT_ID);

    //Subsystems
    private final Drivetrain drivetrain;

    public RobotContainer() {
        System.out.println("Hello, I am the robot container and I am here to initialize!");

        //Create subsystems
        switch(Constants.ROBOT_TYPE) {
            case SIM_2027:
                drivetrain = new Drivetrain(new SwerveModuleIOSim(), new SwerveModuleIOSim(), new SwerveModuleIOSim(), new SwerveModuleIOSim(), new GyroIO() {});
                break;
            case IMAGINARY_PHYSICAL_ROBOT_2027:
                drivetrain = new Drivetrain(new SwerveModuleIOTalonFX(Constants.SWERVE_MODULE_FL_CONFIG), new SwerveModuleIOTalonFX(Constants.SWERVE_MODULE_FR_CONFIG), 
                    new SwerveModuleIOTalonFX(Constants.SWERVE_MODULE_BL_CONFIG), new SwerveModuleIOTalonFX(Constants.SWERVE_MODULE_BR_CONFIG), new GyroIO() {});
                break;
            default:
                drivetrain = null;
                break;
        }

        //Register periodic callbacks
        Scheduler.getDefault().addPeriodic(drivetrain::periodic);

        //Set up controller
        configureBindings();

        System.out.println("Hello, I am the robot container, and I am done initializing!");
    }

    public void configureBindings() {

    }
}
