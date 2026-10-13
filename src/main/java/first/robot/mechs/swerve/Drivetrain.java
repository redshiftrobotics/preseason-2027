package first.robot.mechs.swerve;

import java.util.Arrays;

import org.wpilib.command3.Mechanism;
import org.wpilib.math.estimator.SwerveDrivePoseEstimator;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveModulePosition;

import first.robot.Constants;

public class Drivetrain implements Mechanism {
    //Swerve drive
    private SwerveModule[] modules; //Order: FL, FR, BL, BR
    private SwerveDriveKinematics kinematics;
    private SwerveDrivePoseEstimator poseEstimator;

    //Gyro
    private GyroIO gyroIO;
    private GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    //No-gyro yaw estimation data
	private SwerveModulePosition[] lastKnownModulePositions;
	private Rotation2d yawEstimate = Rotation2d.ZERO;

    //Robot state
    private Pose2d pose = new Pose2d();
    private ChassisVelocities measuredVelocities = new ChassisVelocities();
    private ChassisVelocities targetVelocities = new ChassisVelocities();

    public Drivetrain(SwerveModuleIO flIO, SwerveModuleIO frIO, SwerveModuleIO blIO, SwerveModuleIO brIO, GyroIO _gyroIO) {
        //Initialize swerve modules
        modules = new SwerveModule[] {
				new SwerveModule(flIO, Constants.SWERVE_MODULE_FL_DISTANCE_FROM_CENTER, "FL"),
				new SwerveModule(frIO, Constants.SWERVE_MODULE_FR_DISTANCE_FROM_CENTER, "FR"),
				new SwerveModule(blIO, Constants.SWERVE_MODULE_BL_DISTANCE_FROM_CENTER, "BL"),
				new SwerveModule(brIO, Constants.SWERVE_MODULE_BR_DISTANCE_FROM_CENTER, "BR"),
		};

        //Gyro setup
        gyroIO = _gyroIO;

        //Configure kinematics and odometry
        kinematics = new SwerveDriveKinematics(Arrays.stream(modules).map(SwerveModule::getDistanceFromRobotCenter).toArray(Translation2d[]::new));
        lastKnownModulePositions = Arrays.stream(modules).map(SwerveModule::getCurrentModulePosition).toArray(SwerveModulePosition[]::new);
        poseEstimator = new SwerveDrivePoseEstimator(kinematics, Rotation2d.ZERO, lastKnownModulePositions, new Pose2d());
    }

    public void periodic() {
        
    }
}