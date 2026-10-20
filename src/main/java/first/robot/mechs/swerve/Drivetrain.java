package first.robot.mechs.swerve;

import java.util.Arrays;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.wpilib.annotation.NoDiscard;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.button.CommandXboxController;
import org.wpilib.driverstation.RobotState;
import org.wpilib.math.estimator.SwerveDrivePoseEstimator;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Twist2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;

import first.robot.Constants;
import first.robot.utils.JoystickInputHelper;

public class Drivetrain implements Mechanism {
    //Swerve drive
    private SwerveModule[] modules; //Order: FL, FR, BL, BR
    private SwerveDriveKinematics kinematics;
    private SwerveDrivePoseEstimator poseEstimator;

    //Gyro
    private GyroIO gyroIO;
    private GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private Rotation2d yaw = Rotation2d.ZERO;

    //No-gyro yaw estimation data
	private SwerveModulePosition[] lastKnownModulePositions;

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

    public void setTargetRobotVelocities(ChassisVelocities velocities) {
        //Discretize speeds to account for loop periods instead of continuous control
        targetVelocities = velocities.discretize(Constants.LOOP_PERIOD_SECONDS);
        measuredVelocities = targetVelocities;

        //Set wheel velocities
        setTargetWheelVelocities(kinematics.toSwerveModuleVelocities(targetVelocities));
    }

    public ChassisVelocities getTargetRobotVelocities() {
        return targetVelocities;
    }

    public ChassisVelocities getMeasuredRobotVelocities() {
        return measuredVelocities;
    }

    public void setTargetWheelVelocities(SwerveModuleVelocity[] velocities) {
        //Adjust speeds to account for speed caps
        velocities = SwerveDriveKinematics.desaturateWheelVelocities(velocities, Constants.DRIVETRAIN_CONFIG.maxLinearVelocity());

        //Apply speeds
        for(byte i = 0; i < modules.length; i++) {
            modules[i].setTargetModuleVelocity(velocities[i]);
        }
    }

    SwerveModuleVelocity[] getTargetWheelVelocities() {
		return Arrays.stream(modules).map(SwerveModule::getTargetModuleVelocity).toArray(SwerveModuleVelocity[]::new);
	}

	SwerveModuleVelocity[] getMeasuredWheelVelocities() {
		return Arrays.stream(modules).map(SwerveModule::getMeasuredModuleVelocity).toArray(SwerveModuleVelocity[]::new);
	}
    
    @AutoLogOutput(key = "Pose")
    public Pose2d getPose() {
        return pose;
    }

    public void resetPose() {
        //Reset pose estimator
        poseEstimator.resetPosition(yaw, Arrays.stream(modules).map(SwerveModule::getCurrentModulePosition).toArray(SwerveModulePosition[]::new), new Pose2d());
    }

    public void resetPose(Pose2d _pose) {
        //Reset pose estimator
        poseEstimator.resetPosition(yaw, Arrays.stream(modules).map(SwerveModule::getCurrentModulePosition).toArray(SwerveModulePosition[]::new), _pose);
    }

    @AutoLogOutput(key = "Yaw")
    public Rotation2d getYaw() {
        return yaw;
    }

    public void stop() {
        //Set zero chassis velocities
        setTargetRobotVelocities(new ChassisVelocities());
    }

    public void periodic() {
        //Update inputs
        gyroIO.updateInputs(gyroInputs);
        Arrays.stream(modules).forEach(SwerveModule::updateInputs);

        //Log data
        Logger.processInputs("Drive/Gyro", gyroInputs);
        Logger.recordOutput("Drive/WheelVelocities/Measured", getMeasuredWheelVelocities());
        Logger.recordOutput("Drive/WheelVelocities/Target", getTargetWheelVelocities());
        Logger.recordOutput("Drive/RobotVelocities/Measured", getMeasuredRobotVelocities());
        Logger.recordOutput("Drive/RobotVelocities/Target", getTargetRobotVelocities());

        //Stop if robot has been disabled
        if(RobotState.isDisabled()) stop();

        //Fetch yaw
		if(gyroInputs.gyroConnected) {
            //Very easy
			yaw = gyroInputs.yaw;
		} else {
			//Calculate module deltas to guesstimate turn (hard)
			//This code is slightly modified from code in Drive.java, redshift-robotics/reefscape2025
			SwerveModulePosition[] modulePositions = new SwerveModulePosition[modules.length];
			SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[modules.length];
			for(int i = 0; i < modules.length; i++) {
				// Set current position
				modulePositions[i] = modules[i].getCurrentModulePosition();

				//Calculate delta
				moduleDeltas[i] = new SwerveModulePosition(
						modulePositions[i].distance - lastKnownModulePositions[i].distance,
						modulePositions[i].angle);

				//Copy last position
				lastKnownModulePositions[i] = modulePositions[i];
			}

			//Convert deltas to twist
			Twist2d twist = kinematics.toTwist2d(moduleDeltas);

			//Update yaw
			yaw = yaw.plus(new Rotation2d(twist.dtheta));
		}

        //Update pose estimator
        pose = poseEstimator.update(yaw, Arrays.stream(modules).map(SwerveModule::getCurrentModulePosition).toArray(SwerveModulePosition[]::new));
    }

    public Command teleOpDrive(JoystickInputHelper input) {
        return run(coroutine -> {
            while(RobotState.isTeleop()) {
                if(RobotState.isEnabled()) {
                    Translation2d translation = input.getTranslation();
                    ChassisVelocities velocities = new ChassisVelocities(translation.getX(), translation.getY(), input.getOmega());
                    setTargetRobotVelocities(velocities);
                }
                coroutine.yield();
            }
           stop();
        }).named("Tele-Op Drive");
    }
}