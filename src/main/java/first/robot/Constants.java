package first.robot;

import static org.wpilib.units.Units.MetersPerSecond;

import org.wpilib.framework.RobotBase;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.util.Units;

import first.robot.generated.TunerConstants2026;

public class Constants {
    private Constants() {}

    /** Type of robot we're running on */
	public enum RobotType {
		SIM_2027,
		IMAGINARY_PHYSICAL_ROBOT_2027
	}
	public static final RobotType ROBOT_TYPE = (RobotBase.isReal() ? RobotType.IMAGINARY_PHYSICAL_ROBOT_2027 : RobotType.SIM_2027);

    /** The period, in seconds, of the main robot loop */
    public static final double LOOP_PERIOD_SECONDS = Robot.defaultPeriodSecs; // 0.02

    /** The port ID of the driver's controller */
    public static final int DRIVER_CONTROLLER_PORT_ID = 0;

    /** Joystick control deadband */
	public static final double JOYSTICK_DEADBAND = 0.15;

	/** Gyro device ID */
	public static final int GYRO_DEVICE_ID = switch(ROBOT_TYPE) {
		case SIM_2027 -> -1;
		case IMAGINARY_PHYSICAL_ROBOT_2027 -> TunerConstants2026.DrivetrainConstants.Pigeon2Id;
	};

    /** Drivetrain PID gains */
    public static final double MODULE_DRIVE_PID_KP = switch(ROBOT_TYPE) {
		case SIM_2027 -> 1.3;
		case IMAGINARY_PHYSICAL_ROBOT_2027 -> 20.0;
	};
    public static final double MODULE_DRIVE_PID_KI = switch(ROBOT_TYPE) {
		case SIM_2027 -> 0.0;
		case IMAGINARY_PHYSICAL_ROBOT_2027 -> 0.0;
	};
    public static final double MODULE_DRIVE_PID_KD = switch(ROBOT_TYPE) {
		case SIM_2027 -> 0.0;
		case IMAGINARY_PHYSICAL_ROBOT_2027 -> 0.0;
	};
    public static final double MODULE_TURN_PID_KP  = switch(ROBOT_TYPE) {
		case SIM_2027 -> 10.0;
		case IMAGINARY_PHYSICAL_ROBOT_2027 -> 1400.0;
	};
    public static final double MODULE_TURN_PID_KI = switch(ROBOT_TYPE) {
		case SIM_2027 -> 0.0;
		case IMAGINARY_PHYSICAL_ROBOT_2027 -> 0.0;
	};
    public static final double MODULE_TURN_PID_KD = switch(ROBOT_TYPE) {
		case SIM_2027 -> 0.0;
		case IMAGINARY_PHYSICAL_ROBOT_2027 -> 15.0;
	};

    /** Drivetrain feedforward gains */
    public static final double MODULE_DRIVE_FFWD_KS = switch(ROBOT_TYPE) {
		case SIM_2027 -> 0.0;
		case IMAGINARY_PHYSICAL_ROBOT_2027 -> 2.00544;
	};
    public static final double MODULE_DRIVE_FFWD_KV = switch(ROBOT_TYPE) {
		case SIM_2027 -> 0.0;
		case IMAGINARY_PHYSICAL_ROBOT_2027 -> 1.05719;
	};
    public static final double MODULE_DRIVE_FFWD_KA = switch(ROBOT_TYPE) {
		case SIM_2027 -> 0.0;
		case IMAGINARY_PHYSICAL_ROBOT_2027 -> 0.0;
	};
	public static final double MODULE_TURN_FFWD_KS = switch(ROBOT_TYPE) {
		case SIM_2027 -> 0.0;
		case IMAGINARY_PHYSICAL_ROBOT_2027 -> 0.2;
	};
    public static final double MODULE_TURN_FFWD_KV = switch(ROBOT_TYPE) {
		case SIM_2027 -> 0.0;
		case IMAGINARY_PHYSICAL_ROBOT_2027 -> 0.0;
	};
    public static final double MODULE_TURN_FFWD_KA = switch(ROBOT_TYPE) {
		case SIM_2027 -> 0.0;
		case IMAGINARY_PHYSICAL_ROBOT_2027 -> 0.0;
	};

    /** Robot wheel radius */
    public static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(2);

	// These reductions were copied from ModuleConstants.java in
	// redshiftrobotics/preseason-2026!
	// Originally sourced from:
	// https://www.swervedrivespecialties.com/products/mk5n-swerve-module
	public enum Mk5nReductions {
		L1(12.0),
		L2(14.0),
		L3(16.0);
        
        public static final double TURN = (287.0 / 11.0);
		public final double reduction;

		Mk5nReductions(double adjustableGearTeeth) {
			this.reduction = (54.0 / adjustableGearTeeth) * (25.0 / 32.0) * (30.0 / 15.0);
		}
	}

	/** Reductions */
	public static final double DRIVE_REDUCTION = Mk5nReductions.L3.reduction;
	public static final double TURN_REDUCTION = Mk5nReductions.TURN;

	/** Drive configuration
	  * frameDiagonal is the diagonal length between the corners of the inner metal frame
	  * bumperDiagnal is the same but between the corners of the bumpers
      */
	public record DriveConfiguration(Translation2d frameDiagonal, Translation2d bumperDiagonal, double maxLinearVelocity, double maxLinearAcceleration) {
		public double getBaseRadius() {
			return frameDiagonal.getNorm() / 2;
		}

		public double maxAngularVelocity() {
			return maxLinearVelocity / getBaseRadius();
		}

		public double maxAngularAcceleration() {
			return maxLinearAcceleration / getBaseRadius();
		}
	}

    /** Our resolved drivetrain configuration */
	public static final DriveConfiguration DRIVETRAIN_CONFIG = switch(ROBOT_TYPE) {
		// These values were copied from DriveConstants.java in
		// redshiftrobotics/reefscape-2025
		case SIM_2027 -> new DriveConfiguration(new Translation2d(0.885, 0.885), new Translation2d(0.9612, 0.9612), 5.05968, 14.5);
		case IMAGINARY_PHYSICAL_ROBOT_2027 -> new DriveConfiguration(new Translation2d(
				TunerConstants2026.FrontLeft.LocationX - TunerConstants2026.BackRight.LocationX,
				TunerConstants2026.FrontLeft.LocationY - TunerConstants2026.BackRight.LocationY),
				new Translation2d(
						TunerConstants2026.FrontLeft.LocationX - TunerConstants2026.BackRight.LocationX,
						TunerConstants2026.FrontLeft.LocationY - TunerConstants2026.BackRight.LocationY)
								.plus(new Translation2d(6.750, 6.750).times(2)),
				TunerConstants2026.kSpeedAt12Volts.in(MetersPerSecond), 22.0);
	};

    /** Center of robot positions */
    public static final double CENTER_X = DRIVETRAIN_CONFIG.frameDiagonal.getX() / 2;
	public static final double CENTER_Y = DRIVETRAIN_CONFIG.frameDiagonal.getY() / 2;

    /** Module positions */
	public static final Translation2d SWERVE_MODULE_FL_DISTANCE_FROM_CENTER = new Translation2d(CENTER_X, CENTER_Y);
	public static final Translation2d SWERVE_MODULE_FR_DISTANCE_FROM_CENTER = new Translation2d(CENTER_X, -CENTER_Y);
	public static final Translation2d SWERVE_MODULE_BL_DISTANCE_FROM_CENTER = new Translation2d(-CENTER_X, CENTER_Y);
	public static final Translation2d SWERVE_MODULE_BR_DISTANCE_FROM_CENTER = new Translation2d(-CENTER_X, -CENTER_Y);

    /** Swerve module configuration */
    public record SwerveModuleConfiguration(int driveMotorID, int turnMotorID, int encoderID, Rotation2d encoderOffset, boolean turnMotorInverted) {}

    /** Resolved swerve module configurations
     *  WARNING: THE TUNER CONSTANTS BEING USED ARE FROM 2026 AND ARE NOT ACCURATE!!! THEY ARE HERE SO WE HAVE A DATA SOURCE!!!
     */
	public static final SwerveModuleConfiguration SWERVE_MODULE_FL_CONFIG = switch(ROBOT_TYPE) {
		case SIM_2027 -> new SwerveModuleConfiguration(0, 0, 0, Rotation2d.ZERO, false);
		case IMAGINARY_PHYSICAL_ROBOT_2027 -> new SwerveModuleConfiguration(TunerConstants2026.FrontLeft.DriveMotorId,
				TunerConstants2026.FrontLeft.SteerMotorId, TunerConstants2026.FrontLeft.EncoderId,
				Rotation2d.fromRotations(TunerConstants2026.FrontLeft.EncoderOffset),
				TunerConstants2026.FrontLeft.SteerMotorInverted);
	};
	public static final SwerveModuleConfiguration SWERVE_MODULE_FR_CONFIG = switch(ROBOT_TYPE) {
		case SIM_2027 -> new SwerveModuleConfiguration(0, 0, 0, Rotation2d.ZERO, false);
		case IMAGINARY_PHYSICAL_ROBOT_2027 -> new SwerveModuleConfiguration(TunerConstants2026.FrontRight.DriveMotorId,
				TunerConstants2026.FrontRight.SteerMotorId, TunerConstants2026.FrontRight.EncoderId,
				Rotation2d.fromRotations(TunerConstants2026.FrontRight.EncoderOffset),
				TunerConstants2026.FrontRight.SteerMotorInverted);
	};
	public static final SwerveModuleConfiguration SWERVE_MODULE_BL_CONFIG = switch(ROBOT_TYPE) {
		case SIM_2027 -> new SwerveModuleConfiguration(0, 0, 0, Rotation2d.ZERO, false);
		case IMAGINARY_PHYSICAL_ROBOT_2027 -> new SwerveModuleConfiguration(TunerConstants2026.BackLeft.DriveMotorId,
				TunerConstants2026.BackLeft.SteerMotorId, TunerConstants2026.BackLeft.EncoderId,
				Rotation2d.fromRotations(TunerConstants2026.BackLeft.EncoderOffset),
				TunerConstants2026.BackLeft.SteerMotorInverted);
	};
	public static final SwerveModuleConfiguration SWERVE_MODULE_BR_CONFIG = switch(ROBOT_TYPE) {
		case SIM_2027 -> new SwerveModuleConfiguration(0, 0, 0, Rotation2d.ZERO, false);
		case IMAGINARY_PHYSICAL_ROBOT_2027 -> new SwerveModuleConfiguration(TunerConstants2026.BackRight.DriveMotorId,
				TunerConstants2026.BackRight.SteerMotorId, TunerConstants2026.BackRight.EncoderId,
				Rotation2d.fromRotations(TunerConstants2026.BackRight.EncoderOffset),
				TunerConstants2026.BackRight.SteerMotorInverted);
	};

    /** CAN bus name */
    public static final String CAN_BUS_NAME = "canivore";

	/** Drivetrain device disconnect warning thresholds
	  * Basically, if the device is still disconnected after this time we know
	  * something is up 
	  */
	public static final double DRIVETRAIN_MOTOR_DISCONNECT_WARNING_THRESHOLD_SECONDS = 0.5;
	public static final double DRIVETRAIN_ENCODER_DISCONNECT_WARNING_THRESHOLD_SECONDS = 0.2;

	/** Drive odometry polling frequency */
	public static final double DRIVETRAIN_ODOMETRY_FREQUENCY_HZ = switch(ROBOT_TYPE) { // why are we polling at a rate higher than our update rate? - acei
		case SIM_2027 -> 50.0;
		case IMAGINARY_PHYSICAL_ROBOT_2027 -> (TunerConstants2026.kCANBus.isNetworkFD() ? 250.0 : 100.0);
		default -> 100.0;
	};
}
