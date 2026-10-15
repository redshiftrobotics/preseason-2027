package first.robot.utils;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.util.MathUtil;

import first.robot.Constants;

import org.wpilib.driverstation.DriverStation;
import org.wpilib.driverstation.RobotState;

public class JoystickInputHelper {
	private DoubleSupplier xSupplier, ySupplier, yAngleSupplier;

	public JoystickInputHelper(DoubleSupplier _xSupplier, DoubleSupplier _ySupplier, DoubleSupplier _yAngleSupplier) {
		xSupplier = _xSupplier;
		ySupplier = _ySupplier;
		yAngleSupplier = _yAngleSupplier;
	}

	public static double applyDeadbanding(double input) {
		return MathUtil.applyDeadband(input, Constants.JOYSTICK_DEADBAND);
	}

	public static boolean isWithinDeadbandRange(double input) {
		return Math.abs(input) < Constants.JOYSTICK_DEADBAND;
	}

	//Calculate robot translation in meters per second
	//These calculations are mostly copied from JoystickInputController.java in
	//redshiftrobotics/reefscape-2025
	@AutoLogOutput(key = "TeleOpInput/Driver/TranslationMetersPerSecond")
	public Translation2d getTranslation() {
		//No effect in autonomous mode
		if(RobotState.isAutonomous())
			return Translation2d.ZERO;

		//Get current inputs
		Translation2d translation = new Translation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());

		//Deadbanding
		double magnitude = applyDeadbanding(translation.getNorm());
		if(magnitude <= 0)
			return Translation2d.ZERO;

		//Calculate new translation
		//We apply ramp-up and fine control adjustments by squaring magnitude
		double magnitudeSquared = Math.pow(magnitude, 2);
		Translation2d translationSquared = new Pose2d(Translation2d.ZERO, translation.getAngle().orElse(Rotation2d.ZERO))
				.transformBy(new Transform2d(magnitudeSquared, 0.0, Rotation2d.ZERO)).getTranslation();

		//Return final translation accounting for drive max speed
		return translationSquared.times(Constants.DRIVETRAIN_CONFIG.maxLinearVelocity());
	}

	//Calculate omega in radians per second
	@AutoLogOutput(key = "TeleOpInput/Driver/OmegaRadPerSec")
	public double getOmega() {
		//No effect in autonomous mode
		if(RobotState.isAutonomous()) return 0.0;

		//Deadbanding
		double omega = applyDeadbanding(yAngleSupplier.getAsDouble());

		//Square omega to match driving
		double omegaSquared = Math.copySign(Math.pow(omega, 2), omega);

		//Return omega accounting for max angular speed
		return omegaSquared * Constants.DRIVETRAIN_CONFIG.maxAngularVelocity();
	}
}