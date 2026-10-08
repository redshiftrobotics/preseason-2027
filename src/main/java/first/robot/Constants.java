package first.robot;

import org.wpilib.math.util.Units;

public class Constants {
    private Constants() {}

    /** The period, in seconds, of the main robot loop */
    public static final double LOOP_PERIOD_SECONDS = Robot.defaultPeriodSecs; // 0.02

    /** The port ID of the driver's controller */
    public static final int DRIVER_CONTROLLER_PORT_ID = 0;

    /** Drivetrain PID gains */
    public static final double MODULE_DRIVE_PID_KP = 0;
    public static final double MODULE_DRIVE_PID_KI = 0;
    public static final double MODULE_DRIVE_PID_KD = 0;
    public static final double MODULE_TURN_PID_KP = 0;
    public static final double MODULE_TURN_PID_KI = 0;
    public static final double MODULE_TURN_PID_KD = 0;

    /** Drivetrain feedforward gains */
    public static final double MODULE_DRIVE_FFWD_KS = 0;
    public static final double MODULE_DRIVE_FFWD_KV = 0;
    public static final double MODULE_DRIVE_FFWD_KA = 0;

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

	// Reductions
	public static final double DRIVE_REDUCTION = Mk5nReductions.L3.reduction;
	public static final double TURN_REDUCTION = Mk5nReductions.TURN;
}
