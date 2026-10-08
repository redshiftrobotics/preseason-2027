package first.robot.mechs.swerve;

import org.wpilib.command3.Mechanism;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;

public class Drivetrain implements Mechanism {
    //Swerve modules (order: FL, FR, BL, BR)
    private SwerveModule[] modules;

    //Gyro
    private GyroIO gio;
    private GyroIOInputsAutoLogged ginputs;

    //Robot state
    private Pose2d pose = new Pose2d();
    private ChassisVelocities measuredVelocities = new ChassisVelocities();
    private ChassisVelocities targetVelocities = new ChassisVelocities();

    public Drivetrain(SwerveModule[] _modules) {
        modules = _modules;
    }


}