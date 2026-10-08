package first.robot.mechs.swerve;

import org.wpilib.command3.Mechanism;

public class Drivetrain implements Mechanism {
    private SwerveModule[] modules;

    public Drivetrain(SwerveModule[] _modules) {
        modules = _modules;
    }


}