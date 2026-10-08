package first.robot.mechs.swerve;

import org.littletonrobotics.junction.Logger;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;

import first.robot.Constants;

public class SwerveModule {
    //IO layer
    private SwerveModuleIO io;
    private SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();

    //Properties
    private Translation2d distanceFromRobotCenter;
    private String moduleName;

    //Target stage storage
    private SwerveModuleVelocity targetVelocity;
    
    public SwerveModule(SwerveModuleIO _io, Translation2d _distanceFromRobotCenter, String _moduleName) {
        //Set properties
        io = _io;
        distanceFromRobotCenter = _distanceFromRobotCenter;
        moduleName = _moduleName;

        //Configure motor gains
        io.setDriveMotorPidGains(Constants.MODULE_DRIVE_PID_KP, Constants.MODULE_DRIVE_PID_KI, Constants.MODULE_DRIVE_PID_KD);
        io.setTurnMotorPidGains(Constants.MODULE_TURN_PID_KP, Constants.MODULE_TURN_PID_KI, Constants.MODULE_TURN_PID_KD);
        io.setDriveMotorFFwdGains(Constants.MODULE_DRIVE_FFWD_KS, Constants.MODULE_DRIVE_FFWD_KV, Constants.MODULE_DRIVE_FFWD_KA);

        //Enable default braking
        io.setDriveMotorBrake(true);
        io.setTurnMotorBrake(true);
    }

    public void updateInputs() {
        Logger.processInputs("Drive/SwerveModule" + moduleName, inputs);
        io.updateInputs(inputs);
    }
    
    public Translation2d getDistanceFromRobotCenter() {
        return distanceFromRobotCenter;
    }

    public SwerveModulePosition getCurrentModulePosition() {
        return new SwerveModulePosition(inputs.drivePositionRad * Constants.WHEEL_RADIUS_METERS, inputs.turnAbsPositionRad);
    }

    public SwerveModuleVelocity getMeasuredModuleVelocity() {
        return new SwerveModuleVelocity(inputs.driveVelocityRadPerSec * Constants.WHEEL_RADIUS_METERS, inputs.turnAbsPositionRad);
    }

    public SwerveModuleVelocity getTargetModuleVelocity() {
        return targetVelocity;
    }

    public void setTargetModuleVelocity(SwerveModuleVelocity _targetVelocity) {
        //Optimize the velocity to minimize heading change
        _targetVelocity.optimize(inputs.turnAbsPositionRad);
        
        //Cosine scaling for smoother driving (wow)
        _targetVelocity.cosineScale(inputs.turnAbsPositionRad);

        //Apply
        targetVelocity = _targetVelocity;
        io.setDriveMotorVelocity(targetVelocity.velocity / Constants.WHEEL_RADIUS_METERS);
        io.setTurnMotorPosition(targetVelocity.angle);
    }

    public void stop() {
        io.stop();
    }
}