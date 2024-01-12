package frc.robot.subsystems;

import frc.robot.Constants.DrivingConstants;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule(
        "frontLeft", 
        DrivingConstants.kFrontLeftDriveMotorCANID, 
        DrivingConstants.kFrontLeftTurnMotorCANID,
        DrivingConstants.kFrontLeftDrivingEncoderReversed,
        DrivingConstants.kFrontLeftTurningEncoderReversed        
        );
    private final SwerveModule frontRight = new SwerveModule(
        "frontRight",
        DrivingConstants.kFrontRightDriveMotorCANID,
        DrivingConstants.kFrontRightTurnMotorCANID,
        DrivingConstants.kFrontRightDrivingEncoderReversed,
        DrivingConstants.kFrontRightTurningEncoderReversed
    );

    private final SwerveModule backLeft = new SwerveModule(
        "backLeft",
        DrivingConstants.kBackLeftDriveMotorCANID,
        DrivingConstants.kBackLeftTurnMotorCANID,
        DrivingConstants.kBackLeftDrivingEncoderReversed,
        DrivingConstants.kBackLeftTurningEncoderReversed
    );

    private final SwerveModule backRight = new SwerveModule(
        "backRight",
        DrivingConstants.kBackRightDriveMotorCANID,
        DrivingConstants.kBackRightTurnMotorCANID,
        DrivingConstants.kBackRightDrivingEncoderReversed,
        DrivingConstants.kBackRightTurningEncoderReversed
    );

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DrivingConstants.kPhysicalMaxSpeedMetersPerSec);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }
}
