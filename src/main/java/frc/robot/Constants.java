package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {
    public static final class SwerveModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRation = 1 / 8.14;
        public static final double kTurnMotorGearRation = 1 / 12.8;
        public static final double kDriveEncoderRot2Meters = kDriveMotorGearRation * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurnMotorGearRation * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meters / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = 0.5;  
    }
    public static final class DrivingConstants {

        public static final double kTrackWidth = Units.inchesToMeters(25);
        public static final double kWheelBase = Units.inchesToMeters(25);
         public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2));


        public static final double kPhysicalMaxSpeedMetersPerSec = 3;

        public static final int kFrontLeftTurnMotorCANID = 0;
        public static final int kFrontLeftDriveMotorCANID = 1;
        public static final boolean kFrontLeftTurningEncoderReversed = true;
        public static final boolean kFrontLeftDrivingEncoderReversed = true;

        public static final int kFrontRightTurnMotorCANID = 3;        
        public static final int kFrontRightDriveMotorCANID = 2;
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kFrontRightDrivingEncoderReversed = true;

        public static final int kBackLeftTurnMotorCANID = 4;
        public static final int kBackLeftDriveMotorCANID = 5;
        public static final boolean kBackLeftTurningEncoderReversed = true;
        public static final boolean kBackLeftDrivingEncoderReversed = true;

        public static final int kBackRightTurnMotorCANID = 6;
        public static final int kBackRightDriveMotorCANID = 7;
        public static final boolean kBackRightTurningEncoderReversed = true;
        public static final boolean kBackRightDrivingEncoderReversed = true;

        public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;

        public static final double kDeadband = 0.05;
    }
}
