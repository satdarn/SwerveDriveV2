package frc.robot.subsystems;

import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.Constants.DrivingConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {
    private static String moduleID;

    private final CANSparkMax turnMotor;
    private final CANSparkMax driveMotor;

    private final RelativeEncoder turningEncoder;
    private final RelativeEncoder drivingEncoder;

    private final PIDController turningPidController;


    public SwerveModule(String ID, int driveMotorCANId, int turningMotorCANId, boolean driveMotorReversed, boolean turningMotorReversed) {
        
        moduleID = ID;

        driveMotor = new CANSparkMax(driveMotorCANId, MotorType.kBrushed);
        turnMotor = new CANSparkMax(turningMotorCANId, MotorType.kBrushed);

        turnMotor.setInverted(turningMotorReversed);
        driveMotor.setInverted(driveMotorReversed);

        drivingEncoder = driveMotor.getEncoder(SparkRelativeEncoder.Type.kQuadrature, 4096);
        turningEncoder = turnMotor.getEncoder(SparkRelativeEncoder.Type.kQuadrature, 4096);

        drivingEncoder.setPositionConversionFactor(SwerveModuleConstants.kDriveEncoderRot2Meters);
        drivingEncoder.setVelocityConversionFactor(SwerveModuleConstants.kDriveEncoderRPM2MeterPerSec);
        turningEncoder.setPositionConversionFactor(SwerveModuleConstants.kTurningEncoderRot2Rad);
        turningEncoder.setVelocityConversionFactor(SwerveModuleConstants.kTurningEncoderRPM2RadPerSec);

        turningPidController = new PIDController(SwerveModuleConstants.kPTurning, 0,0);
        turningPidController.enableContinuousInput(-Math.PI,Math.PI);
    }
    public double getDrivePosition() {
        return drivingEncoder.getPosition();
    }
    public double getTurningPosition() {
        return turningEncoder.getPosition();
    }   

    public double getDriveVelocity() {
        return drivingEncoder.getVelocity();
    }

    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle); 
        driveMotor.set(state.speedMetersPerSecond / DrivingConstants.kPhysicalMaxSpeedMetersPerSec);
        turnMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));

        SmartDashboard.putString("Swerve[" + moduleID + "] state", state.toString());
    }
    public void stop(){
        driveMotor.set(0);
        turnMotor.set(0);
    }
}
