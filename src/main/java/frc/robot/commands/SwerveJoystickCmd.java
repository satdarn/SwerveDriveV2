package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DrivingConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;


public class SwerveJoystickCmd extends CommandBase {

  private final SwerveSubsystem swerveSubsystem;
  private final Supplier<Double> xSpeedFunc, ySpeedFunc, turningSpeedFunc;
  private final SlewRateLimiter xSpeedLimiter, ySpeedLimiter, turningSpeedLimiter; 

  public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem, Supplier<Double> xSpeedFunc, Supplier<Double> ySpeedFunc, Supplier<Double> turningSpeedFunc) {
    this.swerveSubsystem = swerveSubsystem;
    this.xSpeedFunc = xSpeedFunc;
    this.ySpeedFunc = ySpeedFunc;
    this.turningSpeedFunc = turningSpeedFunc;
    this.xSpeedLimiter = new SlewRateLimiter(DrivingConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    this.ySpeedLimiter = new SlewRateLimiter(DrivingConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    this.turningSpeedLimiter = new SlewRateLimiter(DrivingConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond);
    addRequirements(swerveSubsystem);

  }


  @Override
  public void initialize() {

  }


  @Override
  public void execute() {
        // Gets the speeds from the joystick inputs
    double xSpeed = xSpeedFunc.get();
    double ySpeed = ySpeedFunc.get();
    double turningSpeed = turningSpeedFunc.get();

    // Applies Deadzones to the joystick inputs
    xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
    turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;
     
    // Smoothing the joystick inputs
    xSpeed = xSpeedLimiter.calculate(xSpeed) * DrivingConstants.kTeleDriveMaxSpeedMetersPerSecond;
    ySpeed = ySpeedLimiter.calculate(ySpeed) * DrivingConstants.kTeleDriveMaxSpeedMetersPerSecond;
    turningSpeed = turningSpeedLimiter.calculate(turningSpeed) *  DrivingConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;


    ChassisSpeeds chassisSpeeds;
    chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);

    SwerveModuleState[] moduleStates = DrivingConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    swerveSubsystem.setModuleStates(moduleStates);
  }


  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
  }


  @Override
  public boolean isFinished() {
    return false;
  }
}
