// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Drive;
import frc.robot.subsystems.Swerve.SwerveModule;


public class DriveSubsystem extends SubsystemBase {

  private final SwerveModule m_frontLeftModule;
  private final SwerveModule m_frontRightModule;
  private final SwerveModule m_backLeftModule;
  private final SwerveModule m_backRightModule;
  private final SwerveDriveKinematics m_kinematics;
  private final AHRS m_navX;
  private final SwerveModulePosition[] m_modulePositions;
  private final SwerveDriveOdometry m_odometry;


  public DriveSubsystem() {
    this.m_frontLeftModule = new SwerveModule(
      Drive.Motors.kFrontLeftDriveFalconCANID, 
      Drive.Motors.kFrontLeftSteerFalconCANID, 
      Drive.Encoders.kFrontLeftSteerEncoderCANID, 
      Drive.Stats.kFrontLeftModuleOffsetInDegrees
    );
    this.m_frontRightModule = new SwerveModule(
      Drive.Motors.kFrontRightDriveFalconCANID, 
      Drive.Motors.kFrontRightSteerFalconCANID, 
      Drive.Encoders.kFrontRightSteerEncoderCANID,
      Drive.Stats.kFrontRightModuleOffsetInDegrees
      );
    this.m_backLeftModule = new SwerveModule(
      Drive.Motors.kBackLeftDriveFalconCANID, 
      Drive.Motors.kBackLeftSteerFalconCANID, 
      Drive.Encoders.kBackLeftSteerEncoderCANID,
      Drive.Stats.kBackLeftModuleOffsetInDegrees
    );
    this.m_backRightModule = new SwerveModule(
      Drive.Motors.kBackRightDriveFalconCANID, 
      Drive.Motors.kBackRightSteerFalconCANID, 
      Drive.Encoders.kBackRightSteerEncoderCANID,
      Drive.Stats.kBackRightModuleOffsetInDegrees
    );

    m_kinematics = new SwerveDriveKinematics(
            new Translation2d(Drive.Stats.kTrackWidthMeters / 2.0, Drive.Stats.kWheelbaseMeters / 2.0),
            new Translation2d(Drive.Stats.kTrackWidthMeters / 2.0, -Drive.Stats.kWheelbaseMeters / 2.0),
            new Translation2d(-Drive.Stats.kTrackWidthMeters / 2.0, Drive.Stats.kWheelbaseMeters / 2.0),
            new Translation2d(-Drive.Stats.kTrackWidthMeters / 2.0, -Drive.Stats.kWheelbaseMeters / 2.0)
    );

    m_navX = new AHRS();

    m_modulePositions = new SwerveModulePosition[]{
      new SwerveModulePosition(m_frontLeftModule.getDriveMotor().getPosition().asSupplier().get(), m_frontLeftModule.getModuleState().angle), 
      new SwerveModulePosition(m_frontRightModule.getDriveMotor().getPosition().asSupplier().get(), m_frontRightModule.getModuleState().angle),
      new SwerveModulePosition(m_backLeftModule.getDriveMotor().getPosition().asSupplier().get(), m_backLeftModule.getModuleState().angle), 
      new SwerveModulePosition(m_backRightModule.getDriveMotor().getPosition().asSupplier().get(), m_backRightModule.getModuleState().angle)
    };

    m_odometry = new SwerveDriveOdometry(m_kinematics, Rotation2d.fromDegrees(Double.valueOf(m_navX.getFusedHeading())), m_modulePositions);
  }


  @Override
  public void periodic()
   {
    m_odometry.update(Rotation2d.fromDegrees(Double.valueOf(m_navX.getFusedHeading())), new SwerveModulePosition[]{
      new SwerveModulePosition(m_frontLeftModule.getDriveMotor().getPosition().asSupplier().get(), m_frontLeftModule.getModuleState().angle), 
      new SwerveModulePosition(m_frontRightModule.getDriveMotor().getPosition().asSupplier().get(), m_frontRightModule.getModuleState().angle),
      new SwerveModulePosition(m_backLeftModule.getDriveMotor().getPosition().asSupplier().get(), m_backLeftModule.getModuleState().angle), 
      new SwerveModulePosition(m_backRightModule.getDriveMotor().getPosition().asSupplier().get(), m_backRightModule.getModuleState().angle)
    }
    );
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
