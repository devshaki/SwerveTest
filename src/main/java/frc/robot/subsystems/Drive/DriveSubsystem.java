// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Drive;
import frc.robot.subsystems.Swerve.SwerveModule;


public class DriveSubsystem extends SubsystemBase {

  private final SwerveModule m_frontLeftModule;
  private final SwerveModule m_frontRightModule;
  private final SwerveModule m_backLeftModule;
  private final SwerveModule m_backRightModule;

  public DriveSubsystem() {
    this.m_frontLeftModule = new SwerveModule(Drive.Motors.kFrontLeftDriveFalconCANID, Drive.Motors.kFrontLeftSteerFalconCANID, Drive.Encoders.kFrontLeftSteerEncoderCANID);
    this.m_frontRightModule = new SwerveModule(Drive.Motors.kFrontRightDriveFalconCANID, Drive.Motors.kFrontRightSteerFalconCANID, Drive.Encoders.kFrontRightSteerEncoderCANID);
    this.m_backLeftModule = new SwerveModule(Drive.Motors.kBackLeftDriveFalconCANID, Drive.Motors.kBackLeftSteerFalconCANID, Drive.Encoders.kBackLeftSteerEncoderCANID);
    this.m_backRightModule = new SwerveModule(Drive.Motors.kBackRightDriveFalconCANID, Drive.Motors.kBackRightSteerFalconCANID, Drive.Encoders.kBackRightSteerEncoderCANID);

  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
