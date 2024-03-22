// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;
//import com.ctre.phoenix.sensors.Pigeon2;
import com.revrobotics.*;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain {
  public static final double kMaxSpeed = 4.0; // 3 meters per second
  public static final double kMaxAngularSpeed = (Math.PI)*2; // 1 rotation per second
  double WHEELBASE = 0.55;
  double TRACKWIDTH = 0.555;
  private final Translation2d m_frontLeftLocation = new Translation2d(WHEELBASE/2, TRACKWIDTH/2);
  private final Translation2d m_frontRightLocation = new Translation2d(WHEELBASE/2, -TRACKWIDTH/2);
  private final Translation2d m_backLeftLocation = new Translation2d(-WHEELBASE/2, TRACKWIDTH/2);
  private final Translation2d m_backRightLocation = new Translation2d(-WHEELBASE/2, -TRACKWIDTH/2);

  //Alter IDs as you see fit:
  private final SwerveModule m_frontLeft = new SwerveModule(1, 2, 9, -26.8945313); 
  private final SwerveModule m_frontRight = new SwerveModule(8,7,10, 11.4257813);
  private final SwerveModule m_backLeft = new SwerveModule(3,4,11, -57.39258);
  private final SwerveModule m_backRight = new SwerveModule(6,5,12, 49.6582031 );
  //Alter Pigeon ID accordingly:
  private final Pigeon2 m_gyro = new Pigeon2(13);

  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          m_kinematics,
          
          Rotation2d.fromDegrees((m_gyro.getYaw()).getValueAsDouble()),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
          });

  public Drivetrain() {
    m_gyro.setYaw(0);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void driveLeft(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble()))
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    //m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    //m_backRight.setDesiredState(swerveModuleStates[3]);

  }

  public void driveright(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble()))
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    //m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    //m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);}


  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble()),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
        });
  }
}
