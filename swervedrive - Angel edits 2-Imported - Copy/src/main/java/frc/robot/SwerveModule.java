// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
//import com.ctre.phoenix.sensors.AbsoluteSensorRange;
//import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.constraint.MaxVelocityConstraint;

public class SwerveModule {
  private static final double kWheelRadius = 0.0508;
  private static final int kEncoderResolution = 4096;
  private static final double kDriveGearRatio = 11/1;
  private static final double kNEO_RPM = 5676;
  private static final double kMaxDriveSpeed = kNEO_RPM / 60 * kWheelRadius * 2 * kDriveGearRatio * Math.PI;
  private static final double kTickScaler = 2 * kWheelRadius * Math.PI / (kEncoderResolution * kDriveGearRatio);
  private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
  private static final double kModuleMaxAngularAcceleration =
      2 * Math.PI; // radians per second squared

  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;

  private final RelativeEncoder m_driveEncoder;
  private final CANcoder m_turningEncoder;

  // Gains are for example purposes only - must be determined for your own robot!
  private final PIDController m_drivePIDController = new PIDController(0.1, 0.00, 0);

  // Gains are for example purposes only - must be determined for your own robot!
  private final PIDController m_turningPIDController =
      //new PIDController(0.00021,0.0000001,0.00000005);
      //new PIDController(0.00025,0.00253,-0.00001);
      //new PIDController(0.0004,0.001,-0.00001);
      //new PIDController(0.0005,0.00001,0.00000);
          //0.00005,
          //0.00005,
          //-0.00005);

        //new PIDController(0.009975,0,0.0001);
          new PIDController(0.0049875,0.0,0.0001);


          
      

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(0, 0);
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(0, 0);

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
   *
   * @param driveMotorChannel PWM output for the drive motor.
   * @param turningMotorChannel PWM output for the turning motor.
   * @param driveEncoderChannelA DIO input for the drive encoder channel A
   * @param driveEncoderChannelB DIO input for the drive encoder channel B
   * @param turningEncoderChannelA DIO input for the turning encoder channel A
   * @param turningEncoderChannelB DIO input for the turning encoder channel B
   * //@offset offset
   */
  public SwerveModule(
      
  int driveMotorChannel,
      int turningMotorChannel,
      int cancoderID, double offset) {
    m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);

    m_driveEncoder = m_driveMotor.getEncoder();
    m_turningEncoder = new CANcoder(cancoderID);
    //m_turningEncoder.configAbsoluteSensorRange(AbsoluteSensorRangeValue.Signed_PlusMinusHalf);

    MagnetSensorConfigs magnetconfig = new MagnetSensorConfigs().withAbsoluteSensorRange(AbsoluteSensorRangeValue.Signed_PlusMinusHalf);
    m_turningEncoder.getConfigurator().apply(magnetconfig);    

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.

    // Set the distance (in this case, angle) in radians per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * pi) divided by the
    // encoder resolution.
    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-180, 180);
    m_turningPIDController.setTolerance(0.1);
    this.offset = offset;
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        (m_driveEncoder.getVelocity()), new Rotation2d(Math.toRadians(m_turningEncoder.getAbsolutePosition().getValueAsDouble())));

  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
       
    m_driveEncoder.getPosition(), new Rotation2d((Math.toRadians(m_turningEncoder.getAbsolutePosition().getValueAsDouble()))));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState,  new Rotation2d(Math.toRadians(m_turningEncoder.getAbsolutePosition().getValueAsDouble())));
    
  m_driveEncoder.setVelocityConversionFactor(kTickScaler);
      
    final double driveOutput =
    m_drivePIDController.calculate(m_driveEncoder.getVelocity(), state.speedMetersPerSecond);

final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    double wantedAngle = ExtraMath.mod(state.angle.getDegrees() + 180.0, 360.0) - 180.0;
    //double wantedAngle = ExtraMath.mod(state.angle.getDegrees()+180.0, 360.0) - 180.0;
    final double turnOutput =
        m_turningPIDController.calculate(m_turningEncoder.getAbsolutePosition().getValueAsDouble(), wantedAngle);
        //m_turningPIDController.calculate(m_turningEncoder.getAbsolutePosition(), state.angle.getDegrees());
        
        //System.out.println("wanted angle: "+ wantedAngle); System.out.println(m_turningEncoder.getAbsolutePosition());
        //System.out.printf("\n %.3f Wanted angle: %.3f current angle:\n", wantedAngle, m_turningEncoder.getAbsolutePosition());
          System.out.println(m_turningPIDController.getPositionError());
    m_driveMotor.set(-driveOutput);
    //System.out.println(driveFeedforward);
    m_turningMotor.set(-turnOutput/2);

      

  }
}
