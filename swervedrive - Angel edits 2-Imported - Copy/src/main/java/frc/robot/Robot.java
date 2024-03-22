package frc.robot;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
//import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.utils.GamepadUtils;

//import java.nio.file.Path;
import java.util.List;


//
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;



public class Robot extends TimedRobot {
  private final XboxController m_controller = new XboxController(0);
  private final Drivetrain m_swerve = new Drivetrain();
  private final Timer timer = new Timer();
  private boolean autoRan = false;
  private boolean armStart = false;
  private boolean armIsUP = false;
  private boolean armIsDOWN = false;
  private boolean hitsTarget = false;
  
  private final ArmSubsystem m_arm = new ArmSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem(); 
  private final LauncherSubsystem m_launcher = new LauncherSubsystem();
  private final ClimberSubsystem m_climber = new ClimberSubsystem();

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  @Override
  public void autonomousPeriodic() {
    //driveWithJoystick(false);
    
     // driveWithJoystick(false);
     m_swerve.updateOdometry();
    
     timer.reset();
     timer.start();
     
     if (timer.get() < 3 ){
      m_intake.runMotor(0.05);
      m_launcher.runLauncher();

     }

     else if (timer.get() < 6){
       m_swerve.driveLeft(-0.5, 0, 0, false);
       m_swerve.driveright(0.5, 0, 0, false );
       //timer.reset();
     }
     else {
       m_swerve.driveLeft(0, 0, 0, false);
       m_swerve.driveright(0, 0, 0, false );
       
     }
     
     }
     // if ( !autoRan ) {
     //   timer.start();
     //   timer.reset();
       
     //   autoRan = false;
     // } 
     // else if ( timer.get() < 10 ){
     //   m_swerve.driveLeft(3, 0, 0, false);
     //    m_swerve.driveright(3, 0, 0, false );
     // }
     // else{
     //   timer.stop();
     //   m_swerve.driveLeft(0, 0, 0, false);
     //   m_swerve.driveright(0, 0, 0, false);
     // }
     
  

  @Override
  public void teleopPeriodic() {
    
    
    if (armIsUP){
      if (hitsTarget == true && m_controller.getLeftTriggerAxis()==0){
      m_arm.runMotor(0.3);

    }

      if (m_arm.getPosition() >= 0 ){

      m_arm.runMotor(0);

    }

     if(m_arm.getPosition() <= -1.05){
      hitsTarget = true;
     }
     else if(m_arm.getPosition() > -1.05){
      hitsTarget = false;
     }

    }

    if(armIsDOWN){

      if (hitsTarget == true && m_controller.getLeftTriggerAxis()==0){
      m_arm.runMotor(0.3);

    }

      if (m_arm.getPosition() >= 1.2){

      m_arm.runMotor(0);

    }

    if(m_arm.getPosition() <= 0.2){
      hitsTarget = true;
     }
     else if(m_arm.getPosition() > 0.2){
      hitsTarget = false;
     }

    }

    System.out.println(m_arm.getPosition());
    
    m_swerve.updateOdometry();
    
    driveWithJoystick(true);
    
    
    
    
    
    
    //rollerWithButtons();
  }

  private void driveWithJoystick(boolean fieldRelative) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    final var xSpeed =
        -m_xspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftY(), 0.02))
            * Drivetrain.kMaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final var ySpeed =
        -m_yspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftX(), 0.02))
            * Drivetrain.kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because w want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final var rot =
        -m_rotLimiter.calculate(MathUtil.applyDeadband(m_controller.getRightX(), 0.02))
            * Drivetrain.kMaxAngularSpeed;

    m_swerve.driveLeft(xSpeed, ySpeed, rot, fieldRelative);
    m_swerve.driveright(-xSpeed, -ySpeed, -rot, fieldRelative);
    RobotContainer();
  }

//private void rollerWithButtons(){ //}
//private final DriveSubsystem m_robotDrive = new DriveSubsystem();



// The driver's controller
//XboxController m_controller = new XboxController(OIConstants.kDriverControllerPort);

public void RobotContainer() {
  // Configure the button bindings
  configureButtonBindings();

  if (m_controller.getXButton() && armStart == false){

    armStart = true;
    armIsUP = true;

  }

  if (m_controller.getBButton() && armStart == false){

    armStart = true;
    armIsDOWN = true;

  }
    
  

  if (armIsUP){

    if (((m_arm.getPosition() <= 0 || m_controller.getRightTriggerAxis() > 0) &&hitsTarget == false) || (hitsTarget == true && m_controller.getLeftTriggerAxis()>0) ){
      m_arm.runMotor((m_controller.getLeftTriggerAxis()) - (m_controller.getRightTriggerAxis()));
    
    }

  }
  
  if (armIsDOWN){

    if (((m_arm.getPosition() <= 1.2 || m_controller.getRightTriggerAxis() > 0)&&hitsTarget == false) || (hitsTarget == true && m_controller.getLeftTriggerAxis()>0) ){
      m_arm.runMotor((m_controller.getLeftTriggerAxis()) - (m_controller.getRightTriggerAxis()));
    
    }

  }

//   if(m_controller.getLeftTriggerAxis() > 0){
//     m_arm.runMotor(0.7);
// }
// if(m_controller.getRightTriggerAxis() > 0){
//   m_arm.runMotor(-0.7);
// }
  if(m_controller.getYButtonReleased()){
    m_launcher.runMotor(0);
}
if(m_controller.getAButtonReleased()){
    m_launcher.runMotor(0);
}

if(m_controller.getRightBumperReleased()){
    m_intake.runMotor(0);
}

if(m_controller.getLeftBumperReleased()){
    m_intake.runMotor(0);
}

// if (m_controller.getXButtonReleased()){
//   m_climber.runMotor(0);
// }

// if (m_controller.getBButtonReleased()){
//   m_climber.runMotor(0);
// }

  if(m_controller.getRightBumper()){
    m_intake.runMotor(0.4);
  }

  else if(m_controller.getLeftBumper()){
    m_intake.runMotor(-0.4);
  }

  if(m_controller.getAButton()){
    m_launcher.runMotor(-1);
  }

  if(m_controller.getYButton()){
    m_launcher.runMotor(0.7);
  }


  if(m_controller.getXButton()){
  m_climber.runMotor(0.7);
}
  
  if(m_controller.getBButton()){
    m_climber.runMotor(-0.7);
  }

  if(m_controller.getBackButton()){
    m_intake.runMotor(0);
    m_climber.runMotor(0);
    m_launcher.runMotor(0);
    
  }

  // set the arm subsystem to run the "runAutomatic" function continuously when no other command
  // is running
 m_arm.setDefaultCommand(new RunCommand(() -> m_arm.runManual(m_controller.getLeftTriggerAxis()), m_arm));
//  m_arm.setDefaultCommand(new RunCommand(() -> m_arm.runAutomatic(), m_arm));
  // set the intake to stop (0 power) when no other command is running
  m_intake.setDefaultCommand(new RunCommand(() -> m_intake.setPower(0.0), m_intake));

  // configure the launcher to stop when no other command is running
  m_launcher.setDefaultCommand(new RunCommand(() -> m_launcher.stopLauncher(), m_launcher));

  m_climber.setDefaultCommand(new RunCommand(() -> m_climber.stopClimber(), m_climber));
}

/**
 * Use this method to define your button->command mappings. Buttons can be created by
 * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
 * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
 * {@link JoystickButton}.
 */
private void configureButtonBindings() {
  // button to put swerve modules in an "x" configuration to hold position
  

  // set up arm preset positions
  new JoystickButton(m_controller, XboxController.Button.kLeftBumper.value)
  .onTrue(new InstantCommand(() -> m_arm.setTargetPosition(Constants.Arm.kScoringPosition)));
new Trigger(
      () ->
          m_controller.getLeftTriggerAxis()
              > Constants.OIConstants.kTriggerButtonThreshold)
  .onTrue(new InstantCommand(() -> m_arm.setTargetPosition(Constants.Arm.kIntakePosition)));
new JoystickButton(m_controller, XboxController.Button.kStart.value)
  .onTrue(new InstantCommand(() -> m_arm.setTargetPosition(Constants.Arm.kHomePosition)));

  // intake controls (run while button is held down, run retract command once when the button is
  // released)
  new Trigger(
          () ->
              m_controller.getRightTriggerAxis()
                  > Constants.OIConstants.kTriggerButtonThreshold)
      .whileTrue(new RunCommand(() -> m_intake.setPower(Constants.Intake.kIntakePower), m_intake))
      .onFalse(m_intake.retract());

  new JoystickButton(m_controller, XboxController.Button.kY.value)
      .whileTrue(new RunCommand(() -> m_intake.setPower(-1.0)));

  // launcher controls (button to pre-spin the launcher and button to launch)
  new JoystickButton(m_controller, XboxController.Button.kRightBumper.value)
      .whileTrue(new RunCommand(() -> m_launcher.runLauncher(), m_launcher));

  new JoystickButton(m_controller, XboxController.Button.kA.value)
      .onTrue(m_intake.feedLauncher(m_launcher));

  new JoystickButton(m_controller, XboxController.Button.kB.value)
  .whileTrue(new RunCommand(() -> m_climber.runClimber(), m_climber));

  new JoystickButton(m_controller, XboxController.Button.kX.value)
     .whileTrue(new RunCommand(() -> m_climber.runClimber(), m_climber));
}

}


/**
 * Use this to pass the autonomous command to the main {@link Robot} class.
 *
 * @return the command to run in autonomous
 */
/*public Command getAutonomousCommand() {
  // Create config for trajectory
  TrajectoryConfig config =
      new TrajectoryConfig(
              AutoConstants.kMaxSpeedMetersPerSecond,
              AutoConstants.kMaxAccelerationMetersPerSecondSquared)
          // Add kinematics to ensure max speed is actually obeyed
          .setKinematics(DriveConstants.kDriveKinematics);

  // An example trajectory to follow. All units in meters.
  Trajectory exampleTrajectory =
      TrajectoryGenerator.generateTrajectory(
          // Start at the origin facing the +X direction
          new Pose2d(0, 0, new Rotation2d(0)),
          // Pass through these two interior waypoints, making an 's' curve path
          List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
          // End 3 meters straight ahead of where we started, facing forward
          new Pose2d(3, 0, new Rotation2d(0)),
          config);

  var thetaController =
      new ProfiledPIDController(
          AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
  thetaController.enableContinuousInput(-Math.PI, Math.PI);

  SwerveControllerCommand swerveControllerCommand =
      new SwerveControllerCommand(
          exampleTrajectory,
          m_robotDrive::getPose, // Functional interface to feed supplier
          DriveConstants.kDriveKinematics,

          // Position controllers
          new PIDController(AutoConstants.kPXController, 0, 0),
          new PIDController(AutoConstants.kPYController, 0, 0),
          thetaController,
          m_robotDrive::setModuleStates,
          m_robotDrive);

  // Reset odometry to the starting pose of the trajectory.
  m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

  // Run path following command, then stop at the end.
  return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
}
}*/







