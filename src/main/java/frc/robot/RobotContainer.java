// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
//import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
//import edu.wpi.first.math.trajectory.TrajectoryUtil;
//import edu.wpi.first.wpilibj.DriverStation;
//import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
//import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Commands.*;
import frc.robot.Commands.AutoCommands.*;
//import frc.robot.Commands.CommandGroups.*;
import frc.robot.Subsystems.*;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
  
public class RobotContainer 
{
  public final ArmSubsystem  armSubsystem = new ArmSubsystem();
  public final OuttakeSubsystem outtakeSubsystem = new OuttakeSubsystem();
  public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  public final ActuatorLeftSubsystem actuatorLeftSubsystem  = new  ActuatorLeftSubsystem ();
  public final ActuatorRightSubsystem actuatorRightSubsystem  = new  ActuatorRightSubsystem ();
  public final DriveSubsystem driveSubsystem = new DriveSubsystem();
  /*
  public final OuttakeStart outtakeStart = new OuttakeStart(outtakeSubsystem);
  public final OuttakeStop outtakeStop = new OuttakeStop(outtakeSubsystem);
  public final ArmUp armUp = new ArmUp(armSubsystem);
  public final ArmDown armDown = new ArmDown(armSubsystem);
  public final IntakeStart intakeStart = new IntakeStart(intakeSubsystem);
  public final IntakeStop intakeStop  = new IntakeStop(intakeSubsystem);
  public final ArmStop armStop = new ArmStop(armSubsystem);
  */

  public LimelightSubsystem limelight = new LimelightSubsystem(); // instantiate
  //public static PoseEstimator poseEstimate;


  public final static Joystick gamepad = new Joystick(OIConstants.DriverGamepadPort);
  public final static Joystick joystickL = new Joystick(OIConstants.DriverJoystickLPort);
  public final static Joystick joystickR = new Joystick(OIConstants.DriverJoystickRPort);
  
  /* 
  public final DutyCycleEncoder dutyCycleEncoder;
  public static double currentArmPostion;
  public static double absoluteArmPostion;
  public static double armPostionOffset;
  public static double armDistance;
  */

  // A chooser for autonomous commands
  public SendableChooser<Command> autoChooser;

  public RobotContainer() 
  {
    /* dutyCycleEncoder = new DutyCycleEncoder(0);
    absoluteArmPostion = dutyCycleEncoder.getAbsolutePosition();
    currentArmPostion = dutyCycleEncoder.get(); 
    armPostionOffset = dutyCycleEncoder.getPositionOffset();
    armDistance = dutyCycleEncoder.getDistance();
    */                                                                      
    configureAutoCommands();

     // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();

    // Put the chooser on the dashboard
    SmartDashboard.putData("Auto Chooser", autoChooser);

    driveSubsystem.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> driveSubsystem.drive(
                 MathUtil.applyDeadband(joystickR.getRawAxis(1), OIConstants.kDriveDeadband),
                 MathUtil.applyDeadband(joystickR.getRawAxis(0), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(joystickL.getRawAxis(0), OIConstants.kDriveDeadband),
                false, true),
            driveSubsystem));
    configureBindings();
  }

  private void configureAutoCommands()
  {
    NamedCommands.registerCommand("OuttakeStart", new OuttakeStart(outtakeSubsystem));
    NamedCommands.registerCommand("OuttakeStop", new OuttakeStop(outtakeSubsystem));
    NamedCommands.registerCommand("ArmUp", new ArmUp(armSubsystem));
    NamedCommands.registerCommand("ArmDown", new ArmDown(armSubsystem));
    NamedCommands.registerCommand("ArmStop", new ArmStop(armSubsystem));
    NamedCommands.registerCommand("IntakeStart", new IntakeStart(intakeSubsystem));
    NamedCommands.registerCommand("IntakeStop", new IntakeStop(intakeSubsystem));
  }

  private void configureBindings() 
  {
    new JoystickButton(gamepad, OIConstants.ArmDownButtonIdx).whileTrue(new ArmDownCommand(armSubsystem));
    new JoystickButton(gamepad, OIConstants.ArmUpButtonIdx).whileTrue(new ArmUpCommand(armSubsystem));
    new JoystickButton(joystickR, OIConstants.OuttakeButtonIdx).whileTrue(new OuttakeCommand(outtakeSubsystem));
    new JoystickButton(joystickL, OIConstants.IntakeOneButtonIdx).whileTrue(new IntakeOneCommand(intakeSubsystem));
    new JoystickButton(joystickL, OIConstants.IntakeTwoButtonIdx).whileTrue(new IntakeTwoCommand(intakeSubsystem));
    new JoystickButton(gamepad, OIConstants.ActuatorLeftUpButtonIdx).whileTrue(new ActuatorLeftUpCommand(actuatorLeftSubsystem));
    new JoystickButton(gamepad, OIConstants.ActuatorLeftDownButtonIdx).whileTrue(new ActuatorLeftDownCommand(actuatorLeftSubsystem));
    new JoystickButton(gamepad, OIConstants.ActuatorLeftDownFixButtonIdx).whileTrue(new ActuatorLeftDownCommandFix(actuatorLeftSubsystem));
    new JoystickButton(gamepad, OIConstants.ActuatorRightUpButtonIdx).whileTrue(new ActuatorRightUpCommand(actuatorRightSubsystem));
    new JoystickButton(gamepad, OIConstants.ActuatorRightDownButtonIdx).whileTrue(new ActuatorRightDownCommand(actuatorRightSubsystem));
    new JoystickButton(gamepad, OIConstants.ActuatorRightDownFixButtonIdx).whileTrue(new ActuatorRightDownCommandFix(actuatorRightSubsystem));
    new JoystickButton(gamepad,OIConstants.SetXButtonIdx)
        .whileTrue(new RunCommand(
            () -> driveSubsystem.setX(),
            driveSubsystem));
    new JoystickButton(gamepad,OIConstants.ResetDriveButtonIdx)
        .whileTrue(new RunCommand(
            () -> driveSubsystem.ResetDrive(),
            driveSubsystem));
  }

  public Command getAutonomousCommand() 
  {
    return autoChooser.getSelected();
  }

  public Command testCommand()
  {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    edu.wpi.first.math.trajectory.Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        driveSubsystem::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        driveSubsystem::setModuleStates,
        driveSubsystem);

    // Reset odometry to the starting pose of the trajectory.
    driveSubsystem.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> driveSubsystem.drive(0, 0, 0, false, false));
  }

  public Command doNothing()
  {
    return null;
  }

  public Command DineNDash()
    {
        outtakeSubsystem.Spin(0.5, 0.5);
        new WaitCommand(0.2);
        intakeSubsystem.Spin(0.5);
        new WaitCommand(0);
        outtakeSubsystem.Spin(0, 0);
        intakeSubsystem.Spin(0);  
        new WaitCommand(0.3);
        armSubsystem.Spin(-0.3, 0.3);
        new WaitCommand(0.2);
        armSubsystem.Spin(0, 0);
        new WaitCommand(0.3);
          // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    edu.wpi.first.math.trajectory.Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(4, 0)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        driveSubsystem::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        driveSubsystem::setModuleStates,
        driveSubsystem);

    // Reset odometry to the starting pose of the trajectory.
    driveSubsystem.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> driveSubsystem.drive(0, 0, 0, false, false));
    }
}