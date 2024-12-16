// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.MuratCont;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;

import java.io.File;
import java.nio.file.Path;
import java.nio.file.Paths;

import org.photonvision.PhotonCamera;

import com.fasterxml.jackson.databind.node.IntNode;

import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LedSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {

  private SlewRateLimiter translationLimiter = new SlewRateLimiter(5.0);
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(5.0);
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(5.0);

  private double speedRate = 0.25;

  // to list auto paths
  // Create a File object for the base directory
  Path currentDir = Paths.get("").toAbsolutePath();

  // Go up two directories
  Path targetDir = currentDir;

  // Combine with your desired path
  File autosFolder = targetDir.resolve("src/main/deploy/pathplanner/autos").toFile();

  File[] listOfAutos = autosFolder.listFiles();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController operator = new CommandXboxController(0);
  final CommandXboxController driverXbox = new CommandXboxController(1);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem s_swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "swerve/neo"));
  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the rotational velocity
  // buttons are quick rotation positions to different ways to face
  // WARNING: default buttons are on the same buttons as the ones defined in
  // configureBindings
  AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(s_swerve,
      () -> -MathUtil.applyDeadband(driverXbox.getLeftY(),
          OperatorConstants.LEFT_Y_DEADBAND),
      () -> -MathUtil.applyDeadband(driverXbox.getLeftX(),
          OperatorConstants.LEFT_X_DEADBAND),
      () -> -MathUtil.applyDeadband(driverXbox.getRightX(),
          OperatorConstants.RIGHT_X_DEADBAND),
      driverXbox.getHID()::getYButtonPressed,
      driverXbox.getHID()::getAButtonPressed,
      driverXbox.getHID()::getXButtonPressed,
      driverXbox.getHID()::getBButtonPressed);

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the desired angle NOT angular rotation
  Command driveFieldOrientedDirectAngle = s_swerve.driveCommand(
      () -> speedRate * MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
      () -> speedRate * MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
      () -> driverXbox.getRightX(),
      () -> driverXbox.getRightY());

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the angular velocity of the robot
  Command driveFieldOrientedAnglularVelocity = s_swerve.driveCommand(
      () -> speedRate * MathUtil.applyDeadband(driverXbox.getLeftY() * -1, OperatorConstants.LEFT_Y_DEADBAND),
      () -> speedRate * MathUtil.applyDeadband(driverXbox.getLeftX() * -1, OperatorConstants.LEFT_X_DEADBAND),
      () -> driverXbox.getRightX() * -1);

  Command driveFieldOrientedDirectAngleSim = s_swerve.simDriveCommand(
      () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
      () -> driverXbox.getRawAxis(2));

  SendableChooser<Command> autoChooser = new SendableChooser<>();
  SendableChooser<Integer> SpeedChooser = new SendableChooser<>();

  private LedSubsystem s_led = new LedSubsystem(s_swerve);
  private Indexer s_Indexer = new Indexer();
  private PhotonCamera camera = new PhotonCamera("Limelight");
  private Intake s_intake = new Intake();


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {


     DriverStation.Alliance color;
     color = DriverStation.getAlliance().get();
     int isBlueAlliance = DriverStation.Alliance.Blue == color ? 1 : -1;

    Command driveFieldOrientedAngularVelocity = s_swerve.driveCommand(
        () -> isBlueAlliance * speedRate
            * translationLimiter
                .calculate(-MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND)),
        () -> isBlueAlliance * speedRate
            * strafeLimiter
                .calculate(-MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND)),
        () -> -MathUtil.applyDeadband(driverXbox.getRightX(), OperatorConstants.RIGHT_X_DEADBAND));

    Command driveFieldOrientedDirectAngleSim = s_swerve.simDriveCommand(
        () -> -MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> -MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -driverXbox.getRightX());

    // Autonomous Chooser (Searchs auto folder)
    autoChooser.setDefaultOption("do nothing", s_swerve.getAutonomousCommand("do nothing"));
    SmartDashboard.putData(autoChooser);
    if (listOfAutos != null) {
      for (int i = 0; i < listOfAutos.length; i++) {
        if (listOfAutos[i].isFile()) {
          String newName = listOfAutos[i].getName().substring(0, listOfAutos[i].getName().length() - 5);
          autoChooser.addOption(newName, s_swerve.getAutonomousCommand(newName));
        }
      }
    }

    // Dashboard Verileri
    SmartDashboard.putNumber("Swerve Speed Rate", speedRate);
    SmartDashboard.putBoolean("Is Back", s_led.getLed());

    // Configure the trigger bindings
    configureBindings();

    // Genel Setup
    s_swerve.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedAngularVelocity : driveFieldOrientedDirectAngleSim);

    // Led Setup
    s_led.setDefaultCommand(s_led.LedCommand(s_swerve,s_Indexer, camera));

  }

  // GENERAL KONFIGIRASYON

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary predicate, or via the
   * named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
   * for
   * {@link CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
   * Flight joysticks}.
   */
  private void configureBindings() {

    // Gyro Sıfırlama
    driverXbox.x().onTrue(s_swerve.zeroGyro().andThen(s_swerve.lock()));
  

    // Swerve Kitleme
    driverXbox.b().whileTrue(
        s_swerve.lock()
        );

    //bilmem
    driverXbox.povUp().whileTrue(s_swerve.aimAtMurat(camera));

    driverXbox.back().whileTrue(new RunCommand(()->{
      s_led.MuratContDegis(true);
    }));
     driverXbox.back().whileFalse(new RunCommand(()->{
      s_led.MuratContDegis(false);
    }));

    driverXbox.rightTrigger().whileTrue(s_intake.RunMotor()).onFalse(s_intake.StopMotor());
    //driverXbox.povLeft().onTrue(s_swerve.turn180());

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }

  public void setDriveMode() {
    configureBindings();
  }

  public void setMotorBrake(boolean brake) {
    s_swerve.setMotorBrake(brake);
  }
}
