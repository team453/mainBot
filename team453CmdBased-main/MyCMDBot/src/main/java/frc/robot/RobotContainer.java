// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.AutoShooter;
import frc.robot.commands.DriveForwardStraight;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.IntakeBall;
import frc.robot.commands.OuttakeBall;
import frc.robot.commands.ShootSimple;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final DriveTrain driveTrain;
  private final DriveWithJoysticks driveWithJoysticks;
  private final DriveForwardStraight driveForwardStraight;
  private final Shooter shooter;
  private final ShootSimple shootSimple;
  private final Intake intake;
  private final IntakeBall intakeBall;
  private final OuttakeBall outtakeBall;
  private final AutoShooter autoShooter;
  public static XboxController driverJoystick;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    driveTrain  = new DriveTrain();
    driveWithJoysticks = new DriveWithJoysticks(driveTrain);
    driveWithJoysticks.addRequirements(driveTrain);
    driveTrain.setDefaultCommand(driveWithJoysticks);

    driveForwardStraight = new DriveForwardStraight(driveTrain);
    driveForwardStraight.addRequirements(driveTrain);

    shooter = new Shooter();
    shootSimple = new ShootSimple(shooter);
    shootSimple.addRequirements(shooter);
    autoShooter = new AutoShooter(shooter);
    autoShooter.addRequirements(shooter);
    shooter.setDefaultCommand(shootSimple);

    intake = new Intake();
    intakeBall = new IntakeBall(intake);
    intakeBall.addRequirements(intake);
    outtakeBall = new OuttakeBall(intake);
    outtakeBall.addRequirements(intake);

    driverJoystick = new XboxController(Constants.DRIVER_JOY);

    UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
    camera.setResolution(Constants.CAM_RES_X, Constants.CAM_RES_Y);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    JoystickButton intakeButton = new JoystickButton(driverJoystick, XboxController.Button.kA.value);
    intakeButton.whileHeld(new IntakeBall(intake));
    JoystickButton outtakeButton = new JoystickButton(driverJoystick, XboxController.Button.kB.value);
    outtakeButton.whileHeld(new OuttakeBall(intake));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return driveForwardStraight;
  }
}
