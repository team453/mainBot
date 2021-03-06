// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class DriveForwardStraight extends CommandBase {
  private final DriveTrain driveTrain;
  private boolean finish = false;
  Timer timer;

  /** Creates a new DriveWithJoysticks. */
  public DriveForwardStraight(DriveTrain dt) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveTrain = dt;
    addRequirements(driveTrain);
    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    while(timer.get() < 2){
      driveTrain.driveForwardStraight(100, Constants.SPEED_FB, Constants.SPEED_TURN);
    }
    finish = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}
