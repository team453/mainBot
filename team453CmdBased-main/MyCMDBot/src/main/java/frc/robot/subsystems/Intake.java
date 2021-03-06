// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  WPI_TalonSRX intake;

  /** Creates a new Intake. */
  public Intake() {
    intake = new WPI_TalonSRX(Constants.INTAKE_ID);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void intakeBall(double speed){
    intake.set(ControlMode.PercentOutput, speed);
  }

  public void outtakeBall(double speed){
    intake.set(ControlMode.PercentOutput, -speed);
  }

  public void stop(){
    intake.set(ControlMode.PercentOutput, 0.0);
  }
}
