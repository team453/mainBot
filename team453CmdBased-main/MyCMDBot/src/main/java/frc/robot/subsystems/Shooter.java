// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  WPI_TalonFX shooter;
  /** Creates a new Shooter. */
  public Shooter() {
    shooter = new WPI_TalonFX(Constants.SHOOTER_ID);
    shooter.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

    shooter.configNominalOutputForward(0, Constants.kTimeoutMs);
		shooter.configNominalOutputReverse(0, Constants.kTimeoutMs);
		shooter.configPeakOutputForward(1, Constants.kTimeoutMs);
    shooter.configPeakOutputReverse(-1, Constants.kTimeoutMs);

    shooter.config_kF(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kF, Constants.kTimeoutMs);
		shooter.config_kP(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kP, Constants.kTimeoutMs);
		shooter.config_kI(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kI, Constants.kTimeoutMs);
		shooter.config_kD(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kD, Constants.kTimeoutMs);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void shootSimple(double speed, XboxController driver){
    shooter.set(ControlMode.PercentOutput, speed * driver.getTriggerAxis(Hand.kRight));
  }

  public void autoShooter(double speed){
    shooter.set(ControlMode.PercentOutput, speed);
  }

  public void stop(){
    shooter.set(ControlMode.PercentOutput, 0.0);
  }
}
