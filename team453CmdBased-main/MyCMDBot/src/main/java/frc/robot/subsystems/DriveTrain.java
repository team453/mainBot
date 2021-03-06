// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;

public class DriveTrain extends SubsystemBase {
  // Declare motor controllers
  WPI_TalonFX leftFront;
  WPI_TalonFX rightFront;
  WPI_TalonFX leftBack;
  WPI_TalonFX rightBack;

  // Decalre Mecanum Drive object
  MecanumDrive m_Drive;

  // Declare encoders
  Encoder rightSide;
  Encoder leftSide;
  Encoder strafe;

  /* EncoderSim m_leftEncoderSim = new EncoderSim(leftSide);
  EncoderSim m_rightEncoderSim = new EncoderSim(rightSide);
  EncoderSim m_STRAFEEncoderSim = new EncoderSim(strafe); */

  Pose2d m_pose;
  DifferentialDriveOdometry m_odometry;
  PigeonIMU m_gyro;

  /** Creates a new DriveTrain. with default Paramaters */
  public DriveTrain() {
    // Set motor IDs and Initialize objects and set direction
    leftFront = new WPI_TalonFX(Constants.LEFT_FRONT);
    leftFront.setInverted(false);
    rightFront = new WPI_TalonFX(Constants.RIGHT_FRONT);
    rightFront.setInverted(false);
    leftBack = new WPI_TalonFX(Constants.LEFT_BACK);
    leftBack.setInverted(false);
    rightBack = new WPI_TalonFX(Constants.RIGHT_BACK);
    rightBack.setInverted(false);

    // reconfig back to factory default to avoid weird settings glitches
    leftFront.configFactoryDefault();
    leftBack.configFactoryDefault();
    rightFront.configFactoryDefault();
    rightBack.configFactoryDefault();

    // Initialize Mecanum drive with motors in LF, LB, RF, RB order every time. This
    // is IMPORTANT)
    m_Drive = new MecanumDrive(leftFront, leftBack, rightFront, rightBack);

    m_gyro = new PigeonIMU(Constants.PIGEON_ID);

    rightSide = new Encoder(1, 2, false, CounterBase.EncodingType.k4X);
    leftSide = new Encoder(3, 4, false, CounterBase.EncodingType.k4X);
    strafe = new Encoder(5, 6, false, CounterBase.EncodingType.k4X);

    rightSide.setSamplesToAverage(5);
    rightSide.setDistancePerPulse(1.0 / Constants.TICKS_PER_REV * Math.PI * Constants.CASTER_WHEEL_RADIUS);
    rightSide.setMinRate(1.0);

    leftSide.setSamplesToAverage(5);
    leftSide.setDistancePerPulse(1.0 / Constants.TICKS_PER_REV * Math.PI * Constants.CASTER_WHEEL_RADIUS);
    leftSide.setMinRate(1.0);

    strafe.setSamplesToAverage(5);
    strafe.setDistancePerPulse(1.0 / Constants.TICKS_PER_REV * Math.PI * Constants.CASTER_WHEEL_RADIUS);
    strafe.setMinRate(1.0);

    m_gyro.configFactoryDefault();
    PigeonIMU.GeneralStatus genStatus = new PigeonIMU.GeneralStatus();
    m_gyro.getGeneralStatus(genStatus);



    m_odometry = new DifferentialDriveOdometry(new Rotation2d());
  }


  @Override
  public void periodic() {
    double [] ypr = new double[3];
    m_gyro.getYawPitchRoll(ypr);
    System.out.println("Yaw:" + ypr[0]);
    // This method will be called once per scheduler run
    var gyroAngle = Rotation2d.fromDegrees(-ypr[0]);

    // Update the pose
    m_pose = m_odometry.update(gyroAngle, leftSide.getDistance(), rightSide.getDistance());
  }

  //manually drive with joysticks (see matching command available at )
  public void driveWithJoysticks(XboxController driver, double speedFB, double speedStrf, double speedTurn){
    double yDeadband = 0.1;
    double xDeadband = 0.1;
    double zDeadband = 0.1;

    double ySpeed = driver.getRawAxis(1);
    double xSpeed = driver.getRawAxis(0);
    double zSpeed = driver.getRawAxis(4);

    if(ySpeed < yDeadband && ySpeed > -yDeadband){
      ySpeed = 0;
    }

    if(xSpeed < xDeadband && xSpeed > -xDeadband){
      xSpeed = 0;
    }

    if(zSpeed < zDeadband && zSpeed > -zDeadband){
      zSpeed = 0;
    }
    m_Drive.driveCartesian(speedFB * ySpeed, speedStrf * xSpeed, speedTurn * zSpeed, 0.0);
  }

  //simple auto drive straight Y distance (see matching command available at )
  public void driveForwardStraight(double distInchesTarget, double speedFB, double speedTurn){
    m_Drive.driveCartesian(speedFB, 0.0, speedTurn, 0.0);
  }

  //Stops drivetrain motors (see matching command available at )
  public void stop(){
    m_Drive.stopMotor();
  }
}
