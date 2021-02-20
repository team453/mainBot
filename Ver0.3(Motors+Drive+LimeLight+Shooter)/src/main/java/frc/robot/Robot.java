package frc.robot;
//**Motor Imports**
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.*;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
//**Joystick Imports**
import edu.wpi.first.wpilibj.Joystick;
//**Auton Imports???**
import edu.wpi.first.wpilibj.TimedRobot;
//**Drive Imports**
import edu.wpi.first.wpilibj.drive.MecanumDrive;
//**LimeLight Imports**
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;


public class Robot extends TimedRobot {
  //**Robot Motors Declarations + Configurations**
  private static final int kFrontLeftChannel = 13;
  private static final int kRearLeftChannel = 11;
  private static final int kFrontRightChannel = 14;
  private static final int kRearRightChannel = 12;
  //Note Shooter + Intake Values Placeholders
  private static final int kShooterMotorChannel = 1;
  private static final int kIntakeMotorChannel = 4;

  WPI_TalonFX frontLeft = new WPI_TalonFX(kFrontLeftChannel);
  WPI_TalonFX rearLeft = new WPI_TalonFX(kRearLeftChannel);
  WPI_TalonFX frontRight = new WPI_TalonFX(kFrontRightChannel);
  WPI_TalonFX rearRight = new WPI_TalonFX(kRearRightChannel);
  TalonFX shooterMotor = new TalonFX(kShooterMotorChannel);
  TalonSRX intakeMotor = new TalonSRX(kIntakeMotorChannel);
  TalonSRX indexerMotor = new TalonSRX(2);


  //**Joystick + Drive Declaration**
  private static final int kJoystickChannel = 0;
  private static final int kJoystickChannelOp = 1;
  private MecanumDrive m_robotDrive;
  private Joystick m_stick;
  private Joystick m_stick2;

  /* String for output */
  StringBuilder _sb = new StringBuilder();
    
  /* Loop tracker for prints */
  int _loops = 0;


  @Override
  public void robotInit() {
    //set factory defaults
    frontLeft.configFactoryDefault();
    rearLeft.configFactoryDefault();
    frontRight.configFactoryDefault();
    rearRight.configFactoryDefault();

    CameraServer.getInstance().startAutomaticCapture();
    CvSink cvSink = CameraServer.getInstance().getVideo();
    CvSource outputStream = CameraServer.getInstance().putVideo("blur", 640, 480);

    shooterMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

    shooterMotor.configNominalOutputForward(0, Constants.kTimeoutMs);
		shooterMotor.configNominalOutputReverse(0, Constants.kTimeoutMs);
		shooterMotor.configPeakOutputForward(1, Constants.kTimeoutMs);
    shooterMotor.configPeakOutputReverse(-1, Constants.kTimeoutMs);

    shooterMotor.config_kF(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kF, Constants.kTimeoutMs);
		shooterMotor.config_kP(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kP, Constants.kTimeoutMs);
		shooterMotor.config_kI(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kI, Constants.kTimeoutMs);
		shooterMotor.config_kD(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kD, Constants.kTimeoutMs);
    

    //**Declaring Drive Style + Joystick**
    m_robotDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);
    m_stick = new Joystick(1);
    m_stick2 = new Joystick(2);
  }


  @Override
  public void teleopPeriodic() {
    //**Setting Drive Speed**
    //m_robotDrive.driveCartesian(-.25*m_stick.getX(), .25*m_stick.getY(), -.25*m_stick.getZ(), 0.0);

    //**LimeLight Pathfinding + Correction**
    //**Limelight Declarations + Configurations

    /* float KpAim = -0.1f;
    float KpDistance = -0.1f;
    float min_aim_command = 0.05f;
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    //**Reading Values**

    double headingError = -tx.getDouble(0.0);
    double distanceError = -ty.getDouble(0.0);
    double steering_adjust = 0.0f;
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0); */

    //**Smart Dashboard**
    /* SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
    if (m_stick.getRawButton(4)){
      if (x > 1.0){
        steering_adjust = KpAim*headingError - min_aim_command;
      }
      else if (x < 1.0){
        steering_adjust = KpAim*headingError + min_aim_command;
      }
      double distance_adjust = KpDistance * distanceError;
      double z = 0;
      z += steering_adjust;
      m_robotDrive.driveCartesian(.10*m_stick.getX(), -.10* distance_adjust, .10*z, 0.0);
    }
    else{ */
      if(m_stick2.getRawButton(7)){
        m_robotDrive.driveCartesian(-.3*m_stick.getRawAxis(0), .3*m_stick.getRawAxis(1), .3*m_stick.getRawAxis(4), 0.0);
        intakeMotor.set(ControlMode.PercentOutput, .9);
      }
      else{
        if(m_stick2.getRawButton(3)){
          intakeMotor.set(ControlMode.PercentOutput, .9);
          
        }
       else if (m_stick2.getRawButton(4)){
         intakeMotor.set(ControlMode.PercentOutput, -0.9);
     }
     else{
       intakeMotor.set(ControlMode.PercentOutput, 0.0);
         
     }
        m_robotDrive.driveCartesian(.3*m_stick.getRawAxis(0), -.3*m_stick.getRawAxis(1), .3*m_stick.getRawAxis(4), 0.0);
      }
      
   // }

    //**Shooter motor if/else set**
    if (m_stick2.getRawButton(5)) {

      double targetVelocity_UnitsPer100ms = -18000;
			/* 500 RPM in either direction */
			shooterMotor.set(TalonFXControlMode.Velocity, targetVelocity_UnitsPer100ms);

			/* Append more signals to print when in speed mode. */
			_sb.append("\terr:");
			_sb.append(shooterMotor.getClosedLoopError(Constants.kPIDLoopIdx));
			_sb.append("\ttrg:");
			_sb.append(targetVelocity_UnitsPer100ms);

      /* Prepare line to print */

		_sb.append("\tspd:");
		_sb.append(shooterMotor.getSelectedSensorVelocity(Constants.kPIDLoopIdx));
    _sb.append("u"); 	// Native units
    
    if(shooterMotor.getSelectedSensorVelocity(Constants.kPIDLoopIdx) < targetVelocity_UnitsPer100ms +200 && shooterMotor.getSelectedSensorVelocity(Constants.kPIDLoopIdx) > targetVelocity_UnitsPer100ms -200){
      indexerMotor.set(ControlMode.PercentOutput, -.99);
    }
    }
    else{
      shooterMotor.set(ControlMode.PercentOutput, 0);
      indexerMotor.set(ControlMode.PercentOutput, 0);
    }
      /* Print built string every 10 loops */
		if (++_loops >= 20) {
			_loops = 0;
			System.out.println(_sb.toString());
        }
        /* Reset built string */
		_sb.setLength(0);
  
}
}