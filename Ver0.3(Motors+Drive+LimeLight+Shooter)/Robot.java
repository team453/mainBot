package frc.robot;
//**Motor Imports**
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.*;
//**Joystick Imports**
import edu.wpi.first.wpilibj.Joystick;
//**Robot Code Framework Import**
import edu.wpi.first.wpilibj.TimedRobot;
//**Drive Imports**
import edu.wpi.first.wpilibj.drive.MecanumDrive;
//**LimeLight Imports**
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


public class Robot extends TimedRobot {
  //**Robot Motors Declarations + Configurations**
  private static final int kFrontLeftChannel = 4;
  private static final int kRearLeftChannel = 3;
  private static final int kFrontRightChannel = 2;
  private static final int kRearRightChannel = 1;
  //Note Shooter + Intake Values Placeholders
  private static final int kShooterChannel = 5;
  private static final int kIntakeChannel = 6;
  WPI_TalonFX frontLeft = new WPI_TalonFX(kFrontLeftChannel);
  WPI_TalonFX rearLeft = new WPI_TalonFX(kRearLeftChannel);
  WPI_TalonFX frontRight = new WPI_TalonFX(kFrontRightChannel);
  WPI_TalonFX rearRight = new WPI_TalonFX(kRearRightChannel);
  WPI_TalonFX shooter = new WPI_TalonFX(kShooterChannel);
  WPI_TalonSRX intake = new WPI_TalonSRX(kIntakeChannel);

  //**Joystick + Drive Declaration**
  private static final int kJoystickChannel = 0;
  private MecanumDrive m_robotDrive;
  private Joystick m_stick;


  @Override
  public void robotInit() {
    //**Optional Motor Inversions**
    frontRight.setInverted(true);
    rearRight.setInverted(true);

    //**Drive + Joystick Configuration**
    m_robotDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);
    m_stick = new Joystick(kJoystickChannel);
  }


  @Override
  public void teleopPeriodic() {
    //**Setting Teleoperation Drive Speed**
    m_robotDrive.driveCartesian(-.25*m_stick.getX(), .25*m_stick.getY(), -.25*m_stick.getZ(), 0.0);

    //**LimeLight Code**
    //**Limelight Declarations + Configurations**
    float KpAim = -0.1f;
    float KpDistance = -0.1f;
    float min_aim_command = 0.05f;
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    //**Reading LimeLight Values**
    double headingError = -tx.getDouble(0.0);
    double distanceError = -ty.getDouble(0.0);
    double steering_adjust = 0.0f;
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
    //**Smart Dashboard Display**
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
    //**LimeLight Tracking Engaging When Y-Button Pushed
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
    else{
      m_robotDrive.driveCartesian(.35*m_stick.getX(), -.35*m_stick.getY(), .20*m_stick.getRawAxis(4), 0.0);
    }

    //**Shooter Motor Engaging When Button Pushed**
    //Note: Button Values Are Placeholders
    if (m_stick.getRawButtonPressed(5)) {
      shooter.set(ControlMode.PercentOutput, 0.25);
    } else if (m_stick.getRawButtonPressed(6)) {
      shooter.set(ControlMode.PercentOutput, -.25);
    } else{
      shooter.set(ControlMode.PercentOutput, 0);
    }
    
    //**Intake Motor Engaging When Button Pushed**
    //Note: Button Values Are Placeholders
    if (m_stick.getRawButtonPressed(7)) {
      intake.set(ControlMode.PercentOutput, 0.25);
    } else if (m_stick.getRawButtonPressed(8)) {
      intake.set(ControlMode.PercentOutput, -.25);
    } else{
      intake.set(ControlMode.PercentOutput, 0);
    }
  }
}