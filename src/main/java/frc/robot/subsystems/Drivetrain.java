/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;

public class Drivetrain extends SubsystemBase {

  public WPI_TalonFX frontLeft = new WPI_TalonFX(1);
  public WPI_TalonFX rearLeft = new WPI_TalonFX(2);
  public WPI_TalonFX frontRight = new WPI_TalonFX(3);
  public WPI_TalonFX rearRight = new WPI_TalonFX(4);

  MotorControllerGroup left = new MotorControllerGroup(frontLeft, rearLeft);
  MotorControllerGroup right = new MotorControllerGroup(frontRight, rearRight);

  public WPI_PigeonIMU gyroscope = new WPI_PigeonIMU(20);

  public DifferentialDrive drivetrain = new DifferentialDrive(left, right);

  //public Orchestra orchestra;

  public static Solenoid driveCooler = new Solenoid(50, PneumaticsModuleType.REVPH, 4);

  public static GenericEntry[] falconTempDashboard = new GenericEntry[4];
  public static GenericEntry rightEncoderDashboard = Shuffleboard.getTab("Default").add("Right Falcon Encoder", 0.0).getEntry();
  public static GenericEntry leftEncoderDashboard = Shuffleboard.getTab("Default").add("Left Falcon Encoder", 0.0).getEntry();
  public static GenericEntry leftRightDifferenceDashboard = Shuffleboard.getTab("Default").add("LeftRight Difference", 0.0).getEntry();

  public int rightEncoderOffset = 0;
  public int leftEncoderOffset = 0;
  public int leftRightDifference = 0;
  
  public DifferentialDriveOdometry m_odometry;

  public int getRightEncoderPosition() {
    return (int) (-frontRight.getSelectedSensorPosition() - rightEncoderOffset);
  }

  public int getLeftEncoderPosition() {
    return (int) (frontLeft.getSelectedSensorPosition() - leftEncoderOffset);
  }

  public void resetEncoders() {
    frontRight.setSelectedSensorPosition(0);
    frontLeft.setSelectedSensorPosition(0);
    leftRightDifference = getLeftEncoderPosition() - getRightEncoderPosition();
  }

   public void zeroGyro() {
     gyroscope.setYaw(0);
   }

   public double getGyroYaw() {
     return gyroscope.getYaw();
   }

   public double getGyroHeadingRads() {
     return Units.degreesToRadians(getGyroYaw());
   }



  /**
   * Creates a new Drivetrain.
   */
  public Drivetrain() {
    right.setInverted(true);

    falconTempDashboard[0] = Shuffleboard.getTab("Default").add("FL Falcon Temp", 0.0).getEntry();
    falconTempDashboard[1] = Shuffleboard.getTab("Default").add("RL Falcon Temp", 0.0).getEntry();
    falconTempDashboard[2] = Shuffleboard.getTab("Default").add("FR Falcon Temp", 0.0).getEntry();
    falconTempDashboard[3] = Shuffleboard.getTab("Default").add("RR Falcon Temp", 0.0).getEntry();

    resetEncoders();
    m_odometry = 
    new DifferentialDriveOdometry(
      new Rotation2d(getGyroHeadingRads()), 
      getRightEncoderPosition(), 
      getLeftEncoderPosition());

    /*ArrayList<TalonFX> fx_s = new ArrayList<TalonFX>();
    fx_s.add(frontLeft);
    //fx_s.add(rearLeft);
    fx_s.add(frontRight);
    //fx_s.add(rearRight);

    orchestra = new Orchestra(fx_s);*/
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry.update(
      new Rotation2d(getGyroHeadingRads()), sensorUnitsToMeters(getLeftEncoderPosition()), sensorUnitsToMeters(getRightEncoderPosition()));

    rightEncoderDashboard.setDouble(getRightEncoderPosition());
    leftEncoderDashboard.setDouble(getLeftEncoderPosition());

    leftRightDifference = getLeftEncoderPosition() - getRightEncoderPosition();
    leftRightDifferenceDashboard.setDouble(leftRightDifference);
  }
  
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public double sensorUnitsToMeters(double sensor_counts) {
    double motorRotations = (double)sensor_counts / Constants.countsPerRevolution;
    double wheelRotations = motorRotations / Constants.gearRatio;
    double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(Constants.wheelRadiusInch));
    return positionMeters;
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
      sensorUnitsToMeters(-frontRight.getSelectedSensorVelocity()),
      sensorUnitsToMeters(frontLeft.getSelectedSensorVelocity()));
    //Constants.k100msPerSecond * sensorUnitsToMeters(getLeftEncoderPosition()), 
    //-Constants.k100msPerSecond * sensorUnitsToMeters(getRightEncoderPosition()));
  }

  public void resetOdometry(Pose2d pose){
    resetEncoders();
    m_odometry.resetPosition(new Rotation2d(getGyroHeadingRads()), sensorUnitsToMeters(frontRight.getSelectedSensorPosition()), sensorUnitsToMeters(frontLeft.getSelectedSensorPosition()), pose);
  }

  public void arcadeDrive(double speed, double rotation){
    drivetrain.arcadeDrive(speed, rotation);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts){
    left.setVoltage(leftVolts);
    right.setVoltage(rightVolts);
    drivetrain.feed();
  }

  public double getAverageEncoderDistance(){
    return ((sensorUnitsToMeters(-frontRight.getSelectedSensorPosition())+sensorUnitsToMeters(frontLeft.getSelectedSensorPosition()))/2);
  }

  public void setMaxOutput(double maxOutput){
    drivetrain.setMaxOutput(maxOutput);
  }
}