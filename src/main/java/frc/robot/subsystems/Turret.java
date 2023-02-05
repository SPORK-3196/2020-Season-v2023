/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANPIDController;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Robot;

public class Turret extends PIDSubsystem {
  
  //public SpeedControllerGroup flywheel = new SpeedControllerGroup(flywheel1, flywheel2);

  public WPI_TalonSRX turret = new WPI_TalonSRX(8);
  public double lastTurretOutput = 0.0;

  public CANSparkMax hood = new CANSparkMax(11, MotorType.kBrushless);
  public CANPIDController hoodPID = hood.getPIDController();
  
  public GenericEntry hoodPos = Shuffleboard.getTab("Default").add("Hood Position", 0.0).getEntry();
  public GenericEntry hoodP = Shuffleboard.getTab("Default").add("Hood P", 0.2).getEntry();
  public GenericEntry hoodI = Shuffleboard.getTab("Default").add("Hood I", 0.0).getEntry();
  public GenericEntry hoodD = Shuffleboard.getTab("Default").add("Hood D", 0.15).getEntry();

  public GenericEntry turretEncoderDashboard = Shuffleboard.getTab("Default").add("Turret Position", 0).getEntry();

  public GenericEntry turretInputDashboard = Shuffleboard.getTab("Default").add("Turret Input", 0.0).getEntry();
  public GenericEntry adjustedInputDashboard = Shuffleboard.getTab("Default").add("Adjusted Input", 0.0).getEntry();

  SensorCollection sensors = turret.getSensorCollection();
  volatile int lastValue = Integer.MIN_VALUE;

  public int getPWMPosition() {
    int raw = sensors.getPulseWidthRiseToFallUs();
    if(raw == 0) {
      int lastValue = this.lastValue;
      if(lastValue == Integer.MIN_VALUE) {
        return 0;
      }
      return lastValue;
    }
    int actualValue = Math.min(4096, raw-128);
    lastValue = actualValue;
    return actualValue;
  }

  /**
   * Creates a new Turret.
   */
  public Turret() {
    super(
        // The PIDController used by the subsystem
        new PIDController(0.005, 0.0, 0.00025));

    //turret.setSetpoint(1640);
    turret.disable();

    hoodPID.setP(0.03);
    hoodPID.setI(0.0);
    hoodPID.setD(0.0);
    hoodPID.setOutputRange(-0.3, 0.3);
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
    if(output > 0.5) {
      output = 0.5;
    } else if(output < -0.5) {
      output = -0.5;
    }
    lastTurretOutput = output;
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return lastValue;
  }
}
