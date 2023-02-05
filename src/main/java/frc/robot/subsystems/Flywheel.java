/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.InvertType;

public class Flywheel extends PIDSubsystem {
  
  public WPI_TalonSRX flywheel1 = new WPI_TalonSRX(9);
  public WPI_VictorSPX flywheel2 = new WPI_VictorSPX(10);

  /**
   * Creates a new Flywheel.
   */
  public Flywheel() {
    super(
        // The PIDController used by the subsystem
        new PIDController(2.0, 0.0, 0.5));
    
    flywheel2.follow(flywheel1);
    flywheel1.setInverted(true);
    flywheel2.setInverted(InvertType.FollowMaster);
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
    if(output < 0.0) { flywheel1.set(0.0); }
    else { flywheel1.set(output); }
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return flywheel1.getSelectedSensorVelocity();
  }
}
