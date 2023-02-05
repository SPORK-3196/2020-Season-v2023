/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Index;
import frc.robot.Robot;

public class DriveToPickup extends CommandBase {

  Drivetrain drivetrain;
  Index index;

  /**
   * Creates a new DriveToPickup.
   */
  public DriveToPickup(Drivetrain p_drivetrain, Index p_index) {
    // Use addRequirements() here to declare subsystem dependencies.
    drivetrain = p_drivetrain;
    index = p_index;
    addRequirements(drivetrain, index);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.resetEncoders();
    index.intakeOut = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double steering = -0.0005 * drivetrain.leftRightDifference;
    drivetrain.drivetrain.arcadeDrive(0.45, steering);

    if(!index.lastSensor0Value && index.getSensorValue(0) && !Robot.shooting){
      index.index();
    }
    index.run();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drivetrain.arcadeDrive(0.0, 0.0);
    
    drivetrain.frontRight.setNeutralMode(NeutralMode.Brake);
    drivetrain.rearRight.setNeutralMode(NeutralMode.Brake);
    drivetrain.frontLeft.setNeutralMode(NeutralMode.Brake);
    drivetrain.rearLeft.setNeutralMode(NeutralMode.Brake);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return drivetrain.getRightEncoderPosition() > 125000; //110000
  }
}
