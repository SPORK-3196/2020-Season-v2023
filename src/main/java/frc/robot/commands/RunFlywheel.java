/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Flywheel;
import com.ctre.phoenix.motorcontrol.InvertType;

public class RunFlywheel extends CommandBase {

  Flywheel flywheel;
  int targetVelocity;

  /**
   * Creates a new RunFlywheel.
   */
  public RunFlywheel(Flywheel p_flywheel, int p_targetVelocity) {
    // Use addRequirements() here to declare subsystem dependencies.
    flywheel = p_flywheel;
    addRequirements(flywheel);
    targetVelocity = p_targetVelocity;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    flywheel.flywheel2.follow(flywheel.flywheel1);
    flywheel.flywheel1.setInverted(true);
    flywheel.flywheel2.setInverted(InvertType.FollowMaster);

    flywheel.setSetpoint(targetVelocity);
    flywheel.enable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.flywheelVel = (int) flywheel.flywheel1.getSelectedSensorVelocity();
    Robot.deltaFlywheelVel = Robot.flywheelVel - Robot.lastFlywheelVel;
    Robot.lastFlywheelVel = Robot.flywheelVel;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    flywheel.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
