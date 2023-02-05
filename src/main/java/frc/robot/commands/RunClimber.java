/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Climber;

public class RunClimber extends CommandBase {

  Climber climber;

  /**
   * Creates a new RunClimber.
   */
  public RunClimber(Climber p_climber) {
    // Use addRequirements() here to declare subsystem dependencies.
    climber = p_climber;
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.left.set(0.0);
    climber.right.set(0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Robot.controllerDrive.getBButton()) {
      climber.left.set(0.5);
      climber.right.set(0.5);
    }

    double rightDownInput = Robot.controllerDrive.getRightTriggerAxis();
    if(rightDownInput > 0.05) {
      climber.right.set(-rightDownInput);
    }

    double leftDownInput = Robot.controllerDrive.getLeftTriggerAxis();
    if(leftDownInput > 0.05) {
      climber.left.set(-leftDownInput);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
