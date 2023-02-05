/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Index;

public class ShootBalls extends CommandBase {

  Timer timer = new Timer();
  Index index;
  int ballCount = 3;

  /**
   * Creates a new ShootThreeBalls.
   */
  public ShootBalls(Index p_index, int p_ballCount) {
    // Use addRequirements() here to declare subsystem dependencies.
    index = p_index;
    addRequirements(index);
    ballCount = p_ballCount;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    index.reset();
    Robot.shooting = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!index.getSensorValue(5) && index.lastSensor5Value) {
      ballCount--;
      System.out.println("Shot a ball");

      if(ballCount <= 0) {
        timer.reset();
        timer.start();
      }
    }
    
    index.run();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    index.stopMotors();
    Robot.shooting = false;
    System.out.println("Shot 3 balls");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ballCount <= 0 && timer.get() > 1.0;
  }
}
