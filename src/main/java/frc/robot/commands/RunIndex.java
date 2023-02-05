/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Index;
import edu.wpi.first.wpilibj.DigitalInput;

public class RunIndex extends CommandBase {

  Index index;

  /**
   * Creates a new RunIndex.
   */
  public RunIndex(Index index_p) {
    index = index_p;
    addRequirements(index);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    index.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    /*double leftY = Robot.controllerSecondary.getLeftY();
    if(leftY > -0.1 && leftY < 0.1) leftY = 0.0;

    double rightX = Robot.controllerSecondary.getRightX();
    if(rightX > -0.1 && rightX < 0.1) rightX = 0.0;

    index.firstStage.set(-rightX);
    index.secondStage.set(-leftY);*/


    if(!index.lastSensor0Value && index.getSensorValue(0) && !Robot.shooting){
      index.index();
    }

    boolean resetIndex = false;//Robot.controllerSecondary.getRightBumper();
    if(resetIndex) {
      index.reset();
    }

    //boolean toggleIntake = Robot.controllerSecondary.getBButtonPressed();
    /*if(toggleIntake) {
      index.intakeOut = !index.intakeOut;
    }*/

    boolean runIntake = Robot.controllerSecondary.getBButton();
    index.intakeOut = runIntake;

    index.run();
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
