/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;
import frc.robot.Robot;
import com.revrobotics.ControlType;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;

public class AlignTurret extends CommandBase {

  Turret turret;
  int turretTarget;
  double hoodTarget;

  boolean hoodZeroed = false;
  boolean turretLocked = false;

  /**
   * Creates a new AlignTurret.
   */
  public AlignTurret(Turret p_turret, int p_turretTarget, double p_hoodTarget) {
    // Use addRequirements() here to declare subsystem dependencies.
    turret = p_turret;
    turretTarget = p_turretTarget;
    hoodTarget = p_hoodTarget;
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turret.setSetpoint(turretTarget);
    turret.enable();

    System.out.print("Hood target: ");
    System.out.println(hoodTarget);
    turret.hood.set(-0.025);

    Robot.turretError = (int)turret.getController().getPositionError();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double hoodValue = turret.hood.getEncoder().getPosition();
    turret.hoodPos.setDouble(hoodValue);

    if(turret.hood.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen).get()) {
      hoodZeroed = true;
    }

    if(hoodZeroed) {
      turret.hood.set(0.0);
      turret.hoodPID.setReference(hoodTarget, ControlType.kPosition);
    } else {
      turret.hood.set(-0.025);
    }

    Robot.turretError = (int)turret.getController().getPositionError();
    if(Math.abs(Robot.turretError) < 10) {
      turretLocked = true;
    }

    if(turretLocked) {
      turret.turret.set(0.0);
    } else {
      turret.turret.set(0.5*turret.lastTurretOutput);
    }

    turret.turretEncoderDashboard.setDouble(turret.getPWMPosition());
    Robot.hoodTarget = hoodTarget;
    turret.hoodPos.setDouble(hoodValue);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.disable();
    turret.turret.set(0.0);
    turret.hoodPID.setReference(0.25, ControlType.kPosition);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
