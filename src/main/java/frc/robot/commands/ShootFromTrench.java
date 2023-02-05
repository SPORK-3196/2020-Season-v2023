/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Index;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ShootFromTrench extends ParallelRaceGroup {
  /**
   * Creates a new ShootFromLine.
   */
  public ShootFromTrench(Turret p_turret, Flywheel p_flywheel, Index p_index) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());super();
    super(
      new AlignTurret(p_turret, 3475, 11.1),
      new RunFlywheel(p_flywheel, 270),
      new ShootBalls(p_index, 2)
    );
  }
}
