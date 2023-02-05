/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.commands.DriveForwardTimed;
import frc.robot.commands.DriveWithJoystick;
import frc.robot.commands.FiveBallAuto;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.commands.RunIndex;
import frc.robot.subsystems.Index;
import frc.robot.commands.RunTurret;
import frc.robot.subsystems.Turret;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain drivetrain = new Drivetrain();
  private final Index index = new Index();
  private final Turret turret = new Turret();
  private final Flywheel flywheel = new Flywheel();

  private final DriveWithJoystick driveWithJoystick = new DriveWithJoystick(drivetrain);
  private final RunIndex runIndex = new RunIndex(index);
  private final RunTurret runTurret = new RunTurret(turret, flywheel);

  private GenericEntry autoSelect = Shuffleboard.getTab("Default").add("Shoot during auto?", true).getEntry();


  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    drivetrain.setDefaultCommand(driveWithJoystick);
    index.setDefaultCommand(runIndex);
    turret.setDefaultCommand(runTurret);
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    if(autoSelect.getBoolean(true)) {
      return new FiveBallAuto(turret, flywheel, index, drivetrain);
    } else {
      return new DriveForwardTimed(drivetrain, 1.0);
    }
  }
  public Command getTrajectory() {
    //Voltage constraint of robot, how much voltage can be used for movement
    var autoVoltageConstraint = 
      new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerMeter, Constants.kaVoltSecondsSquaredPerMeter), 
        Constants.kDriveKinematics, 
        10);
    
    TrajectoryConfig trajectoryConfig = 
      new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond,
      Constants.kMaxAcceleration)
      .setKinematics(Constants.kDriveKinematics)
      .addConstraint(autoVoltageConstraint);

    Trajectory trajectory = 
       TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(new Translation2d(2, 2), new Translation2d(4, -2)),
        new Pose2d(7, 0, new Rotation2d(0)),
        trajectoryConfig
      );
    
    RamseteCommand ramseteCommand =
      new RamseteCommand(
        trajectory, 
        drivetrain::getPose, 
        new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta), 
        new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerMeter, Constants.kaVoltSecondsSquaredPerMeter),
        Constants.kDriveKinematics, 
        drivetrain::getWheelSpeeds,
        new PIDController(Constants.kPDriveVel, 0, 0), 
        new PIDController(Constants.kPDriveVel, 0, 0), 
        drivetrain::tankDriveVolts, 
        drivetrain
        );
    
    drivetrain.resetOdometry(trajectory.getInitialPose());

    return ramseteCommand.andThen(() -> drivetrain.tankDriveVolts(0,0));
  }
  public Command getTrajCommand() {
    return getTrajectory();
  }
}
