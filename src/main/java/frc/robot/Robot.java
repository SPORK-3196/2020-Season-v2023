/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.SerialPort;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static XboxController controllerDrive = new XboxController(0);
  public static XboxController controllerSecondary = new XboxController(1);
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  public Compressor compressor = new Compressor(50, PneumaticsModuleType.CTREPCM);

  private UsbCamera cam0;
  private SerialPort cam0_ser;

  public static double camX = 320.0;
  public static double camY = 0.0;

  public static int flywheelVel = 0;
  public static int lastFlywheelVel = 0;
  public static int deltaFlywheelVel = 0;

  public static boolean shooting = false;

  public static double hoodTarget = 0.0;
  public static double manualHoodOffset = 0.0;

  public static int lastPOV = -1;

  public static int turretError = 100;

  public GenericEntry camXDashboard = Shuffleboard.getTab("Default").add("Camera X", 160.0).getEntry();
  public GenericEntry camYDashboard = Shuffleboard.getTab("Default").add("Camera Y", 0.0).getEntry();

  public GenericEntry flywheelVelocityDashboard = Shuffleboard.getTab("Default").add("Flywheel Velocity", 0.0).getEntry();

  public GenericEntry hoodTargetDashboard = Shuffleboard.getTab("Default").add("Hood Target", 0.0).getEntry();
  public GenericEntry manualHoodOffsetDashboard = Shuffleboard.getTab("Default").add("Manual Hood Offset", 0.0).getEntry();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    cam0 = CameraServer.startAutomaticCapture();

    try {
      cam0_ser = new SerialPort(115200, SerialPort.Port.kUSB);
    } catch(Exception e) {
      System.out.println(e.toString());
    }

    //compressor.clearAllPCMStickyFaults();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    String data = "";
    if(cam0_ser != null) {
      data = cam0_ser.readString();
    }
    if(!data.equals("")) {
      //System.out.println(data);
      try {
        String numData = data.substring(32, data.length() - 2);

        int commaIndex = numData.indexOf(",");
        String xStr = numData.substring(0,commaIndex);
        String yStr = numData.substring(commaIndex+1);

        camX = Double.parseDouble(xStr);
        camY = Double.parseDouble(yStr);
        //System.out.println(camX);
      } catch(Exception e) {
        System.out.println(e);
      }
    }

    camXDashboard.setDouble(camX);
    camYDashboard.setDouble(camY);
    flywheelVelocityDashboard.setDouble(flywheelVel);

    hoodTargetDashboard.setDouble(hoodTarget);
    manualHoodOffsetDashboard.setDouble(manualHoodOffset);
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getTrajCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
