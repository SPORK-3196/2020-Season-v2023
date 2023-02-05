/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    
    public static final double ksVolts = 0.1849;
    public static final double kvVoltSecondsPerMeter = .21685;
    public static final double kaVoltSecondsSquaredPerMeter = 0.073225;
    public static final double kPDriveVel = .16373;

    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = .7;

    public static final double kMaxSpeedMetersPerSecond = 2;
    public static final double kMaxAcceleration = .5;

    public static final double kTrackwidthMeters = Units.inchesToMeters(24.5);
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

    public static final double countsPerRevolution = 2048;
    public static final double wheelRadiusInch = 3;
    public static final double gearRatio = 8.68;
    public static final double k100msPerSecond = 10;
}
