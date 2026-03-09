// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  // Alliance
  public static final Alliance alliance = DriverStation.getAlliance().isPresent()
                                ? DriverStation.getAlliance().get()
                                : Alliance.Red;

  // Helper functions
  public static final Translation2d mirrorAlliance(Translation2d t) {
    return (alliance == Alliance.Blue) ? t : new Translation2d(16.540988 - t.getX(), 8.069326 - t.getY());
  }

  public static final Pose2d mirrorAlliance(Pose2d t) {
    return (alliance == Alliance.Blue) ? t : new Pose2d(new Translation2d(16.540988 - t.getTranslation().getX(), 8.069326 - t.getTranslation().getY()), t.getRotation());
  }

  // Field measurements
  public static Translation2d fieldCenterTarget = new Translation2d(8.270494, 4.034663);
  public static Translation2d hubTarget = new Translation2d(4.625594, 4.034536);
  public static Translation2d nearTrenchTarget = new Translation2d(4.611624, 0.650748);
  public static Translation2d farTrenchTarget = new Translation2d(16.540988 - 4.611624, 8.069326 - 0.650748);

  // Autonomous constants
  public static Pose2d autonomousDestination = new Pose2d(2, 2, new Rotation2d(0.5));
}
