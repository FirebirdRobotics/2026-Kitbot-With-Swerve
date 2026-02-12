// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.superstructure;

public class SuperstructureConstants {
  public static final int feederCanId = 21;
  public static final double feederMotorReduction = 1.0;
  public static final int feederCurrentLimit = 60;

  public static final int intakeLauncherCanId = 20;
  public static final double intakeLauncherMotorReduction = 1.0;
  public static final int intakeLauncherCurrentLimit = 60;

  public static final double intakingFeederVoltage = -12.0;
  public static final double intakingIntakeVoltage = 10.0;
  public static final double launchingFeederVoltage = 10.0;
  public static final double launchingLauncherVoltage = -7.5;
  public static final double spinUpFeederVoltage = -6.0;
  public static final double spinUpSeconds = 1.0;

  public static final double kFeederP = 0.01;
  public static final double kFeederI = 0;
  public static final double kFeederD = 0.001;
  public static final double kFeederMinOutput = 0;
  public static final double kFeederMaxOutput = 10;

  public static final double kLauncherP = 0.01;
  public static final double kLauncherI = 0;
  public static final double kLauncherD = 0.001;
  public static final double kLauncherMinOutput = 0;
  public static final double kLauncherMaxOutput = 10;
}
