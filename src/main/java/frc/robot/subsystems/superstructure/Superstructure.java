// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.superstructure;

import static frc.robot.subsystems.superstructure.SuperstructureConstants.controlSystemsVelocityRadPerSec;
import static frc.robot.subsystems.superstructure.SuperstructureConstants.intakingFeederVoltage;
import static frc.robot.subsystems.superstructure.SuperstructureConstants.launchingFeederVoltage;
import static frc.robot.subsystems.superstructure.SuperstructureConstants.launchingLauncherVoltage;
import static frc.robot.subsystems.superstructure.SuperstructureConstants.spinUpFeederVoltage;
import static frc.robot.subsystems.superstructure.SuperstructureConstants.spinUpSeconds;
import static frc.robot.subsystems.superstructure.SuperstructureConstants.totalExitVelocity;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.SuperstructureConstants.SpeedInterpolationMap;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {
  public final SuperstructureIO io;
  private final SuperstructureIOInputsAutoLogged inputs = new SuperstructureIOInputsAutoLogged();

  public Superstructure(SuperstructureIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Superstructure", inputs);
  }

  /** Set the rollers to the values for intaking. */
  public Command intake() {
    return runEnd(
        () -> {
          io.setFeederVoltage(intakingFeederVoltage);
          io.setIntakeLauncherVoltage(intakingFeederVoltage);
        },
        () -> {
          io.setFeederVoltage(0.0);
          io.setIntakeLauncherVoltage(0.0);
        });
  }

  /** Set the rollers to the values for ejecting fuel out the intake. */
  public Command eject() {
    return runEnd(
        () -> {
          io.setFeederVoltage(-intakingFeederVoltage);
          io.setIntakeLauncherVoltage(-intakingFeederVoltage);
        },
        () -> {
          io.setFeederVoltage(0.0);
          io.setIntakeLauncherVoltage(0.0);
        });
  }

  /** Set the rollers to the values for launching. Spins up before feeding fuel. */
  public Command launch() {
    return run(() -> {
          io.setFeederVoltage(spinUpFeederVoltage);
          io.setIntakeLauncherVoltage(launchingLauncherVoltage);
        })
        .withTimeout(spinUpSeconds)
        .andThen(
            run(
                () -> {
                  io.setFeederVoltage(launchingFeederVoltage);
                  io.setIntakeLauncherVoltage(launchingLauncherVoltage);
                }))
        .finallyDo(
            () -> {
              io.setFeederVoltage(0.0);
              io.setIntakeLauncherVoltage(0.0);
            });
  }

  public Command launchWithPID() {
    return run(
        () -> {
          io.setIntakeLauncherVelocity(controlSystemsVelocityRadPerSec);
        });
  }

  public Command launchAtVelocity(double vel) {
    return run(
        () -> {
          io.setIntakeLauncherVelocity(vel);
        });
  }

  public Command setHoodAngle(double angle) {
    angle = angle + 1;
    return Commands.none();
  }

  public Command shootOnTheFly(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Translation2d> gSupplier) {
    // This maps distance to horizontal speed
    // To calculate these values, we need to try shooting from a fixed (non-zero) angle at different
    // velocities and then record the distance it shoots; only a few data points should be necessary
    final SpeedInterpolationMap shooterSpeedMap =
        (new SuperstructureConstants()).new SpeedInterpolationMap();

    ChassisSpeeds robotSpeeds = drive.getChassisSpeeds();

    // Calculate where robot will be once we're done processing and actually ready to shoot
    Translation2d futurePose =
        drive
            .getPose()
            .getTranslation()
            .plus(
                new Translation2d(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond)
                    .times(SuperstructureConstants.latency));

    // offset
    Translation2d target = gSupplier.get().minus(futurePose);

    // Shot vector (target vector -> normalized -> multiplied by horiz. speed -> subtract robot
    // speed)
    Translation2d shot =
        target
            .div(target.getNorm())
            .times(shooterSpeedMap.get(target.getNorm()))
            .minus(new Translation2d(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond));

    double angle = shot.getAngle().getRadians();

    // Find parabola angle to compensate for horizontal speed
    double pitch =
        Math.acos(Math.min(shot.getNorm() / SuperstructureConstants.totalExitVelocity, 1.0));

    // Parallel because drive at angle takes a while to terminate
    return Commands.parallel(
        DriveCommands.joystickDriveAtAngle(
            drive, xSupplier, ySupplier, () -> new Rotation2d(angle)),
        setHoodAngle(pitch),
        launchAtVelocity(totalExitVelocity));
  }
}
