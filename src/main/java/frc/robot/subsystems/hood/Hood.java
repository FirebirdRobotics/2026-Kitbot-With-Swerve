// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hood;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** AdvantageKit-style hood subsystem delegating to a HoodIO implementation. */
public class Hood extends SubsystemBase {
  private final HoodIO io;
  // Use the raw IO inputs type here so compilation doesn't depend on the autolog-generated
  // "AutoLogged" wrapper being present. The AdvantageKit annotation processor normally
  // generates HoodIOInputsAutoLogged from the @AutoLog annotation; if annotation
  // processing isn't running the generated type will be missing and cause a compile error.
  private final HoodIO.HoodIOInputs inputs = new HoodIO.HoodIOInputs();

  private LinearFilter currentFilter = LinearFilter.movingAverage(5);

  public static final double ZEROING_CURRENT_THRESHOLD_AMPS = 15; // was usually barely over 16

  @AutoLogOutput(key = "Hood/Current Filter Value")
  private double currentFilterValue = 0.0;

  public Hood(HoodIO io) {
    this.io = io;
  }

  public void setDutyCycleOutput(double dutyCycle) {
    io.setDutyCycleOutput(dutyCycle);
  }

  public Command CommandSetDutyCycleOutput(double dutyCycle) {
    return runOnce(() -> setDutyCycleOutput(dutyCycle));
  }

  public void goToAngle(double angle) {
    io.setPivotMotionMagicPosition(angle);
  }

  public Command CommandGoToAngle(double angle) {
    return runOnce(() -> goToAngle(angle));
  }

  public Command CommandGoToLowestAngle() {
    return runOnce(() -> goToAngle(HoodConstants.lowestAngle));
  }

  public Command runCurrentZeroing() {
    return this.run(
            () -> {
              io.setVoltage(-0.5);
            })
        .until(() -> Math.abs(currentFilterValue) > ZEROING_CURRENT_THRESHOLD_AMPS)
        .andThen(Commands.parallel(Commands.print("Hood Zeroed"), rezero()))
        .andThen(
            () -> {
              io.setVoltage(0.0);
            });
  }

  public Command rezero() {
    // return Commands.runOnce(() -> io.resetEncoder(0.0));
    return Commands.runOnce(
        () ->
            io.resetEncoder(
                16.778206)); // Set this to 16.778206 when correcting shooter setpoint units later
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    // Log individual fields to avoid a hard dependency on the autolog-generated
    // "AutoLogged" wrapper type (which may not be present if the annotation
    // processor hasn't run). Using explicit recordOutput calls keeps logging
    // working even when generated types are missing.
    Logger.recordOutput("Hood/Connected", inputs.pivotConnected);
    Logger.recordOutput("Hood/Position", inputs.pivotPosition);
    Logger.recordOutput("Hood/Velocity", inputs.pivotVelocity);
    Logger.recordOutput("Hood/AppliedVolts", inputs.pivotAppliedVolts);
    Logger.recordOutput("Hood/StatorCurrentAmps", inputs.pivotStatorCurrentAmps);
    Logger.recordOutput("Hood/SupplyCurrentAmps", inputs.pivotSupplyCurrentAmps);

    // Update moving-average of stator current used for zeroing detection
    currentFilterValue = currentFilter.calculate(inputs.pivotStatorCurrentAmps);
  }
}
