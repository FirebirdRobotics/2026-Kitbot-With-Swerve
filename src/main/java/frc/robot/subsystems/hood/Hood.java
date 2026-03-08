// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hood;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** AdvantageKit-style hood subsystem delegating to a HoodIO implementation. */
public class Hood extends SubsystemBase {
  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

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

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);
  }
}
