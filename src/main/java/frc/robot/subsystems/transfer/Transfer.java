package frc.robot.subsystems.transfer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** AdvantageKit-style Transfer subsystem. */
public class Transfer extends SubsystemBase {
  private final TransferIO io;
  private final TransferIOInputsAutoLogged inputs = new TransferIOInputsAutoLogged();

  public Transfer(TransferIO io) {
    this.io = io;
  }

  public void setRollerMotorPercentOutput(double outputPercent) {
    io.setRollerOpenLoop(outputPercent);
  }

  public Command manualRollForwards(double power) {
    return runOnce(() -> setRollerMotorPercentOutput(power));
  }

  public Command manualRollBackward(double power) {
    return runOnce(() -> setRollerMotorPercentOutput(power * -1));
  }

  /**
   * Start rolling forwards until the command ends; stops when ended. Useful for binding to a button
   * as a while-held command.
   */
  public Command startRollForwardsCommand(double power) {
    return runEnd(() -> setRollerMotorPercentOutput(power), () -> setRollerMotorPercentOutput(0));
  }

  /** Start rolling backwards until the command ends; stops when ended. */
  public Command startRollBackwardsCommand(double power) {
    return runEnd(
        () -> setRollerMotorPercentOutput(power * -1), () -> setRollerMotorPercentOutput(0));
  }

  /** Stop the rollers immediately. */
  public Command stopRollersCommand() {
    return runOnce(() -> setRollerMotorPercentOutput(0));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Transfer", inputs);
  }
}
