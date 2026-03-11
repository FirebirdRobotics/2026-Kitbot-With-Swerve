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

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Transfer", inputs);
  }
}
