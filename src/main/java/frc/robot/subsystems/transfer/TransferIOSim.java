package frc.robot.subsystems.transfer;

/** Simple simulation IO for transfer roller. */
public class TransferIOSim implements TransferIO {
  private double lastOutput = 0.0;

  @Override
  public void updateInputs(TransferIOInputs inputs) {
    inputs.connected = true;
    inputs.appliedVolts = lastOutput * 12.0;
    inputs.currentAmps = Math.abs(lastOutput) * 3.0;
  }

  @Override
  public void setRollerOpenLoop(double output) {
    lastOutput = output;
  }
}
