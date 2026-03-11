package frc.robot.subsystems.diagonAlley;

/** Simple simulation IO for DiagonAlley rollers. */
public class DiagonAlleyIOSim implements DiagonAlleyIO {
  private double lastOutput = 0.0;

  @Override
  public void updateInputs(DiagonAlleyIOInputs inputs) {
    inputs.leaderConnected = true;
    inputs.leaderAppliedVolts = lastOutput * 12.0;
    inputs.leaderCurrentAmps = Math.abs(lastOutput) * 3.0;
  }

  @Override
  public void setRollerOpenLoop(double output) {
    lastOutput = output;
  }
}
