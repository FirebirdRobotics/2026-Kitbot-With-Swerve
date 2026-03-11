package frc.robot.subsystems.shooter;

/** Simple simulation implementation of {@link ShooterIO}. */
public class ShooterIOSim implements ShooterIO {
  private double lastRps = 0.0;
  private double lastPercent = 0.0;

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.leaderConnected = true;
    inputs.leaderVelocityRPS = lastRps;
    inputs.leaderAppliedVolts = lastPercent * 12.0;
    inputs.leaderCurrentAmps = Math.abs(lastPercent) * 5.0;
  }

  @Override
  public void setVelocity(double rps) {
    lastRps = rps;
  }

  @Override
  public void setPercentOutput(double percent) {
    lastPercent = percent;
  }
}
