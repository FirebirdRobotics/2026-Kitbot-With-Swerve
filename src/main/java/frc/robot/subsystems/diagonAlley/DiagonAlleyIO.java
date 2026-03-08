package frc.robot.subsystems.diagonAlley;

import org.littletonrobotics.junction.AutoLog;

/** IO interface for DiagonAlley rollers. */
public interface DiagonAlleyIO {
  @AutoLog
  public static class DiagonAlleyIOInputs {
    public boolean leaderConnected = false;
    public double leaderAppliedVolts = 0.0;
    public double leaderCurrentAmps = 0.0;
  }

  public default void updateInputs(DiagonAlleyIOInputs inputs) {}

  public default void setRollerOpenLoop(double output) {}
}
