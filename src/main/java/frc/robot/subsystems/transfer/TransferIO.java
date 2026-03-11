package frc.robot.subsystems.transfer;

import org.littletonrobotics.junction.AutoLog;

/** IO interface for Transfer rollers. */
public interface TransferIO {
  @AutoLog
  public static class TransferIOInputs {
    public boolean connected = false;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  public default void updateInputs(TransferIOInputs inputs) {}

  public default void setRollerOpenLoop(double output) {}
}
