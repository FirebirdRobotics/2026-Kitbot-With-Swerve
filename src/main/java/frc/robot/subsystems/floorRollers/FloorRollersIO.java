package frc.robot.subsystems.floorRollers;

import org.littletonrobotics.junction.AutoLog;

/** IO interface for floor rollers. */
public interface FloorRollersIO {
  @AutoLog
  public static class FloorRollersIOInputs {
    public boolean connected = false;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  public default void updateInputs(FloorRollersIOInputs inputs) {}

  public default void setRollerOpenLoop(double output) {}
}
