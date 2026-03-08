package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

/** IO interface for the Shooter subsystem (AdvantageKit style). */
public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public boolean leaderConnected = false;
    public double leaderVelocityRPS = 0.0;
    public double leaderAppliedVolts = 0.0;
    public double leaderCurrentAmps = 0.0;
  }

  /** Update the set of loggable inputs. */
  public default void updateInputs(ShooterIOInputs inputs) {}

  /** Set the shooter velocity in rotations per second (RPS). */
  public default void setVelocity(double rps) {}

  /** Set open-loop percent output (-1..1). */
  public default void setPercentOutput(double percent) {}
}
