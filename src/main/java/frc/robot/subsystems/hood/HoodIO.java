package frc.robot.subsystems.hood;

import org.littletonrobotics.junction.AutoLog;

/** IO interface for the Hood subsystem (AdvantageKit style). */
public interface HoodIO {
  @AutoLog
  public static class HoodIOInputs {
    public boolean pivotConnected = false;
    public double pivotPosition = 0.0;
    public double pivotVelocity = 0.0;
    public double pivotAppliedVolts = 0.0;
    public double pivotStatorCurrentAmps = 0.0;
    public double pivotSupplyCurrentAmps = 0.0;
  }

  /** Update the set of loggable inputs. */
  public default void updateInputs(HoodIOInputs inputs) {}

  /** Run the pivot motor to a motion magic / position target. */
  public default void setPivotMotionMagicPosition(double position) {}

  /** Run the pivot motor in open loop duty cycle -1.0..1.0. */
  public default void setDutyCycleOutput(double duty) {}

  /** Set an open-loop voltage on the pivot motor (default: no-op). */
  public default void setVoltage(double volts) {}

  /** Reset the pivot encoder to the given position (default: no-op). */
  public default void resetEncoder(double position) {}

  /** Reset the pivot encoder to 0.0 (convenience default). */
  public default void resetEncoder() {
    resetEncoder(0.0);
  }
}
