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

  void setVoltage(double volts);

  void resetEncoder(double position);
}
