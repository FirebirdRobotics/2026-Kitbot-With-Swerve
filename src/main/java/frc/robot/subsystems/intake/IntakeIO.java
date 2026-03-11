package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

/** IO interface for the Intake subsystem (AdvantageKit style). */
public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    // Pivot (position is in inches to match the reference implementation)
    public boolean pivotConnected = false;
    public double pivotPositionInches = 0.0;
    public double pivotVelocityInchesPerSec = 0.0;
    public double pivotAppliedVolts = 0.0;
    public double pivotCurrentAmps = 0.0;

    // Roller
    public boolean rollerConnected = false;
    public double rollerAppliedVolts = 0.0;
    public double rollerCurrentAmps = 0.0;
  }

  /** Update the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /** Run the pivot motor to a motion magic / position target in inches. */
  public default void setPivotMotionMagicPosition(double inches) {}

  /** Run the roller in open loop -1.0..1.0. */
  public default void setRollerOpenLoop(double output) {}
}
