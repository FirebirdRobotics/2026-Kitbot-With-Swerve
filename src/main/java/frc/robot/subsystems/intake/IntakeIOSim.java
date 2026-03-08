package frc.robot.subsystems.intake;

/**
 * Simple simulation implementation of {@link IntakeIO}. This provides minimal simulation behavior
 * so the rest of the robot code can call the IO methods during simulation or on a real robot if a
 * concrete hardware IO isn't supplied yet.
 */
public class IntakeIOSim implements IntakeIO {
  // Track last commands for simple simulation state
  private double lastPivotTargetInches = 0.0;
  private double lastRollerOutput = 0.0;

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // Copy our simulated state into the provided inputs object.
    inputs.pivotConnected = true;
    inputs.pivotPositionInches = this.lastPivotTargetInches;
    inputs.pivotVelocityInchesPerSec = 0.0;
    inputs.pivotAppliedVolts = lastRollerOutput * 12.0; // not realistic; simple placeholder
    inputs.pivotCurrentAmps = 0.0;

    inputs.rollerConnected = true;
    inputs.rollerAppliedVolts = lastRollerOutput * 12.0;
    inputs.rollerCurrentAmps = Math.abs(lastRollerOutput) * 5.0;
  }

  @Override
  public void setPivotMotionMagicPosition(double inches) {
    // Immediately set the target (no motion profiling in this sim)
    lastPivotTargetInches = inches;
  }

  @Override
  public void setRollerOpenLoop(double output) {
    lastRollerOutput = output;
  }
}
