package frc.robot.subsystems.hood;

/** Simple simulation implementation of {@link HoodIO}. */
public class HoodIOSim implements HoodIO {
  private final HoodIOInputs inputs = new HoodIOInputs();

  private double lastPosition = 0.0;
  private double lastDuty = 0.0;

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.pivotConnected = true;
    inputs.pivotPosition = lastPosition;
    inputs.pivotVelocity = 0.0;
    inputs.pivotAppliedVolts = lastDuty * 12.0;
    inputs.pivotStatorCurrentAmps = Math.abs(lastDuty) * 3.0;
    inputs.pivotSupplyCurrentAmps = Math.abs(lastDuty) * 3.0;
  }

  @Override
  public void setPivotMotionMagicPosition(double position) {
    lastPosition = position;
  }

  @Override
  public void setDutyCycleOutput(double duty) {
    lastDuty = duty;
  }

  public void setVoltage(double volts) {}

  @Override
  public void resetEncoder(double position) {}
}
