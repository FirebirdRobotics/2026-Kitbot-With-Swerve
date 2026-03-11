package frc.robot.subsystems.floorRollers;

/** Simple simulation IO for floor rollers. */
public class FloorRollersIOSim implements FloorRollersIO {
  private double lastOutput = 0.0;

  @Override
  public void updateInputs(FloorRollersIOInputs inputs) {
    inputs.connected = true;
    inputs.appliedVolts = lastOutput * 12.0;
    inputs.currentAmps = Math.abs(lastOutput) * 3.0;
  }

  @Override
  public void setRollerOpenLoop(double output) {
    lastOutput = output;
  }
}
