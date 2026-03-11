package frc.robot.subsystems.floorRollers;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** AdvantageKit-style FloorRollers subsystem. */
public class FloorRollers extends SubsystemBase {
  private final FloorRollersIO io;
  private final FloorRollersIOInputsAutoLogged inputs = new FloorRollersIOInputsAutoLogged();

  public FloorRollers(FloorRollersIO io) {
    this.io = io;
  }

  public void setRollerMotorPercentOutput(double outputPercent) {
    io.setRollerOpenLoop(outputPercent);
  }

  public Command StartTurn(double power) {
    return runOnce(() -> setRollerMotorPercentOutput(power));
  }

  public Command rollOutwards(double power) {
    return runOnce(() -> setRollerMotorPercentOutput(power * -1));
  }

  public Command rollInwardsCommand(double power) {
    return runOnce(() -> setRollerMotorPercentOutput(power));
  }

  public Command Break(double power) {
    return runOnce(() -> setRollerMotorPercentOutput(0));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("FloorRollers", inputs);
  }
}
