package frc.robot.subsystems.diagonAlley;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** AdvantageKit-style DiagonAlley subsystem. */
public class DiagonAlley extends SubsystemBase {
  private final DiagonAlleyIO io;
  private final DiagonAlleyIOInputsAutoLogged inputs = new DiagonAlleyIOInputsAutoLogged();

  public DiagonAlley(DiagonAlleyIO io) {
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
    Logger.processInputs("DiagonAlley", inputs);
  }
}
