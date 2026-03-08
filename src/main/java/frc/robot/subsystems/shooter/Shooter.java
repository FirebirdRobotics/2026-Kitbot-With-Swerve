package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** AdvantageKit-style Shooter subsystem delegating to ShooterIO. */
public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  public void goToRPS(double rps) {
    io.setVelocity(rps);
  }

  public Command setVelocityCommand(double rps) {
    return runOnce(() -> goToRPS(rps));
  }

  public void setPercentOutput(double percent) {
    io.setPercentOutput(percent);
  }

  public Command setPercentOutputCommand(double percent) {
    return runOnce(() -> setPercentOutput(percent));
  }

  public Command goTo400RPS() {
    return runOnce(() -> goToRPS(400));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }
}
