package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** AdvantageKit-style Intake subsystem. Delegates hardware to an IntakeIO implementation. */
public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
    this.io = io;
  }

  /** Extend pivot to an absolute distance in inches. */
  public void extendToDistance(double inches) {
    io.setPivotMotionMagicPosition(inches);
  }

  public Command goToDeployAndThenToUndeployCommand() {
    return runEnd(this::goToDeployedPosition, this::goToFramePerimeterPosition);
  }

  public void goToDeployedPosition() {
    io.setPivotMotionMagicPosition(IntakeConstants.deployDistance);
  }

  public void goToFramePerimeterPosition() {
    io.setPivotMotionMagicPosition(IntakeConstants.framePerimeterDistance);
    setRollerMotorPercentOutput(0);
  }

  public Command CommandGoToDistance(double inches) {
    return runOnce(() -> extendToDistance(inches));
  }

  public void setRollerMotorPercentOutput(double outputPercent) {
    io.setRollerOpenLoop(outputPercent);
  }

  public Command setRollerMotorPercentOutputAndThenTo0Command(double power) {
    return runEnd(() -> setRollerMotorPercentOutput(power), () -> setRollerMotorPercentOutput(0));
  }

  public Command setRollerMotorPercentOutputCommand(double power) {
    return runOnce(() -> setRollerMotorPercentOutput(power));
  }

  public Command goToDeployedPositionCommand() {
    return runOnce(this::goToDeployedPosition);
  }

  public Command goToFramePerimeterPositionCommand() {
    return runOnce(this::goToFramePerimeterPosition);
  }

  public Command goToMidPointPositionCommand() {
    return runOnce(() -> extendToDistance(IntakeConstants.midPoint));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }
}
