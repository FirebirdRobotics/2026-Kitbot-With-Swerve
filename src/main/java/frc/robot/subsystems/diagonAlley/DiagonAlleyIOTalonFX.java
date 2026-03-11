package frc.robot.subsystems.diagonAlley;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/** TalonFX IO implementation for DiagonAlley rollers. */
public class DiagonAlleyIOTalonFX implements DiagonAlleyIO {
  private final TalonFX leader;
  private final TalonFX follower;

  private final StatusSignal<AngularVelocity> leaderVelocity;
  private final StatusSignal<Voltage> leaderAppliedVolts;
  private final StatusSignal<Current> leaderCurrent;

  private final DutyCycleOut dutyRequest = new DutyCycleOut(0);

  public DiagonAlleyIOTalonFX() {
    leader = new TalonFX(DiagonAlleyConstants.leftRollerMotorCANID, "rio");
    follower = new TalonFX(DiagonAlleyConstants.rightRollerMotorCANID, "rio");

    follower.setControl(new Follower(leader.getDeviceID(), MotorAlignmentValue.Opposed));

    var cfg = new TalonFXConfiguration();
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    cfg.CurrentLimits.SupplyCurrentLimit = 50;
    cfg.CurrentLimits.StatorCurrentLimitEnable = true;
    cfg.CurrentLimits.StatorCurrentLimit = 50;

    tryUntilOk(5, () -> leader.getConfigurator().apply(cfg, 0.25));

    leaderVelocity = leader.getVelocity();
    leaderAppliedVolts = leader.getMotorVoltage();
    leaderCurrent = leader.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, leaderVelocity, leaderAppliedVolts, leaderCurrent);
  }

  @Override
  public void updateInputs(DiagonAlleyIOInputs inputs) {
    var status = BaseStatusSignal.refreshAll(leaderVelocity, leaderAppliedVolts, leaderCurrent);
    inputs.leaderConnected = status.isOK();
    inputs.leaderAppliedVolts = leaderAppliedVolts.getValueAsDouble();
    inputs.leaderCurrentAmps = leaderCurrent.getValueAsDouble();
  }

  @Override
  public void setRollerOpenLoop(double output) {
    leader.setControl(dutyRequest.withOutput(output));
  }
}
