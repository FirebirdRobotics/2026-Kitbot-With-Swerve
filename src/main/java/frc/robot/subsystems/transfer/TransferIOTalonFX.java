package frc.robot.subsystems.transfer;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.generated.TunerConstants;

/** TalonFX IO implementation for Transfer roller. */
public class TransferIOTalonFX implements TransferIO {
  private final TalonFX talon;

  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> current;

  private final DutyCycleOut dutyRequest = new DutyCycleOut(0);

  public TransferIOTalonFX() {
    talon = new TalonFX(TransferConstants.transferRollerMotorID, TunerConstants.kCANBus);

    var cfg = new TalonFXConfiguration();
    cfg.CurrentLimits.StatorCurrentLimitEnable = true;
    cfg.CurrentLimits.StatorCurrentLimit = 50;
    cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    cfg.CurrentLimits.SupplyCurrentLimit = 50;

    tryUntilOk(5, () -> talon.getConfigurator().apply(cfg, 0.25));

    velocity = talon.getVelocity();
    appliedVolts = talon.getMotorVoltage();
    current = talon.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, velocity, appliedVolts, current);
  }

  @Override
  public void updateInputs(TransferIOInputs inputs) {
    var status = BaseStatusSignal.refreshAll(velocity, appliedVolts, current);
    inputs.connected = status.isOK();
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = current.getValueAsDouble();
  }

  @Override
  public void setRollerOpenLoop(double output) {
    talon.setControl(dutyRequest.withOutput(output));
  }
}
