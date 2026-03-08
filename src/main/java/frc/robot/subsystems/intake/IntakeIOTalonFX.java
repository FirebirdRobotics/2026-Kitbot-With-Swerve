package frc.robot.subsystems.intake;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/** TalonFX-based IO implementation for the Intake subsystem. */
public class IntakeIOTalonFX implements IntakeIO {
  private final TalonFX pivotTalon;
  private final TalonFX rollerTalon;

  // Status signals
  private final StatusSignal<Angle> pivotPosition;
  private final StatusSignal<AngularVelocity> pivotVelocity;
  private final StatusSignal<Voltage> pivotAppliedVolts;
  private final StatusSignal<Current> pivotCurrent;

  private final StatusSignal<Voltage> rollerAppliedVolts;
  private final StatusSignal<Current> rollerCurrent;

  // Control requests (reused)
  private final MotionMagicVoltage pivotMotionRequest = new MotionMagicVoltage(0);

  public IntakeIOTalonFX() {
    pivotTalon = new TalonFX(IntakeConstants.pivotMotorCANID, IntakeConstants.kCANBus);
    rollerTalon = new TalonFX(IntakeConstants.rollerMotorCANID, IntakeConstants.kCANBus);

    // Configure pivot motor (basic settings inspired by the reference implementation)
    var pivotConfig = new TalonFXConfiguration();
    pivotConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    pivotConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    // Sensor-to-mechanism ratio copied from the reference intake (tune as needed)
    pivotConfig.Feedback.SensorToMechanismRatio = 6.333;

    pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    pivotConfig.CurrentLimits.SupplyCurrentLimit = 40;
    pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    pivotConfig.CurrentLimits.StatorCurrentLimit = 40;

    // Basic slot 0 gains (copied from the reference; tune in tuner if needed)
    var slot0 = pivotConfig.Slot0;
    slot0.kS = 0.17;
    slot0.kV = 0.23;
    slot0.kA = 0.18;
    slot0.kP = 0.121;
    slot0.kI = 0.0;
    slot0.kD = 0.3;

    tryUntilOk(5, () -> pivotTalon.getConfigurator().apply(pivotConfig, 0.25));

    // Configure roller motor current limits
    var rollerConfig = new TalonFXConfiguration();
    rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rollerConfig.CurrentLimits.StatorCurrentLimit = 50;
    rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rollerConfig.CurrentLimits.SupplyCurrentLimit = 50;
    tryUntilOk(5, () -> rollerTalon.getConfigurator().apply(rollerConfig, 0.25));

    // Create status signals
    pivotPosition = pivotTalon.getPosition();
    pivotVelocity = pivotTalon.getVelocity();
    pivotAppliedVolts = pivotTalon.getMotorVoltage();
    pivotCurrent = pivotTalon.getStatorCurrent();

    rollerAppliedVolts = rollerTalon.getMotorVoltage();
    rollerCurrent = rollerTalon.getStatorCurrent();

    // Configure update frequency for logging
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, pivotVelocity, pivotAppliedVolts, pivotCurrent, rollerAppliedVolts, rollerCurrent);
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, pivotPosition);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    var pivotStatus =
        BaseStatusSignal.refreshAll(pivotPosition, pivotVelocity, pivotAppliedVolts, pivotCurrent);
    var rollerStatus = BaseStatusSignal.refreshAll(rollerAppliedVolts, rollerCurrent);

    inputs.pivotConnected = pivotStatus.isOK();
    // Note: sensor returns rotations/angle based value depending on config. We mirror the
    // reference by exposing the raw position value here; tune SensorToMechanismRatio so that
    // this value corresponds to inches if required.
    inputs.pivotPositionInches = pivotPosition.getValueAsDouble();
    inputs.pivotVelocityInchesPerSec = pivotVelocity.getValueAsDouble();
    inputs.pivotAppliedVolts = pivotAppliedVolts.getValueAsDouble();
    inputs.pivotCurrentAmps = pivotCurrent.getValueAsDouble();

    inputs.rollerConnected = rollerStatus.isOK();
    inputs.rollerAppliedVolts = rollerAppliedVolts.getValueAsDouble();
    inputs.rollerCurrentAmps = rollerCurrent.getValueAsDouble();
  }

  @Override
  public void setPivotMotionMagicPosition(double inches) {
    pivotTalon.setControl(pivotMotionRequest.withPosition(inches));
  }

  @Override
  public void setRollerOpenLoop(double output) {
    DutyCycleOut rollerDutyRequest = new DutyCycleOut(output);
    rollerTalon.setControl(rollerDutyRequest.withOutput(output));
  }
}
