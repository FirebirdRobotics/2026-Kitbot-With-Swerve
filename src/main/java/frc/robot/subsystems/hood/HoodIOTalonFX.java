package frc.robot.subsystems.hood;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/** TalonFX-based IO implementation for the Hood subsystem. */
public class HoodIOTalonFX implements HoodIO {
  private final TalonFX pivotTalon;

  private final StatusSignal<Angle> pivotPosition;
  private final StatusSignal<AngularVelocity> pivotVelocity;
  private final StatusSignal<Voltage> pivotAppliedVolts;
  private final StatusSignal<Current> pivotStatorCurrent;
  private final StatusSignal<Current> pivotSupplyCurrent;

  private VoltageOut voltageOut = new VoltageOut(0.0);
  private final MotionMagicVoltage pivotMotionRequest = new MotionMagicVoltage(0);
  private final DutyCycleOut dutyRequest = new DutyCycleOut(0);

  public HoodIOTalonFX() {
    pivotTalon = new TalonFX(HoodConstants.hoodPivotMotorID, "rio");

    var pivotConfig = new TalonFXConfiguration();
    pivotConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    pivotConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    // Match the original reference: converts angular motion to mechanism angle/distance
    // The original code used SensorToMechanismRatio = 4.0 (sensor rotations -> mechanism units).
    pivotConfig.Feedback.SensorToMechanismRatio = 4.0;

    pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    pivotConfig.CurrentLimits.SupplyCurrentLimit = 50;
    pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    pivotConfig.CurrentLimits.StatorCurrentLimit = 50;

    pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    var slot0 = pivotConfig.Slot0;
    slot0.kG = 0.25;
    slot0.kS = 0.0;
    slot0.kV = 0.85;
    slot0.kA = 0.1;
    slot0.kP = 45;
    slot0.kI = 0.0;
    slot0.kD = 0.0;
    slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
    slot0.GravityType = GravityTypeValue.Arm_Cosine;

    var mm = pivotConfig.MotionMagic;
    mm.MotionMagicCruiseVelocity = 10.0;
    mm.MotionMagicAcceleration = 10.0;
    mm.MotionMagicJerk = 0.0;

    tryUntilOk(5, () -> pivotTalon.getConfigurator().apply(pivotConfig, 0.25));

    pivotPosition = pivotTalon.getPosition();
    pivotVelocity = pivotTalon.getVelocity();
    pivotAppliedVolts = pivotTalon.getMotorVoltage();
    pivotStatorCurrent = pivotTalon.getStatorCurrent();
    pivotSupplyCurrent = pivotTalon.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, pivotVelocity, pivotAppliedVolts, pivotStatorCurrent, pivotSupplyCurrent);
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, pivotPosition);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    var status =
        BaseStatusSignal.refreshAll(
            pivotPosition,
            pivotVelocity,
            pivotAppliedVolts,
            pivotStatorCurrent,
            pivotSupplyCurrent);
    inputs.pivotConnected = status.isOK();
    inputs.pivotPosition = pivotPosition.getValueAsDouble();
    inputs.pivotVelocity = pivotVelocity.getValueAsDouble();
    inputs.pivotAppliedVolts = pivotAppliedVolts.getValueAsDouble();
    inputs.pivotStatorCurrentAmps = pivotStatorCurrent.getValueAsDouble();
    inputs.pivotSupplyCurrentAmps = pivotSupplyCurrent.getValueAsDouble();
  }

  @Override
  public void setPivotMotionMagicPosition(double position) {
    pivotTalon.setControl(pivotMotionRequest.withPosition(position));
  }

  @Override
  public void setDutyCycleOutput(double duty) {
    pivotTalon.setControl(dutyRequest.withOutput(duty));
  }

  @Override
  public void setVoltage(double volts) {
    pivotTalon.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void resetEncoder(double position) {
    pivotTalon.setPosition(position);
  }
}
