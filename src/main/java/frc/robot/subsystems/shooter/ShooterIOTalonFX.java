package frc.robot.subsystems.shooter;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/** TalonFX-based Shooter IO implementation. */
public class ShooterIOTalonFX implements ShooterIO {
  private final TalonFX leader;
  private final TalonFX follower;

  private final StatusSignal<AngularVelocity> leaderVelocity;
  private final StatusSignal<Voltage> leaderAppliedVolts;
  private final StatusSignal<Current> leaderCurrent;

  private final MotionMagicVelocityVoltage velocityRequest = new MotionMagicVelocityVoltage(0);
  private final DutyCycleOut dutyRequest = new DutyCycleOut(0);

  public ShooterIOTalonFX() {
    leader = new TalonFX(ShooterConstants.shooterLeaderMotorID, "rio");
    follower = new TalonFX(ShooterConstants.shooterFollowerMotorID, "rio");

    // Basic configuration
    var leaderConfig = new TalonFXConfiguration();
    leaderConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    leaderConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    leaderConfig.Feedback.SensorToMechanismRatio = 1.0;
    leaderConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    leaderConfig.CurrentLimits.StatorCurrentLimit = 50;
    leaderConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    leaderConfig.CurrentLimits.SupplyCurrentLimit = 50;

    var slot0 = leaderConfig.Slot0;
    slot0.kS = 0.150390625;
    slot0.kV = 0.126;
    slot0.kA = 0.0;
    slot0.kP = 0.0;
    slot0.kI = 0.0;
    slot0.kD = 0.0;

    var mm = leaderConfig.MotionMagic;
    mm.MotionMagicAcceleration = 400;

    tryUntilOk(5, () -> leader.getConfigurator().apply(leaderConfig, 0.25));

    // Configure follower to follow leader
    follower.setControl(
        new Follower(leader.getDeviceID(), com.ctre.phoenix6.signals.MotorAlignmentValue.Opposed));

    leaderVelocity = leader.getVelocity();
    leaderAppliedVolts = leader.getMotorVoltage();
    leaderCurrent = leader.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, leaderVelocity, leaderAppliedVolts, leaderCurrent);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    var status = BaseStatusSignal.refreshAll(leaderVelocity, leaderAppliedVolts, leaderCurrent);
    inputs.leaderConnected = status.isOK();
    inputs.leaderVelocityRPS = leaderVelocity.getValueAsDouble();
    inputs.leaderAppliedVolts = leaderAppliedVolts.getValueAsDouble();
    inputs.leaderCurrentAmps = leaderCurrent.getValueAsDouble();
  }

  @Override
  public void setVelocity(double rps) {
    leader.setControl(velocityRequest.withVelocity(rps));
  }

  @Override
  public void setPercentOutput(double percent) {
    leader.setControl(dutyRequest.withOutput(percent));
  }
}
