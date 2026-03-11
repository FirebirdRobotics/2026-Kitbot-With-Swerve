package frc.robot.subsystems.intake;

/** Constants for the Intake subsystem. Values taken from the provided reference implementation. */
public class IntakeConstants {
  // CAN IDs (set to values from the user's reference)
  public static final int pivotMotorCANID = 42;
  public static final int rollerMotorCANID = 47;

  public static final String kCANBus = "CANivore";

  // Distances / positions (units: inches to match the reference)
  public static final double deployDistance = 4.59;
  public static final double midPoint = 3.0;
  public static final double framePerimeterDistance = 1.3;
}
