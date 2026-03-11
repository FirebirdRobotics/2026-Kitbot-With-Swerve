package frc.robot.subsystems.hood;

import edu.wpi.first.math.util.Units;

/** Constants for the Hood subsystem (angles expressed in radians). */
public class HoodConstants {
  // CAN ID for the hood pivot TalonFX
  public static final int hoodPivotMotorID = 0; // replace with real ID if different

  // Example base/limit angles (expressed in degrees here and converted to radians)
  // The rest of the code (superstructure) passes radians, so store the limits in radians.
  public static final double lowestAngle = Units.degreesToRadians(17.0); // ~0.2967 rad
  public static final double highestAngle = Units.degreesToRadians(47.0); // ~0.8203 rad
}
