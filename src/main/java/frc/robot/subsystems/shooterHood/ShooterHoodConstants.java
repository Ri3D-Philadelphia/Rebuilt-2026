package frc.robot.subsystems.shooterHood;

import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.Voltage;

public class ShooterHoodConstants {
    
    // Motor configuration
    public static final int kHoodMotorId = 5;
    public static final boolean kInverted = false;
    public static final int kCurrentLimit = 20;  // Hood doesn't need much current

    // Angle limits (degrees)
    public static final double kMinAngleDeg = 0.0;    // Flat/lowest
    public static final double kMaxAngleDeg = 40.0;   // Steep/highest

    // PID Constants
    public static final double kP = 2;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kG = 3.0;

    // Tolerance
    public static final double kAngleToleranceDeg = 1.0;

    // Preset angles for different shot distances
    public static final double kCloseAngleDeg = 10.0;    // Close shot
    public static final double kMidAngleDeg = 20.0;      // Mid range
    public static final double kFarAngleDeg = 35.0;      // Far shot
    public static final double kAmpAngleDeg = 5.0;       // Amp scoring

    // Mechanical conversion
    // TODO: Get actual gear ratio from mech team!
    public static final double kGearRatio = 100.0;  // Example: 100:1 reduction
    public static final double kDegreesPerRotation = 360.0 / kGearRatio;

    // Motor config
    public static final Voltage kNominalVoltage = Volts.of(12);
}