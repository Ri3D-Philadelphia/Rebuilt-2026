package frc.robot.subsystems.shooterFlywheel;

import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.Voltage;

public class ShooterFlywheelConstants {
    // Motor configuration
    public static final int kLeaderId = 6;
    public static final boolean kInverted = true;
    public static final int kCurrentLimit = 60;  // NEO Vortex can handle more current

    // Shooting parameters
    public static final double kDefaultShootRPM = 4500;
    public static final double kRpmTolerance = 100;  // Within 100 RPM = "at speed"

    // Closed-loop PID Constants (on-board SparkFlex PID)
    // These run on the motor controller at 1kHz, much faster than RoboRIO
    // Start conservative and tune up
    public static final double kP = 0.0001;   // Proportional gain
    public static final double kI = 0.0;      // Integral gain (usually not needed for velocity)
    public static final double kD = 0.0;      // Derivative gain (usually not needed for velocity)
    
    // Velocity feedforward (kV)
    // This is the primary control for velocity - tells motor "X% output = Y RPM"
    // Calculate as: kV = 1.0 / maxRPM
    // For NEO Vortex: max ~6784 RPM free speed
    // kV ≈ 1.0 / 6784 ≈ 0.000147
    public static final double kV = 0.00015;  // Start here, tune if needed
    
    // Mechanical
    public static final double kFlywheelGearing = 1.0;  // Direct drive? Verify with mech team

    // Motor conversions (for encoder readings)
    public static final double kFlywheelPositionConversionFactor = 360 / kFlywheelGearing;
    public static final double kFlywheelVelocityConversionFactor = kFlywheelPositionConversionFactor / 60;
    public static final Voltage kNominalVoltage = Volts.of(12);
    
    // Speed presets for different shot distances
    public static final double kCloseShootRPM = 3000;   // Close shot
    public static final double kMidShootRPM = 4500;     // Mid range
    public static final double kFarShootRPM = 5500;     // Far shot
    public static final double kAmpShootRPM = 1500;     // Slow shot for amp
}