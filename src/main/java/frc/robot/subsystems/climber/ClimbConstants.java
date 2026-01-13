package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

@Logged
public class ClimbConstants {
    public static final int kClimbMotorId = 7;  
    public static final boolean kInverted = false;
    public static final int kCurrentLimit = 60;

    // homing
    public static final Voltage kHomeVoltage = Volts.of(-2);
    public static final Current kHomeCurrentThresh = Amps.of(15);
    
    // mechanical
    public static final double kClimbGearing = 9.0; // TODO: ASK MECH!!!

    // soft limits
    public static final double kMinRotations = 0.0;
    public static final double kMaxRotations = 4500.0;

    // pid
    public static final Angle kControllerTolerance = Degrees.of(1);

    // moc
    public static final double kClimbPositionConversionFactor = 360 / kClimbGearing;
    public static final double kClimbVelocityConversionFactor = kClimbPositionConversionFactor / 60;
    public static final Voltage kNominalVoltage = Volts.of(12);
}
