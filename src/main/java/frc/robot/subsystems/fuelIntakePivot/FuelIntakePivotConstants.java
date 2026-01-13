package frc.robot.subsystems.fuelIntakePivot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

@Logged
public class FuelIntakePivotConstants {
    // CAN
    public static final int kPivotMotorId = 2;
    public static final boolean kInverted = false;
    public static final int kCurrentLimit = 40;
    
    // homing
    public static final Voltage kHomeVoltage = Volts.of(-2);
    public static final Current kHomeCurrentThresh = Amps.of(1);
    
    // mechanical
    public static final double kPivotGearing = 1.0; // TODO: ASK MECH!!!

    // pid
    public static final Angle kControllerTolerance = Degrees.of(1);

    // moc
    public static final double kPivotPositionConversionFactor = 360 / kPivotGearing;
    public static final double kPivotVelocityConversionFactor = kPivotPositionConversionFactor / 60;
    public static final Voltage kNominalVoltage = Volts.of(12);
}
