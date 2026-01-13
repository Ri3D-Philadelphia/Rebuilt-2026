package frc.robot.subsystems.fuelIntakeRoller;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;

@Logged
public class FuelIntakeRollerConstants {
    // CAN
    public static final int kRollerMotorId = 3;

    // moc outputs
    public static final Voltage kRollerInVoltage = Volts.of(6);
    public static final Voltage kRollerOutVoltage = Volts.of(-6);
    
    // mechanical
    public static final double kPivotGearing = 1.0; // TODO: might be incorrect

    // moc
    public static final boolean kInverted = false;
    public static final int kCurrentLimit = 40;
    /**
     * In Rotations.
     */
    public static final double kPivotPositionConversionFactor = 1 / kPivotGearing;
    /**
     * In Rotations/Second.
     */
    public static final double kPivotVelocityConversionFactor = kPivotPositionConversionFactor / 60;
    public static final Voltage kNominalVoltage = Volts.of(12);
    
}
