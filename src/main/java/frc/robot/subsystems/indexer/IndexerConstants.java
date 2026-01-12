package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;

@Logged
public class IndexerConstants {
    public static final int kIndexerMotorId = 4;
    public static final boolean kInverted = false;
    public static final int kCurrentLimit = 40;
    
    // mechanical
    public static final double kIndexerGearing = 1.0; // TODO: ASK MECH

    // moc outputs
    public static final Voltage kIndexerOnVoltage = Volts.of(3);
    public static final Time kIndexerRunTime = Seconds.of(2);

    // moc config
    public static final double kIndexerPositionConversionFactor = 1 / kIndexerGearing;
    public static final double kIndexerVelocityConversionFactor = kIndexerPositionConversionFactor / 60;
    public static final Voltage kNominalVoltage = Volts.of(12);

    
}
