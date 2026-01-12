package frc.robot.subsystems.indexer;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Voltage;

@Logged
public class IndexerIOSpark implements IndexerIO {
    private SparkFlex indexerMotor = new SparkFlex(IndexerConstants.kIndexerMotorId, MotorType.kBrushless);

    public IndexerIOSpark() {
        configureMotors();
    }

    /**
     * Configures the SparkFlex.
     */
    private void configureMotors() {
        indexerMotor.configure(
            new SparkFlexConfig()
                .inverted(IndexerConstants.kInverted)
                .voltageCompensation(IndexerConstants.kNominalVoltage.in(Volts))
                .smartCurrentLimit(IndexerConstants.kCurrentLimit),
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }
    
    /**
     * Set the indexer moc voltage.
     */
    public void setIndexerVoltage(Voltage volts) {
        indexerMotor.setVoltage(volts);
    }
    
}
