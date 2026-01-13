package frc.robot.subsystems.indexer;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Voltage;

@Logged
public interface IndexerIO {
    
    default void setIndexerVoltage(Voltage volts) {}
    
    default void setSecondIndexerVoltage(Voltage volts) {}

}
