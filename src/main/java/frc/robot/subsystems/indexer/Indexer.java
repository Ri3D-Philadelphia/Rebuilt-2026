package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class Indexer extends SubsystemBase {

    private IndexerIO io;

    public Indexer() {
        this.io = new IndexerIOSpark();
    }
    
    public Command runIndexerOnce() {
        return run(() -> io.setIndexerVoltage(IndexerConstants.kIndexerOnVoltage))
            .withTimeout(IndexerConstants.kIndexerRunTime)
            .finallyDo(() -> io.setIndexerVoltage(Volts.of(0)));
    }
    
    public Command runSecondIndexerOnce() {
        return run(() -> io.setSecondIndexerVoltage(IndexerConstants.kSecondIndexerOnVoltage))
            .withTimeout(IndexerConstants.kSecondIndexerRunTime)
            .finallyDo(() -> io.setSecondIndexerVoltage(Volts.of(0)));
    }
}
