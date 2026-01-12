/**
 * CODE STRUCTURE INSPIRED BY TEAM 321 REEFSCAPE
 */
package frc.robot.subsystems.fuelIntakeRoller;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class FuelIntakeRoller extends SubsystemBase {
    
    private FuelIntakeRollerIO io;

    private double tuningVoltage = 0;

    public FuelIntakeRoller() {
        this.io = new FuelIntakeRollerIOSpark();
    }

    /**
     * Returns a Command to spin the intake inwards.
     */
    public Command rollIn() {
        return run(() -> io.setRollerVoltage(FuelIntakeRollerConstants.kRollerInVoltage));
    }

    /**
     * Returns a Command to spin the intake outwards.
     */
    public Command rollOut() {
        return run(() -> io.setRollerVoltage(FuelIntakeRollerConstants.kRollerOutVoltage));
    }

    /**
     * Returns a Command to stop the intake roller.
     */
    public Command stopIntake() {
        return run(() -> io.setRollerVoltage(Volts.of(0)));
    }
    
}
