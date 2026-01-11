/**
 * CODE STRUCTURE INSPIRED BY TEAM 321 REEFSCAPE
 */
package frc.robot.subsystems.fuelIntakeRoller;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class FuelIntakeRoller extends SubsystemBase {
    
    private FuelIntakeRollerIO io;

    private double tuningVoltage = 0;

    public FuelIntakeRoller() {
        this.io = new FuelIntakeRollerIOSpark();
    }
    
}
