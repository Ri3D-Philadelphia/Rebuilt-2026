/**
 * CODE STRUCTURE INSPIRED BY TEAM 321 REEFSCAPE
 */
package frc.robot.subsystems.fuelIntakeRoller;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.fuelIntakePivot.FuelIntakePivot;

@Logged
public class FuelIntakeRoller extends SubsystemBase {

    public enum RollerState {
        IN,
        OUT,
        STOPPED
    }
    
    private RollerState desiredState = RollerState.STOPPED;
    
    private FuelIntakeRollerIO io;

    private FuelIntakePivot pivotRef;

    private double tuningVoltage = 0;

    public FuelIntakeRoller(FuelIntakePivot pivotRef) {
        this.io = new FuelIntakeRollerIOSpark();
        this.pivotRef = pivotRef;
    }

    /**
     * Update this subsystem's roller state to In.
     */
    public void setRollIn() {
        desiredState = RollerState.IN;
    }

    /**
     * Update this subsystem's roller state to Out.
     */
    public void setRollOut() {
        desiredState = RollerState.OUT;
    }

    /**
     * Update this subsystem's roller state to Stopped.
     */
    public void setStopped() {
        desiredState = RollerState.STOPPED;
    }

    /**
     * Get whether this subsystem's roller state is In.
     */
    public boolean isRollingIn() {
        return desiredState == RollerState.IN;
    }

    /**
     * Get whether this subsystem's roller state is Out.
     */
    public boolean isRollingOut() {
        return desiredState == RollerState.OUT;
    }

    /**
     * Set the roller motor voltage based on roller state.
     */
    public Command holdRoller() {
        return run(() -> {
            if (!pivotRef.isExtended()) {
                io.setRollerVoltage(Volts.of(0));
                return;
            }

            switch (desiredState) {
                case IN ->
                    io.setRollerVoltage(FuelIntakeRollerConstants.kRollerInVoltage);
                case OUT ->
                    io.setRollerVoltage(FuelIntakeRollerConstants.kRollerOutVoltage);
                case STOPPED ->
                    io.setRollerVoltage(Volts.of(0));
            }
        });
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
