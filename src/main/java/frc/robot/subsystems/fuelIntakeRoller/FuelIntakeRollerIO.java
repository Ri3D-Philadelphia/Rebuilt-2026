package frc.robot.subsystems.fuelIntakeRoller;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Voltage;

@Logged
public interface FuelIntakeRollerIO {

    default void setRollerVoltage(Voltage volts) {};
    
}
