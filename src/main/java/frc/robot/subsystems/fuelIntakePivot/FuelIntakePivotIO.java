package frc.robot.subsystems.fuelIntakePivot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;

@Logged
public interface FuelIntakePivotIO {

    default void setPivotVoltage(Voltage volts) {}

    default void updateInputs(FuelIntakePivotInputs inputs) {}

    default void resetEncoder(Angle angle) {}
    
}
