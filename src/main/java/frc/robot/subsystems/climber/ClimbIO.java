package frc.robot.subsystems.climber;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;

@Logged
public interface ClimbIO {

    default void setClimbVoltage(Voltage volts) {}

    default void updateInputs(ClimbInputs inputs) {}

    default void resetEncoder(Angle angle) {}
    
}
