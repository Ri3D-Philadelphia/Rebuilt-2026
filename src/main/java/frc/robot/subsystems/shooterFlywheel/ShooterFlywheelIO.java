package frc.robot.subsystems.shooterFlywheel;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Voltage;

@Logged
public interface ShooterFlywheelIO {

    default void updateInputs(ShooterFlywheelInputs inputs){}
    
    default void setVoltage(double volts){}

    default void setVelocityRPM(double rpm){}

    default void stop(){}
}
