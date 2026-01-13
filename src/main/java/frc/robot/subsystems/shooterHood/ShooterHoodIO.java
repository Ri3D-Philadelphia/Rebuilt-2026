package frc.robot.subsystems.shooterHood;

public interface ShooterHoodIO{
    default void updateInputs(ShooterHoodInputs inputs){}
    default void setVoltage (double volts){}
    default void stop(){}
}