package frc.robot.subsystems.shooterFlywheel;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Voltage;

public class ShooterFlywheelIOSpark implements ShooterFlywheelIO {
    private final CANSparkMax motor;
    private final RelativeEncoder encoder;
    private final SparkPIDController pid;

    public ShooterFlywheelIOSpark(int canId) {
        motor = new CANSparkMax(canId, MotorType.kBrushless);
        encoder = motor.getEncoder();
        pid = motor.getPIDController();
    }

    @Override
    public void updateInputs(ShooterFlywheelInputs inputs) {
        inputs.velocityRPM = encoder.getVelocity();
        inputs.appliedVoltage = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.currentAmps = motor.getOutputCurrent();
    }

    @Override
    public void setVelocityRPM(double rpm) {
        pid.setReference(rpm, ControlType.kVelocity);
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }
}
