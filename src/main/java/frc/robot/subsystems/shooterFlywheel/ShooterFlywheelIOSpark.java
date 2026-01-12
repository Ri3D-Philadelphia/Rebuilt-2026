package frc.robot.subsystems.shooterFlywheel;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Voltage;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.SmartMotorControllerOptions;

public class ShooterFlywheelIOSpark implements ShooterFlywheelIO {
    private final SparkMax flywheelMotor = new SparkMax(ShooterFlywheelConstants.kLeaderId, MotorType.kBrushless);
    private final RelativeEncoder encoder;
    
    public ShooterFlywheelIOSpark() {
        encoder = flywheelMotor.getEncoder();
    }

    @Override
    public void updateInputs(ShooterFlywheelInputs inputs) {
        inputs.velocityRPM = encoder.getVelocity();
        inputs.appliedVoltage = flywheelMotor.getAppliedOutput() * flywheelMotor.getBusVoltage();
        inputs.currentAmps = flywheelMotor.getOutputCurrent();
    }

    @Override
    public void setVoltage(double volts) {
        flywheelMotor.setVoltage(volts);
    }

    @Override
    public void stop() {
        flywheelMotor.stopMotor();
    }
}
