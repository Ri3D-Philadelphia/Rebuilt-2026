package frc.robot.subsystems.shooterFlywheel;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.fuelIntakePivot.FuelIntakePivotConstants;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.SmartMotorControllerOptions;

public class ShooterFlywheelIOSpark implements ShooterFlywheelIO {
    private final SparkFlex flywheelMotor = new SparkFlex(ShooterFlywheelConstants.kLeaderId, MotorType.kBrushless);
    private final SparkClosedLoopController velocityController;
    private final RelativeEncoder encoder;
    
    public ShooterFlywheelIOSpark() {
        encoder = flywheelMotor.getEncoder();
        velocityController = flywheelMotor.getClosedLoopController();
        // velocityController.setSetpoint(100, ControlType.kVelocity); // this doesnt work
        configureMotors();
    }

    private void configureMotors() {
    SparkFlexConfig config = new SparkFlexConfig();
    
    config
        .inverted(ShooterFlywheelConstants.kInverted)
        .voltageCompensation(ShooterFlywheelConstants.kNominalVoltage.in(Volts))
        .smartCurrentLimit(ShooterFlywheelConstants.kCurrentLimit);
    
    // Configure PID
    config.closedLoop
        .p(ShooterFlywheelConstants.kP)
        .i(ShooterFlywheelConstants.kI)
        .d(ShooterFlywheelConstants.kD)
        .outputRange(-1, 1);
    
    // Configure feedforward
    config.closedLoop.feedForward
        .kV(ShooterFlywheelConstants.kV);
    
    // Configure encoder
    config.encoder
        .velocityConversionFactor(ShooterFlywheelConstants.kFlywheelVelocityConversionFactor)
        .positionConversionFactor(ShooterFlywheelConstants.kFlywheelPositionConversionFactor);
    
    // Apply the configuration
    flywheelMotor.configure(
        config,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters
    );
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
    public void setVelocityRPM(double rpm) {
        // This tells the SparkFlex to use its internal PID to reach this velocity
        velocityController.setSetpoint(rpm, ControlType.kVelocity);
    }

    @Override
    public void stop() {
        flywheelMotor.stopMotor();
    }
}
