// ...existing code...
package frc.robot.subsystems.shooterFlywheel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

// removed CTRE ControlMode import to avoid collision with YAMS
// import com.ctre.phoenix.motorcontrol.ControlMode;

// Use REV CANSparkMax classes
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

// ...existing code...
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.Constants;
import frc.robot.subsystems.indexer.IndexerIOSpark;
// removed swervelib import — not present in your dependencies
// import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;



public class ShooterFlywheel extends SubsystemBase {
    private ShooterFlywheelIO io;
    private final ShooterFlywheelInputs inputs = new ShooterFlywheelInputs();

    // removed self-referential field
    // private ShooterFlywheel m_shooterFlywheel;

    private final  PIDController pid = new PIDController(
        ShooterFlywheelConstants.kP, 
        ShooterFlywheelConstants.kI, 
        ShooterFlywheelConstants.kD);

    private double targetRPM = 0.0;


    public ShooterFlywheel(ShooterFlywheelIO io){
       this.io = new ShooterFlywheelIOSpark();
        pid.setTolerance(ShooterFlywheelConstants.kRpmTolerance);
    }


    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        io.updateInputs(inputs);
        if (targetRPM > 0.0){
            double volts = pid.calculate(inputs.velocityRPM, targetRPM);
            io.setVoltage(volts);
        }
    }

    public void setRPM(double rpm){
        targetRPM = rpm;
    }

    public void stop(){
        targetRPM = 0.0;
        io.stop();
        pid.reset();
    }

    public boolean atSPeed(){
        return pid.atSetpoint();
    }

    public double getRPM(){
        return inputs.velocityRPM;
    }


    public Command spinUpCommand(double rpm){
        return Commands.run(() -> setRPM(rpm), this);
    }

    public Command stopCommand(){
        return Commands.runOnce(this::stop, this);
    }
}