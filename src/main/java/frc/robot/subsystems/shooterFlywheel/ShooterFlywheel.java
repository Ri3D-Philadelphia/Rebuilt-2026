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

import frc.robot.Constants;
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
    
    // removed self-referential field
    // private ShooterFlywheel m_shooterFlywheel;

    private SparkMax shooterSpark = new SparkMax(Constants.ShooterConstants.kLeaderMotorId, MotorType.kBrushless);

    // declare followerSpark so Pair.of(...) compiles (initialize later where appropriate)
    private SparkMax followerSpark;

    private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
    .withFollowers(Pair.of(followerSpark, false))
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(0.1, 0, 0)
      .withFeedforward(new SimpleMotorFeedforward(0, 0.5, 0))
      .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(4)))
      .withMotorInverted(false)
      .withIdleMode(MotorMode.COAST)
      .withStatorCurrentLimit(Amps.of(40));

    private SmartMotorController smc = new SparkWrapper(shooterSpark, DCMotor.getNEO(1), smcConfig);


    private final FlyWheelConfig shooterConfig = new FlyWheelConfig(smc)
        .withDiameter(Inches.of(4))
        .withMass(Pounds.of(1))
        .withUpperSoftLimit(RPM.of(6000))
        .withLowerSoftLimit(RPM.of(0))
        .withTelemetry("ShooterFlywheel", TelemetryVerbosity.HIGH);

    private final FlyWheel shooterFlywheel = new FlyWheel(shooterConfig);

    public ShooterFlywheel(){

    }

    public Command setSpeed(AngularVelocity speed){
        return shooterFlywheel.setSpeed(speed);
    }

    public Command spinUp(){
        return shooterFlywheel.setSpeed(RotationsPerSecond.of(50));
    }

    public Command stop(){
        return shooterFlywheel.set(0);
    }

    public AngularVelocity getSpeed(){
        return shooterFlywheel.getSpeed();
    }

    public Command set(double dutyCycle){
        return shooterFlywheel.set(dutyCycle);
    }

    public Command sysId(){
        return shooterFlywheel.sysId(Volts.of(10), Volts.of(2).per(Second), Seconds.of(10));
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        shooterFlywheel.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        shooterFlywheel.simIterate();
    }
}