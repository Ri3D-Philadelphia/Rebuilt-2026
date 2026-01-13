// ...existing code...
package frc.robot.subsystems.shooterFlywheel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Volts;

// removed CTRE ControlMode import to avoid collision with YAMS
// import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;



public class ShooterFlywheel extends SubsystemBase {
    private ShooterFlywheelIO io;
    private final ShooterFlywheelInputs inputs = new ShooterFlywheelInputs();

    // removed self-referential field
    // private ShooterFlywheel m_shooterFlywheel;

    private double targetRPM = 0.0;


    public ShooterFlywheel(){
       this.io = new ShooterFlywheelIOSpark();
    }


    @Override
    public void periodic() {
        // // This method will be called once per scheduler run
        // io.updateInputs(inputs);
        // if (targetRPM > 0.0){
        //     double volts = pid.calculate(inputs.velocityRPM, targetRPM);
        //     io.setVoltage(volts);
        // }

        // Update inputs from hardware
        io.updateInputs(inputs);
        
        // Log data to SmartDashboard for debugging
        SmartDashboard.putNumber("Shooter/CurrentRPM", inputs.velocityRPM);
        SmartDashboard.putNumber("Shooter/TargetRPM", targetRPM);
        SmartDashboard.putNumber("Shooter/Voltage", inputs.appliedVoltage);
        SmartDashboard.putNumber("Shooter/Current", inputs.currentAmps);
        SmartDashboard.putBoolean("Shooter/AtSpeed", atSpeed());
        SmartDashboard.putNumber("Shooter/RPMError", Math.abs(targetRPM - inputs.velocityRPM));
    }


    public void setVelocityRPM(double rpm) {
        targetRPM = rpm;
        io.setVelocityRPM(rpm);
    }

    public Command powerFlywheel() {
        return run(() ->io.setVoltage(6));
    }

    public Command stop(){
        
        return run(() -> {
            targetRPM = 0.0;
            io.stop();
            io.setVoltage(0);});
    }

    public boolean atSpeed(){
        if (targetRPM == 0.0){
            return false;
        }
        double error = Math.abs(targetRPM - inputs.velocityRPM);
        return error < ShooterFlywheelConstants.kRpmTolerance;
    }

    public double getRPM(){
        return inputs.velocityRPM;
    }

    public double getTargetRPM(){
        return targetRPM;
    }


    // ==================== COMMANDS ====================

    /**
     * Command to spin the flywheel to a specific RPM.
     * The command runs continuously - use .withTimeout() or button release to stop.
     */
    public Command spinToRPM(double rpm) {
        // return Commands.run(
        //     () -> setVelocityRPM(rpm), 
        //     this
        // ).withName("SpinToRPM_" + rpm);

        return Commands.run(() -> io.setVoltage(6), this);
    }

    /**
     * Command to spin up to default shooting speed.
     */
    public Command spinToDefault() {
        return spinToRPM(ShooterFlywheelConstants.kDefaultShootRPM)
            .withName("SpinToDefault");
    }

    /**
     * Command to stop the flywheel.
     */
    public Command stopCommand() {
        return Commands.runOnce(this::stop, this)
            .withName("StopShooter");
    }

    /**
     * Command that spins up and waits until the flywheel reaches target speed.
     * Useful for autonomous or sequenced commands.
     */
    public Command spinUpAndWait(double rpm) {
        return Commands.sequence(
            Commands.runOnce(() -> setVelocityRPM(rpm)),
            Commands.waitUntil(this::atSpeed)
        ).withTimeout(3.0)  // 3 second timeout
         .withName("SpinUpAndWait_" + rpm);
    }

    /**
     * Command to spin up to default speed and wait.
     */
    public Command spinUpDefaultAndWait() {
        return spinUpAndWait(ShooterFlywheelConstants.kDefaultShootRPM)
            .withName("SpinUpDefaultAndWait");
    }

}