package frc.robot.subsystems.shooterHood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterHood extends SubsystemBase {
    
    private final ShooterHoodIO io;
    private final ShooterHoodInputs inputs = new ShooterHoodInputs();
    
    private final PIDController pid = new PIDController(
        ShooterHoodConstants.kP,
        ShooterHoodConstants.kI, 
        ShooterHoodConstants.kD
    );
    
    private double targetAngleDeg = 0.0;

    public ShooterHood() {
        this.io = new ShooterHoodIOSpark();
        pid.setTolerance(ShooterHoodConstants.kAngleToleranceDeg);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        // Only run PID if we have a target angle
        double volts = pid.calculate(inputs.angleDeg, targetAngleDeg) + ShooterHoodConstants.kG;
        if (volts != 0) {
            io.setVoltage(volts);
        }
        
        // Log to SmartDashboard
        SmartDashboard.putNumber("Hood/CurrentAngle", inputs.angleDeg);
        SmartDashboard.putNumber("Hood/TargetAngle", targetAngleDeg);
        SmartDashboard.putNumber("Hood/PID Output", volts);
        SmartDashboard.putNumber("Hood/Voltage", inputs.appliedVolts);
        SmartDashboard.putNumber("Hood/Current", inputs.currentAmps);
        SmartDashboard.putBoolean("Hood/AtTarget", atTarget());
    }

    /**
     * Set the hood to a specific angle (clamped to safe range).
     */
    public void setAngle(double angleDeg) {
        targetAngleDeg = MathUtil.clamp(
            angleDeg, 
            ShooterHoodConstants.kMinAngleDeg, 
            ShooterHoodConstants.kMaxAngleDeg
        );
    }

    /**
     * Adjust the current target angle by a delta.
     */
    public void adjustAngle(double deltaDeg) {
        setAngle(bound(targetAngleDeg + deltaDeg, 0, 5));
    }

    /**
     * bound x between lower and upper
     */
    private double bound(double x, double lower, double upper) {
        return Math.min(upper, Math.max(lower, x));
    }

    /**
     * Stop the hood motor.
     */
    public void stop() {
        targetAngleDeg = 0;
        io.stop();
        pid.reset();
    }

    /**
     * Check if hood is at target angle.
     */
    public boolean atTarget() {
        return pid.atSetpoint();
    }

    /**
     * Get the current hood angle.
     */
    public double getAngle() {
        return inputs.angleDeg;
    }

    /**
     * Get the target hood angle.
     */
    public double getTargetAngle() {
        return targetAngleDeg;
    }

    // ==================== COMMANDS ====================

    /**
     * Command to set the hood to a specific angle.
     */
    public Command setAngleCommand(double angleDeg) {
        return runOnce(() -> setAngle(angleDeg))
            .withName("SetHoodAngle_" + angleDeg);
    }

    /**
     * Command to manually adjust the hood angle by a delta.
     */
    public Command adjustAngleCommand(double deltaDeg) {
        return runOnce(() -> adjustAngle(deltaDeg))
            .withName("AdjustHood_" + deltaDeg);
    }

    /**
     * Command to stop the hood.
     */
    public Command stopCommand() {
        return Commands.runOnce(this::stop, this)
            .withName("StopHood");
    }

    /**
     * Command to set angle and wait until reached.
     */
    public Command setAngleAndWait(double angleDeg) {
        return Commands.sequence(
            setAngleCommand(angleDeg),
            Commands.waitUntil(this::atTarget)
        ).withTimeout(2.0)
         .withName("SetHoodAndWait_" + angleDeg);
    }
}