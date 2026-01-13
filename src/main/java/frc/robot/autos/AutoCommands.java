package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.shooterFlywheel.ShooterFlywheel;
import frc.robot.subsystems.shooterHood.ShooterHood;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooterFlywheel.ShooterFlywheelConstants;
import frc.robot.subsystems.shooterHood.ShooterHoodConstants;

public class AutoCommands {
    
    /**
     * AUTO 1: Do absolutely nothing (emergency fallback)
     */
    public static Command doNothing() {
        return Commands.none().withName("DoNothing");
    }
    
    /**
     * AUTO 2: Just drive out of the starting zone
     * Points: 3 (for leaving)
     */
    public static Command justLeave(SwerveDrive drive) {
        return Commands.sequence(
            Commands.print("AUTO: Just Leave"),
            Commands.waitSeconds(0.5),
            
            // Drive backwards at half speed for 2.5 seconds
            drive.driveFieldCentric(() -> -0.5, () -> 0.0, () -> 0.0)
                .withTimeout(2.5),
            
            // Stop
            drive.driveFieldCentric(() -> 0.0, () -> 0.0, () -> 0.0)
                .withTimeout(0.1)
        ).withName("JustLeave");
    }
    
    /**
     * AUTO 3: Shoot preloaded note, then leave
     * Points: 5 (speaker) + 3 (leave) = 8 total
     */
    public static Command shootAndLeave(
        ShooterFlywheel shooter,
        ShooterHood hood,
        Indexer indexer,
        SwerveDrive drive
    ) {
        return Commands.sequence(
            Commands.print("AUTO: Shoot and Leave"),
            
            // Step 1: Spin up shooter and set hood angle
            Commands.parallel(
                Commands.runOnce(() -> 
                    shooter.setVelocityRPM(ShooterFlywheelConstants.kDefaultShootRPM)),
                Commands.runOnce(() -> 
                    hood.setAngle(ShooterHoodConstants.kMidAngleDeg))
            ),
            
            // Step 2: Wait for shooter to reach speed (max 2 seconds)
            Commands.waitUntil(shooter::atSpeed).withTimeout(2.0),
            Commands.print("Shooter ready - firing!"),
            
            // Step 3: Feed the note
            indexer.runIndexerOnce().withTimeout(0.5),
            Commands.print("Note fired!"),
            
            // Step 4: Wait a moment
            Commands.waitSeconds(0.3),
            
            // Step 5: Drive backwards to leave zone
            drive.driveFieldCentric(() -> -0.5, () -> 0.0, () -> 0.0)
                .withTimeout(2.5),
            
            // Step 6: Stop everything
            Commands.parallel(
                Commands.runOnce(shooter::stop),
                Commands.runOnce(hood::stop),
                drive.driveFieldCentric(() -> 0.0, () -> 0.0, () -> 0.0)
                    .withTimeout(0.1)
            ),
            
            Commands.print("AUTO: Complete!")
        ).withName("ShootAndLeave");
    }
    
    /**
     * AUTO 4: Shoot, drive forward to get another note, drive back, shoot again
     * Points: 5 + 5 + 3 = 13 total (if you can grab another note)
     * WARNING: More complex, might not work consistently
     */
    public static Command shootTwice(
        ShooterFlywheel shooter,
        ShooterHood hood,
        Indexer indexer,
        SwerveDrive drive
    ) {
        return Commands.sequence(
            Commands.print("AUTO: Shoot Twice (RISKY)"),
            
            // Shoot first note (same as above)
            Commands.parallel(
                Commands.runOnce(() -> 
                    shooter.setVelocityRPM(ShooterFlywheelConstants.kDefaultShootRPM)),
                Commands.runOnce(() -> 
                    hood.setAngle(ShooterHoodConstants.kMidAngleDeg))
            ),
            Commands.waitUntil(shooter::atSpeed).withTimeout(2.0),
            indexer.runIndexerOnce().withTimeout(0.5),
            
            // Drive forward to pick up note
            Commands.print("Going for second note..."),
            drive.driveFieldCentric(() -> 0.6, () -> 0.0, () -> 0.0)
                .withTimeout(2.0),
            
            // Wait for intake (you'll need to add intake commands)
            Commands.waitSeconds(0.5),
            
            // Drive back to shooting position
            Commands.print("Returning to shoot..."),
            drive.driveFieldCentric(() -> -0.6, () -> 0.0, () -> 0.0)
                .withTimeout(2.0),
            
            // Shoot second note
            Commands.waitUntil(shooter::atSpeed).withTimeout(1.0),
            indexer.runIndexerOnce().withTimeout(0.5),
            
            // Leave zone
            drive.driveFieldCentric(() -> -0.5, () -> 0.0, () -> 0.0)
                .withTimeout(1.5),
            
            // Stop
            Commands.parallel(
                Commands.runOnce(shooter::stop),
                Commands.runOnce(hood::stop)
            ),
            
            Commands.print("AUTO: Complete!")
        ).withName("ShootTwice");
    }
}