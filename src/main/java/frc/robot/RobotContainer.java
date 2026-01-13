package frc.robot;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.climber.Climb;
import frc.robot.subsystems.drivetrain.Pigeon2Gyro;


import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.SwerveDriveConstants;
import frc.robot.subsystems.fuelIntakePivot.FuelIntakePivot;
import frc.robot.subsystems.fuelIntakeRoller.FuelIntakeRoller;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooterFlywheel.ShooterFlywheel;
import frc.robot.subsystems.shooterHood.ShooterHood;
import frc.robot.subsystems.shooterFlywheel.ShooterFlywheelConstants;
import frc.robot.subsystems.shooterHood.ShooterHoodConstants;



@Logged
public class RobotContainer {
    private SwerveDrive swerveDrive = new SwerveDrive();
    private FuelIntakePivot fuelIntakePivot = new FuelIntakePivot();
    private FuelIntakeRoller fuelIntakeRoller = new FuelIntakeRoller();
    private Indexer indexer = new Indexer();
    private final ShooterHood hood = new ShooterHood();
    private final ShooterFlywheel shooter = new ShooterFlywheel();
    private Climb climb = new Climb();

  
    // private final Pigeon2Gyro pigeon = new Pigeon2Gyro(30); // CAN ID 30

    
    //     public Pigeon2Gyro getPigeon() {
    //         return this.pigeon;
    //     }

    // gamepads
    private CommandXboxController driver = new CommandXboxController(0);
    private DoubleSupplier driverForward =
        () -> -MathUtil.applyDeadband(driver.getLeftY(), SwerveDriveConstants.kDriveDeadband)
         * SwerveDriveConstants.kJoyDriveSpeedFactor;

    private DoubleSupplier driverStrafe =
        () -> -MathUtil.applyDeadband(driver.getLeftX(), SwerveDriveConstants.kDriveDeadband)
            * SwerveDriveConstants.kJoyDriveSpeedFactor;

    private DoubleSupplier driverTurn =
        () -> -MathUtil.applyDeadband(driver.getRightX(), SwerveDriveConstants.kAngleDeadband)
            * SwerveDriveConstants.kJoyAngleSpeedFactor;

     public RobotContainer() {

        configureBindings1();
        configureBindings2();

       //  m_shooterFlywheel.setDefaultCommand(m_shooterFlywheel.set(0));

     }

    private void configureBindings1() {
      
      
      
        // call in a periodic or button action:

        // new RunCommand(() -> {
            
        // }, swerveDrive).schedule();
        
        // xButton.whileTrue(
        //     swerveDrive.GO()
        // );
        // xButton.whileFalse(
        //     swerveDrive.STOP()
        // );
        // xButton.whileTrue(swerveDrive.driveFieldCentric(
        //     1, 0, 0
        // ));
        // xButton.whileFalse(swerveDrive.driveFieldCentric(
        //     0, 0, 0
        // ));

        // Trigger lJoy = driver.leftStick();

        driver.x().whileTrue(swerveDrive.driveFieldCentric(
            driverForward, 
            driverStrafe, 
            driverTurn
        ));
        swerveDrive.setDefaultCommand(swerveDrive.driveFieldCentric(
            () -> 0, 
            () -> 0, 
            () -> 0
        ));



        // ========= CLIMB CONTROLS =========

        // driver.x().whileTrue(climb.climbDown()
        //     .repeatedly()
        //     .until(climb::atSetpoint));
        // driver.y().whileTrue(climb.climbUp()
        //     .repeatedly()
        //     .until(climb::atSetpoint));
        
        driver.a().whileTrue(climb.setVoltage1());
        driver.b().whileTrue(climb.setVoltageminus1());
        driver.start().whileTrue(climb.resetEncoder());
        climb.setDefaultCommand(climb.stop());


         // ==================== SHOOTING ====================
        
        // Keep flywheel always spinning at default speed for instant response
        shooter.setDefaultCommand(shooter.spinToDefault());
        
        // Right Bumper → SHOOT!
        // Complete shooting sequence: wait for speed → feed note
        driver.rightBumper().onTrue(
            Commands.sequence(
                // Ensure shooter is at target speed
                Commands.waitUntil(shooter::atSpeed).withTimeout(1.0),
                
                // Feed the note for 0.5 seconds
                indexer.runIndexerOnce().withTimeout(0.5),
                
                // Brief pause after feeding
                Commands.waitSeconds(0.1)
            ).withName("Shoot")
        );
        

        // ==================== HOOD ANGLE ADJUSTMENT ====================
        
        // DPad Up → Increase hood angle (shoot farther)
        driver.povUp().onTrue(
            Commands.parallel(
                hood.adjustAngleCommand(5.0),  // Increase by 5 degrees
                Commands.runOnce(() -> shooter.setVelocityRPM(ShooterFlywheelConstants.kFarShootRPM), shooter)
            ).withName("IncreaseShotDistance")
        );
        
        // DPad Down → Decrease hood angle (shoot closer)
        driver.povDown().onTrue(
            Commands.parallel(
                hood.adjustAngleCommand(-5.0),  // Decrease by 5 degrees
                Commands.runOnce(() -> shooter.setVelocityRPM(ShooterFlywheelConstants.kCloseShootRPM), shooter)
            ).withName("DecreaseShotDistance")
        );
        
        // DPad Left → Preset: Close shot
        driver.povLeft().onTrue(
            Commands.parallel(
                hood.setAngleCommand(ShooterHoodConstants.kCloseAngleDeg),
                Commands.runOnce(() -> shooter.setVelocityRPM(ShooterFlywheelConstants.kFarShootRPM), shooter)
            ).withName("CloseShot")
        );
        
        // DPad Right → Preset: Far shot
        driver.povRight().onTrue(
            Commands.parallel(
                hood.setAngleCommand(ShooterHoodConstants.kFarAngleDeg),
                Commands.runOnce(() -> shooter.setVelocityRPM(ShooterFlywheelConstants.kFarShootRPM), shooter)
            ).withName("FarShot")
        );

        // ==================== GYRO RESET ====================
        
        // Left Stick & Right Stick (pressed together) → Reset gyro
        driver.leftStick().and(driver.rightStick()).onTrue(
            Commands.runOnce(() -> {
                // swerveDrive.resetGyro();  // Uncomment when method exists
                System.out.println("Gyro reset!");
            }).withName("ResetGyro")
        );

      
    }

    private void configureBindings2() {

        
        
    }


   

    
}

