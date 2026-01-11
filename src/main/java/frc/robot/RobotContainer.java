package frc.robot;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.fuelIntakePivot.FuelIntakePivot;
import frc.robot.subsystems.fuelIntakeRoller.FuelIntakeRoller;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooterFlywheel.ShooterFlywheel;
import frc.robot.subsystems.shooterHood.ShooterHood;

@Logged
public class RobotContainer {
    private SwerveDrive swerveDrive = new SwerveDrive();
    private FuelIntakePivot fuelIntakePivot = new FuelIntakePivot();
    private FuelIntakeRoller fuelIntakeRoller = new FuelIntakeRoller();
    private Indexer indexer = new Indexer();
    private ShooterHood shooterHood = new ShooterHood();
    private final ShooterFlywheel m_shooterFlywheel = new ShooterFlywheel();
    private Climber climber = new Climber();

     private final CommandXboxController m_driverController =
      new CommandXboxController(ControllerConstants.kDriverControllerPort);

     public RobotContainer() {

        configureBindings();

        m_shooterFlywheel.setDefaultCommand(m_shooterFlywheel.set(0));

     }

    private void configureBindings() {
        // Configure your button bindings here
         // Schedule `setVelocity` when the Xbox controller's B button is pressed,
        // cancelling on release.
        m_driverController.a().whileTrue(m_shooterFlywheel.setSpeed(RotationsPerSecond.of(60)));
        m_driverController.b().whileTrue(m_shooterFlywheel.setSpeed(RotationsPerSecond.of(300)));
        // Schedule `set` when the Xbox controller's B button is pressed,
        // cancelling on release.
        m_driverController.x().whileTrue(m_shooterFlywheel.set(0.3));
        m_driverController.y().whileTrue(m_shooterFlywheel.set(-0.3));


    }}
