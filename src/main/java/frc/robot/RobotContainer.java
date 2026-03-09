// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.diagonAlley.DiagonAlley;
import frc.robot.subsystems.diagonAlley.DiagonAlleyIO;
import frc.robot.subsystems.diagonAlley.DiagonAlleyIOSim;
import frc.robot.subsystems.diagonAlley.DiagonAlleyIOTalonFX;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.floorRollers.FloorRollers;
import frc.robot.subsystems.floorRollers.FloorRollersIO;
import frc.robot.subsystems.floorRollers.FloorRollersIOSim;
import frc.robot.subsystems.floorRollers.FloorRollersIOTalonFX;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodIO;
import frc.robot.subsystems.hood.HoodIOSim;
import frc.robot.subsystems.hood.HoodIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureIO;
import frc.robot.subsystems.superstructure.SuperstructureIOSim;
import frc.robot.subsystems.superstructure.SuperstructureIOSpark;
import frc.robot.subsystems.transfer.Transfer;
import frc.robot.subsystems.transfer.TransferIO;
import frc.robot.subsystems.transfer.TransferIOSim;
import frc.robot.subsystems.transfer.TransferIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  public final Drive drive;
  public final Superstructure superstructure;
  public final Vision vision;
  public final Intake intake;
  public final Hood hood;
  public final Shooter shooter;
  public final Transfer transfer;
  public final FloorRollers floorRollers;
  public final DiagonAlley diagonAlley;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder

        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(
                    VisionConstants.cameraName, VisionConstants.robotToCamera));

        // The ModuleIOTalonFXS implementation provides an example implementation for
        // TalonFXS controller connected to a CANdi with a PWM encoder. The
        // implementations
        // of ModuleIOTalonFX, ModuleIOTalonFXS, and ModuleIOSpark (from the Spark
        // swerve
        // template) can be freely intermixed to support alternative hardware
        // arrangements.
        // Please see the AdvantageKit template documentation for more information:
        // https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template#custom-module-implementations
        //
        // drive =
        // new Drive(
        // new GyroIOPigeon2(),
        // new ModuleIOTalonFXS(TunerConstants.FrontLeft),
        // new ModuleIOTalonFXS(TunerConstants.FrontRight),
        // new ModuleIOTalonFXS(TunerConstants.BackLeft),
        // new ModuleIOTalonFXS(TunerConstants.BackRight));

        superstructure = new Superstructure(new SuperstructureIOSpark() {});
        intake = new Intake(new IntakeIOTalonFX() {});
        hood = new Hood(new HoodIOTalonFX() {});
        shooter = new Shooter(new ShooterIOTalonFX() {});
        transfer = new Transfer(new TransferIOTalonFX() {});
        floorRollers = new FloorRollers(new FloorRollersIOTalonFX() {});
        diagonAlley = new DiagonAlley(new DiagonAlleyIOTalonFX() {});

        hood.rezero();
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    VisionConstants.cameraName, VisionConstants.robotToCamera, drive::getPose));

        superstructure = new Superstructure(new SuperstructureIOSim());
        intake = new Intake(new IntakeIOSim());
        hood = new Hood(new HoodIOSim());
        shooter = new Shooter(new ShooterIOSim());
        transfer = new Transfer(new TransferIOSim());
        floorRollers = new FloorRollers(new FloorRollersIOSim());
        diagonAlley = new DiagonAlley(new DiagonAlleyIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});

        superstructure = new Superstructure(new SuperstructureIO() {});
        intake = new Intake(new IntakeIO() {});
        hood = new Hood(new HoodIO() {});
        shooter = new Shooter(new ShooterIO() {});
        transfer = new Transfer(new TransferIO() {});
        floorRollers = new FloorRollers(new FloorRollersIO() {});
        diagonAlley = new DiagonAlley(new DiagonAlleyIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    autoChooser.addDefaultOption(
        "Drive to Start Pose",
        DriveCommands.autoDriveToPose(drive, Constants.autonomousDestination));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Lock to center of field when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickRotateToward(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () ->
                    (DriverStation.getAlliance().isPresent()
                                ? DriverStation.getAlliance().get()
                                : Alliance.Red)
                            == Alliance.Red
                        ? Constants.redHubTarget
                        : Constants.blueHubTarget));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    // Control bindings for superstructure
    // controller.leftBumper().whileTrue(superstructure.intake());
    // controller.rightBumper().whileTrue(superstructure.launch());

    // controller
    //     .leftBumper()
    //     .whileTrue(
    //         Commands.sequence(
    //             intake.goToDeployedPositionCommand(),
    //             intake.setRollerMotorPercentOutputCommand(0.5)));

    // Uncomment Above command and comment below command once lintake deploying is fixed

    // controller.leftBumper().onTrue(intake.setRollerMotorPercentOutputCommand(0.3));
    // controller.leftBumper().onFalse(intake.setRollerMotorPercentOutputCommand(0));

    // controller.leftBumper().onTrue(hood.CommandGoToAngle(0.43)); // Test angle
    // controller.leftBumper().onFalse(hood.CommandGoToLowestAngle());

    // controller.leftBumper().onTrue(shooter.setVelocityCommand(5)); // Test angle
    // controller.leftBumper().onFalse(shooter.setVelocityCommand(0));

    controller
        .leftBumper()
        .onTrue(
            Commands.parallel(
                transfer.manualRollBackward(0.6),
                floorRollers.rollInwardsCommand(0.7),
                shooter.setVelocityCommand(25),
                diagonAlley.rollOutwards(0.3)));
    controller
        .leftBumper()
        .onFalse(
            Commands.parallel(
                transfer.manualRollForwards(0),
                floorRollers.rollInwardsCommand(0),
                shooter.setVelocityCommand(0),
                diagonAlley.Break(0)));
    controller.leftBumper().onFalse(shooter.setVelocityCommand(0));

    // controller.rightBumper().whileTrue(intake.goToFramePerimeterPositionCommand());

    // controller.rightTrigger().whileTrue(superstructure.eject());
    controller
        .rightTrigger()
        .whileTrue(
            superstructure.shootOnTheFly(
                drive,
                hood,
                shooter,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () ->
                    (DriverStation.getAlliance().isPresent()
                                ? DriverStation.getAlliance().get()
                                : Alliance.Red)
                            == Alliance.Red
                        ? Constants.redHubTarget
                        : Constants.blueHubTarget));

    controller.y().onTrue(hood.runCurrentZeroing());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  /**
   * Use this to pass the current pose to the main {@link Robot} class.
   *
   * @return The current Pose2d
   */
  public Pose2d getPose2d() {
    return drive.getPose();
  }
}
