package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.intake.RotateWristToPosition;
import frc.robot.commands.intake.RotateWristToPositionInfinite;
import frc.robot.commands.intake.RunIntakeWheels;
import frc.robot.commands.arm.RotateArmToPosition;
import frc.robot.commands.arm.RotateArmManual;
import frc.robot.commands.climber.ExtendClimber;
import frc.robot.commands.compound.*;
import frc.robot.commands.controller.ToggleRumble;
import frc.robot.commands.leds.SetLEDColor;
import frc.robot.commands.shooter.*;
import frc.robot.commands.swerve.AimToAprilTag;
import frc.robot.commands.swerve.FODC;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ClimberConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.OIConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.Vision.camera.Back_Camera;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.intake.IntakeIRSensor;
import frc.robot.subsystems.intake.IntakeWheels;
import frc.robot.subsystems.intake.Wrist;
import frc.robot.subsystems.shooter.ShooterServos;
import frc.robot.subsystems.shooter.ShooterIRSensor;
import frc.robot.subsystems.shooter.ShooterRotation;
import frc.robot.subsystems.shooter.ShooterWheels;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.generated.TunerConstants;
import frc.robot.subsystems.Vision.VisionVariables;
import frc.robot.utils.LimelightUtils;
import frc.robot.utils.Telemetry;

import static frc.robot.constants.IntakeConstants.kIntakeNoteWheelSpeed;

public class RobotContainer {
	public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;

	private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
			.withDeadband(SwerveConstants.MaxSpeed * 0.1)
			.withRotationalDeadband(SwerveConstants.MaxAngularRate * 0.1)
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage);

	private final SwerveRequest.FieldCentricFacingAngle autoAim = new SwerveRequest.FieldCentricFacingAngle()
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage);

	private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
	private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
	private final Telemetry logger = new Telemetry(SwerveConstants.MaxSpeed);
	private boolean EnableFODC;

	public double swerveSpeedMultiplier = 1;

	// SHOOTER
	private final ShooterWheels shooterWheels;
	private final ShooterRotation shooterRotation;
	private final ShooterServos servos;
	private final ShooterIRSensor shooterIRSensor;

	// INTAKE
	private final Wrist wrist;
	private final IntakeWheels intakeWheels;
	private final IntakeIRSensor intakeIRSensor;

	// ARM
	private final Arm arm;

	// Climber
	private final Climber climber;
	public final Leds leds;
	private final Back_Camera shooterSideCamera;
	public final CommandXboxController driverController;
	public final CommandXboxController operatorController;

	SendableChooser<Command> autonChooser = new SendableChooser<>();

	ShuffleboardTab commandsTab = Shuffleboard.getTab("commands");

	public RobotContainer() {
		// ARM
		arm = new Arm();

		// SHOOTER
		shooterWheels = new ShooterWheels();
		shooterRotation = new ShooterRotation(arm::getArmAngle);
		servos = new ShooterServos();
		shooterIRSensor = new ShooterIRSensor();

		// CLIMBER
		climber = new Climber();

		// INTAKE
		wrist = new Wrist();
		intakeWheels = new IntakeWheels();
		intakeIRSensor = new IntakeIRSensor();

		// LEDS
		leds = new Leds();

		// CAMERAS
		shooterSideCamera = new Back_Camera();

		// CONTROLLERS
		driverController = new CommandXboxController(OIConstants.kDriverController);
		operatorController = new CommandXboxController(OIConstants.kOperatorController);

		// Add subsystems to their respective shuffleboard tabs
		arm.getShuffleboardTab().add("arm", arm);
		shooterWheels.getShuffleboardTab().add("shooter wheels", shooterWheels);
		shooterRotation.getShuffleboardTab().add("shooter rotation", shooterRotation);
		climber.getShuffleboardTab().add("climber", climber);
		wrist.getShuffleboardTab().add("wrist", wrist);
		intakeWheels.getShuffleboardTab().add("wheels", intakeWheels);
		shooterWheels.getShuffleboardTab().add("servo", servos);

		configureDefaultCommands();
		configureDriverController();
		configureOperatorController();

		setupArmCommands();
		setupClimberCommands();
		setupIntakeCommands();
		setupShooterCommands();
		setupErrorTriggers();
		setupAutonCommands();

		Shuffleboard.getTab("autons").add(autonChooser);
		setupAutonPaths();

	}

	private void configureFODC(boolean status) {
		drivetrain.setDefaultCommand(new FODC(status, drivetrain, drive, driverController,
				swerveSpeedMultiplier, drivetrain.getPigeon2()));

	}

	private void setupAutonCommands() {
		// AUTON NAMED COMMANDS
		NamedCommands.registerCommand("shootNote",
				new ShootFromHandoff(shooterRotation, shooterWheels, servos, leds,
						() -> VisionVariables.ExportedVariables.Distance, shooterIRSensor));

		NamedCommands.registerCommand("intakeNote",
				new DeployIntake(wrist, intakeWheels, shooterRotation, intakeIRSensor, leds,
						driverController,
						operatorController));

		NamedCommands.registerCommand("handoffToShooter", new ParallelCommandGroup(
				new RotateArmToPosition(arm, () -> 0),
				new AlignShooterAndIntake(shooterRotation, wrist, intakeWheels,
						servos, shooterIRSensor, leds)));

		NamedCommands.registerCommand("fieldCentric", drivetrain.runOnce(drivetrain::seedFieldRelative));

		NamedCommands.registerCommand("stopShooter",
				new ShootNoteMotionMagicVelocity(shooterWheels, () -> 0, () -> 0));

		// Create and add autonomous commands to the chooser

	}

	private void setupAutonPaths() {
		Command straightLineAuto = drivetrain.getAutoPath("straightLineAuto");
		Command testPath = drivetrain.getAutoPath("testPath");
		Command hello = drivetrain.getAutoPath("Hello");

		autonChooser.addOption("Straight Back 2 note", straightLineAuto);
		autonChooser.addOption("Test Path", testPath);
		autonChooser.addOption("Hello", hello);
	}

	private void configureDefaultCommands() {

		if (EnableFODC) {
			configureFODC(EnableFODC);
		} else {
			configureFODC(EnableFODC);

		}
		if (Utils.isSimulation()) {
			drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
		}

		drivetrain.registerTelemetry(logger::telemeterize);

		arm.setDefaultCommand(new RotateArmManual(arm, () -> -operatorController.getLeftY()));

		new Trigger(() -> arm.getArmAngle() > ArmConstants.kArmClimbLimit)
				.onTrue(new InstantCommand(() -> leds.setLedColor(Leds.LedColors.YELLOW)))
				.onFalse(new InstantCommand(() -> leds.setLedColor(Leds.getAllianceColor())));

		shooterWheels.setDefaultCommand(new ShootNoteMotionMagicVelocity(shooterWheels, () -> 0, () -> 0));
	}

	private void configureDriverController() {
		// Reset field-centric heading on left bumper press
		driverController.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldRelative));

		// Left trigger activates DeployIntake, resets intake and toggles rumble on
		// release
		driverController.leftTrigger()
				.onTrue(new DeployIntake(wrist, intakeWheels, shooterRotation, intakeIRSensor, leds,
						driverController, operatorController))
				.onFalse(new ParallelCommandGroup(
						new ResetIntake(wrist, intakeWheels),
						new ToggleRumble(driverController, 0),
						new ToggleRumble(operatorController, 0),
						new SequentialCommandGroup(
								new WaitCommand(1.5),
								new SetLEDColor(leds, Leds.LedColors.WHITE))
								.onlyIf(intakeIRSensor::getBeamBroken)));

		// Right trigger activates SpitOutNote, resets intake on release
		driverController.rightTrigger()
				.onTrue(new SpitOutNote(wrist, intakeWheels))
				.onFalse(new ResetIntake(wrist, intakeWheels));

		// Right bumper runs intake wheels, stops them on release
		driverController.rightBumper()
				.onTrue(new RunIntakeWheels(intakeWheels, () -> kIntakeNoteWheelSpeed))
				.onFalse(new RunIntakeWheels(intakeWheels, () -> 0));

		// Button 'A' rotates shooter to feeder setpoint
		driverController.a().onTrue(new RotateShooterToPosition(shooterRotation,
				() -> ShooterConstants.RotationPIDForExternalEncoder.kShooterRotationFeederSetpoint));

		// Button 'X' executes shooter and LED sequence
		driverController.x().onTrue(new SequentialCommandGroup(
				new ParallelDeadlineGroup(
						new WaitCommand(2),
						new RotateShooterToPosition(shooterRotation,
								() -> ShooterConstants.RotationPIDForExternalEncoder.kShooterHorizontal),
						new ShootNoteMotionMagicVelocity(shooterWheels,
								() -> LimelightUtils.GetSpeedTop(
										VisionVariables.ExportedVariables.Distance),
								() -> LimelightUtils.GetSpeedBottom(
										VisionVariables.ExportedVariables.Distance)),
						new WaitCommand(0.7),
						new EjectNote(servos),
						new SetLEDColor(leds, Leds.getAllianceColor())),
				new ShootNoteMotionMagicVelocity(shooterWheels, () -> 0, () -> 0),
				new RotateShooterToPosition(shooterRotation,
						() -> ShooterConstants.RotationPIDForExternalEncoder.kShooterRotationFeederSetpoint)))
				.onFalse(new ParallelCommandGroup(
						new RotateShooterToPosition(shooterRotation,
								() -> ShooterConstants.RotationPIDForExternalEncoder.kShooterRotationFeederSetpoint),
						new ShootNoteMotionMagicVelocity(shooterWheels, () -> 0, () -> 0)));

		// Left bumper changes swerve speed multiplier
		driverController.leftBumper()
				.onTrue(new InstantCommand(() -> swerveSpeedMultiplier = 0.2))
				.onFalse(new InstantCommand(() -> swerveSpeedMultiplier = 1));
	}

	public void configureOperatorController() {
		// Left bumper loads position and aligns shooter and intake
		operatorController.leftBumper()
				.onTrue(new ParallelCommandGroup(
						new RotateArmToPosition(arm, () -> 0),
						new AlignShooterAndIntake(shooterRotation, wrist, intakeWheels, servos,
								shooterIRSensor, leds)));

		// Right trigger aims and activates wrist position, LEDs, and rumble
		operatorController.rightTrigger()
				.onTrue(new SequentialCommandGroup(
						new ParallelCommandGroup(
								new AimToAprilTag(drivetrain,
										() -> -driverController.getLeftX(),
										() -> -driverController.getLeftY(),
										() -> VisionVariables.ExportedVariables.Distance)
										.andThen(new ParallelCommandGroup(
												new SetLEDColor(leds,
														Leds.LedColors.VIOLET),
												new ToggleRumble(
														driverController,
														0.3),
												new ToggleRumble(
														operatorController,
														0.3))),
								new RotateWristToPositionInfinite(wrist,
										IntakeConstants.WristPID.kWristOutOfTheWay))))
				.onFalse(new ParallelDeadlineGroup(
						new WaitCommand(0.2),
						drivetrain.applyRequest(() -> brake),
						new RotateWristToPosition(wrist,
								IntakeConstants.WristPID.kWristShooterFeederSetpoint)));

		// Left trigger shoots from handoff and activates rumble
		operatorController.leftTrigger()
				.onTrue(new ParallelCommandGroup(
						new RotateWristToPositionInfinite(wrist,
								IntakeConstants.WristPID.kWristOutOfTheWay),
						new ShootFromHandoff(shooterRotation, shooterWheels, servos, leds,
								() -> VisionVariables.ExportedVariables.Distance,
								shooterIRSensor))
						.andThen(new ParallelCommandGroup(
								new ToggleRumble(driverController, 0.3),
								new ToggleRumble(operatorController, 0.3))))
				.onFalse(new ParallelCommandGroup(
						new SetLEDColor(leds, Leds.getAllianceColor()),
						new RotateShooterToPosition(shooterRotation,
								() -> ShooterConstants.RotationPIDForExternalEncoder.kShooterRotationFeederSetpoint),
						new ShootNoteMotionMagicVelocity(shooterWheels, () -> 0, () -> 0),
						new ResetIntake(wrist, intakeWheels),
						new ToggleRumble(driverController, 0),
						new ToggleRumble(operatorController, 0)));

		// Button 'A' places object and activates LED and rumble feedback
		operatorController.a()
				.onTrue(new PlaceInAmp(wrist, intakeWheels, arm, leds, shooterRotation)
						.andThen(new ParallelCommandGroup(
								new SetLEDColor(leds, leds.getLedColor()),
								new ToggleRumble(driverController, 0.3),
								new ToggleRumble(operatorController, 0.3))))
				.onFalse(new ParallelCommandGroup(
						new RotateWristToPosition(wrist,
								IntakeConstants.WristPID.kWristIdlePosition),
						new RotateArmToPosition(arm, () -> 0),
						new RunIntakeWheels(intakeWheels, () -> 0)));

		// Button 'X' rotates wrist to shooter feeder setpoint
		operatorController.x()
				.onTrue(new RotateWristToPosition(wrist,
						IntakeConstants.WristPID.kWristShooterFeederSetpoint));
	}

	private void setupErrorTriggers() {
		// There should be some feedback for an "failure mode" but rumbling the
		// controller continuously was obnoxious lol.
	}

	public Command getAutonomousCommand() {
		return autonChooser.getSelected(); // runAuto;
		// return null;
	}

	private void setupIntakeCommands() {
		intakeWheels.getShuffleboardTab().add("Intake Wheels",
				new RunIntakeWheels(intakeWheels, () -> kIntakeNoteWheelSpeed));

		intakeWheels.getShuffleboardTab().add("Reverse Intake Wheels",
				new RunIntakeWheels(intakeWheels, () -> -kIntakeNoteWheelSpeed));

		wrist.getShuffleboardTab().add("Note Position",
				new RotateWristToPosition(wrist,
						IntakeConstants.WristPID.kWristNotePosition));

		wrist.getShuffleboardTab().add("Idle Position",
				new RotateWristToPosition(wrist,
						IntakeConstants.WristPID.kWristIdlePosition));

		wrist.getShuffleboardTab().add("Shooter Position",
				new RotateWristToPosition(wrist,
						IntakeConstants.WristPID.kWristShooterFeederSetpoint));

		wrist.getShuffleboardTab().add("Rotate until note in intake",
				new SequentialCommandGroup(
						new RotateWristToPosition(wrist,
								IntakeConstants.WristPID.kWristNotePosition),
						new IntakeUntilNoteIn(intakeWheels, intakeIRSensor, leds,
								driverController, operatorController),
						new RotateWristToPosition(wrist,
								IntakeConstants.WristPID.kWristShooterFeederSetpoint)));
	}

	private void setupArmCommands() {
		arm.getShuffleboardTab().add("Rotate Arm",
				new RotateArmToPosition(arm,
						() -> ArmConstants.ArmPIDForExternalEncoder.kArmRotationFeederSetpoint));
	}

	private void setupShooterCommands() {
		ShuffleboardTab shooterTab = shooterWheels.getShuffleboardTab();

		shooterTab.add("100 RPS Shoot",
				new ShootNoteMotionMagicVelocity(shooterWheels,
						() -> ShooterConstants.kShootVelocity,
						() -> ShooterConstants.kShootVelocity));

		shooterTab.add("0 RPS Shoot",
				new ShootNoteMotionMagicVelocity(shooterWheels,
						() -> 0, () -> 0));

		shooterTab.add("-15 Deg Rotate", new RotateShooterToPosition(shooterRotation,
				() -> ShooterConstants.RotationPIDForExternalEncoder.kArm60InchSetpoint));

		shooterTab.add("-30 Deg Rotate",
				new RotateShooterToPosition(shooterRotation, () -> -30));

		shooterTab.add("Spin Servos", new EjectNote(servos));

		shooterTab.add("Stop Servos", new StopServos(servos));

		shooterTab.add("Shoot Note Full Command",
				new ShootNoteAtSpeakerOnly(shooterRotation, shooterWheels, servos, leds,
						() -> VisionVariables.ExportedVariables.Distance, shooterIRSensor));
	}

	private void setupClimberCommands() {
		;
		climber.getShuffleboardTab().add("Climb Simple",
				new ExtendClimber(climber,
						() -> ClimberConstants.kClimberSpeed));
	}

}
