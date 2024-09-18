package frc.robot.subsystems.swerve;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import com.ctre.phoenix6.hardware.Pigeon2;
// import frc.robot.subsystems.swerve.generated.TunerConstants;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.swerve.generated.TunerConstants;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private static Pigeon2 pigeon = new Pigeon2(0);
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    public static final PIDController pidForVision = new PIDController(SwerveConstants.kRotationPForVision , SwerveConstants.kRotationIForVision , SwerveConstants.kRotationDForVision);
    public static final PIDController pidForSwerve = new PIDController(SwerveConstants.kRotationPForSwerve , SwerveConstants.kRotationIForSwerve , SwerveConstants.kRotationDForSwerve);
//    public static final PIDController pidForVision = new PIDController(0.04, .0009, .009);
//    public static final PIDController pidForVision =  new PIDController(.06135, .00, .00);


    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean hasAppliedOperatorPerspective = false;

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            SwerveModuleConstants... modules) {

        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }
        setupPIDControllerForVision();
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }
        setupPIDControllerForVision();
    }

    public void setupPIDControllerForVision() {
        pidForVision.setTolerance(1); // allowable angle error
        pidForVision.enableContinuousInput(0 , 360); // it is faster to go 1 degree from 359 to 0 instead of 359 degrees
        pidForVision.setIntegratorRange(-0.1 , 0.1);
        pidForVision.setSetpoint(0); // 0 = apriltag angle
    }

    public void setTarget(double target) {
        pidForVision.setSetpoint(target);
    }

    public boolean getPIDAtSetpoint() {
        return pidForVision.atSetpoint();
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public Command getAutoPath(String pathName) {
        return new PathPlannerAuto(pathName);
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    private void configurePathPlanner() {
        double driveBaseRadius = 0;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        AutoBuilder.configureHolonomic(
                () -> this.getState().Pose, // Supplier of current robot pose
                this::seedFieldRelative, // Consumer for seeding pose against auto
                this::getCurrentRobotChassisSpeeds,
                (speeds) -> this.setControl(autoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the
                                                                             // robot
                new HolonomicPathFollowerConfig(
                        new PIDConstants(10, 0, 0),
                        new PIDConstants(10, 0, 0),
                        TunerConstants.kSpeedAt12VoltsMps,
                        driveBaseRadius,
                        new ReplanningConfig()),
                () -> DriverStation.getAlliance().get() == Alliance.Red, // Change this if the path needs to be flipped
                                                                         // on red vs blue
                this); // Subsystem for requirements
    }

    public double getPIDRotation(double currentX , boolean isVision) {
        if ( isVision ) {
            return pidForVision.calculate(currentX);
        } else {
            return pidForSwerve.calculate(currentX);
        }
    }


    public void resetPID() {
        pidForVision.reset();
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    @Override
    public void periodic() {
        SmartDashboard.getNumber("PID:Error" , pidForVision.getPositionError());
        SmartDashboard.getBoolean("PID: ISthere" , pidForVision.atSetpoint());
        SmartDashboard.getNumber("PID: P" , pidForVision.getP());
        SmartDashboard.getNumber("PID: I" , pidForVision.getI());
        SmartDashboard.getNumber("PID: D" , pidForVision.getD());
        /* Periodically try to apply the operator perspective */
        /*
         * If we haven't applied the operator perspective before, then we should apply
         * it regardless of DS state
         */
        /*
         * This allows us to correct the perspective in case the robot code restarts
         * mid-match
         */
        /*
         * Otherwise, only check and apply the operator perspective if the DS is
         * disabled
         */
        /*
         * This ensures driving behavior doesn't change until an explicit disable event
         * occurs during testing
         */
        // if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
        // DriverStation.getAlliance().ifPresent((allianceColor) -> {
        // this.setOperatorPerspectiveForward(
        // allianceColor == Alliance.Red ? RedAlliancePerspectiveRotation
        // : BlueAlliancePerspectiveRotation);
        // hasAppliedOperatorPerspective = true;
        // });i
        // }


    }

    public Command SpeedPercentage(Double Speed) {
        return run(() -> SwerveConstants.speedpercentage = Speed);
    }
}
