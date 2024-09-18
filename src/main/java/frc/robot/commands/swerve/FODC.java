package frc.robot.commands.swerve;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import java.util.function.DoubleSupplier;

public class FODC extends Command {
    private final CommandSwerveDrivetrain drivetrain;

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(SwerveConstants.MaxSpeed * 0.1)
            .withRotationalDeadband(SwerveConstants.MaxAngularRate * 0.1)
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);
    private final CommandXboxController driverController;
    private final double swerveSpeedMultiplier;
    private final boolean active;
    private double lastAngle = 0.0;
    private final int lineCount = 72;
    private final DoubleSupplier c_leftX, c_leftY;
    private final Pigeon2 pigeonIMU;
    private double lastOutput;

    public FODC(boolean active, CommandSwerveDrivetrain drivetrain, SwerveRequest.FieldCentric drive,
                CommandXboxController driverController , double swerveSpeedMultiplier , DoubleSupplier cLeftX , DoubleSupplier cLeftY , Pigeon2 pigeonIMU) {
        this.active = active;
        this.drivetrain = drivetrain;
        this.driverController = driverController;
        this.swerveSpeedMultiplier = swerveSpeedMultiplier;
        c_leftX = cLeftX;
        c_leftY = cLeftY;
        this.pigeonIMU = pigeonIMU;

        // Ensure the command requires the drivetrain subsystem
        addRequirements(drivetrain);
    }

    private double applyDeadband(double value, double deadband) {
        return Math.abs(value) > deadband ? value : 0.0;
    }

    private double snapToNearestLine(double angle, int lineCount) {
        double snapAngle = 360.0 / lineCount;
        return Math.round(angle / snapAngle) * snapAngle;
    }

    private double getRobotAngle() {
        return pigeonIMU.getAngle();
    }

    @Override
    public void initialize() {
        SmartDashboard.putBoolean("FODCCommand" , true);
        drivetrain.resetPID();
        double initialAngle = getRobotAngle();
        double snappedAngle = snapToNearestLine(initialAngle , lineCount);
        drivetrain.setTarget(snappedAngle);
    }

    @Override
    public void execute() {
        double rightStickX = applyDeadband(driverController.getRightX() , 0.10);
        double rightStickY = applyDeadband(driverController.getRightY() , 0.10);
        double angle;

        if ( rightStickX != 0 || rightStickY != 0 ) {
            angle = Math.toDegrees(Math.atan2(-rightStickY , rightStickX));
            lastAngle = angle;
        } else {
            angle = lastAngle;
        }

        double snappedAngle = snapToNearestLine(angle , lineCount);
        double currentAngle = getRobotAngle();
        double angleError = snappedAngle - currentAngle;

        SmartDashboard.putNumber("Angle" , angle);
        SmartDashboard.putNumber("Snapped Angle" , snappedAngle);
        SmartDashboard.putNumber("Current Angle" , currentAngle);
        SmartDashboard.putNumber("Angle Error" , angleError);
        SwerveConstants.AngleError = angleError;

        if ( Math.abs(angleError) <= 5.0 ) {
            // Release drive control system and allow free movement
            drivetrain.applyRequest(() -> drive
                    .withVelocityX(-driverController.getLeftY() * SwerveConstants.MaxSpeed * swerveSpeedMultiplier)
                    .withVelocityY(-driverController.getLeftX() * SwerveConstants.MaxSpeed * swerveSpeedMultiplier)
                    .withRotationalRate(0));
        } else {
            double output = drivetrain.getPIDRotation(SwerveConstants.AngleError , false);

            SmartDashboard.putNumber("Output" , output);
            SmartDashboard.putNumber("Error Value" , SwerveConstants.AngleError);

            drivetrain.setControl(drive
                    .withVelocityX(-c_leftX.getAsDouble() * 0.5)
                    .withVelocityY(-c_leftY.getAsDouble() * 0.5)
                    .withRotationalRate(output));
            lastOutput = output;
        }
    }

    @Override
    public boolean isFinished() {
        return drivetrain.getPIDAtSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("FODCCommand" , false);
    }
}