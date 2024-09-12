// FODC.java

package frc.robot.commands.swerve;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class FODC extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.FieldCentric drive;
    private final CommandXboxController driverController;
    private final double swerveSpeedMultiplier;
    private final boolean active;
    private double lastAngle = 0.0;
    private final int lineCount = 72;
    private final Pigeon2 pigeonIMU;

    public FODC(boolean active, CommandSwerveDrivetrain drivetrain, SwerveRequest.FieldCentric drive,
                CommandXboxController driverController, double swerveSpeedMultiplier, Pigeon2 pigeonIMU) {
        this.active = active;
        this.drivetrain = drivetrain;
        this.drive = drive;
        this.driverController = driverController;
        this.swerveSpeedMultiplier = swerveSpeedMultiplier;
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
    public void execute() {
        double rightStickX = applyDeadband(driverController.getRightX(), 0.05);
        double rightStickY = applyDeadband(driverController.getRightY(), 0.05);
        double angle;

        if (rightStickX != 0 || rightStickY != 0) {
            angle = Math.toDegrees(Math.atan2(-rightStickY, rightStickX));
            lastAngle = angle;
        } else {
            angle = lastAngle;
        }

        double snappedAngle = snapToNearestLine(angle, lineCount);
        double currentAngle = getRobotAngle();
        double angleError = snappedAngle - currentAngle;

        SmartDashboard.putNumber("Angle", angle);
        SmartDashboard.putNumber("Snapped Angle", snappedAngle);
        SmartDashboard.putNumber("Current Angle", currentAngle);
        SmartDashboard.putNumber("Angle Error", angleError);

        if (active) {
            drivetrain.applyRequest(() -> drive
                .withVelocityX(-driverController.getLeftY() * SwerveConstants.MaxSpeed * swerveSpeedMultiplier)
                .withVelocityY(-driverController.getLeftX() * SwerveConstants.MaxSpeed * swerveSpeedMultiplier)
                .withRotationalRate(angleError)); // Use angle error for rotational rate

            SmartDashboard.putNumber("Velocity X", -driverController.getLeftY() * SwerveConstants.MaxSpeed * swerveSpeedMultiplier);
            SmartDashboard.putNumber("Velocity Y", -driverController.getLeftX() * SwerveConstants.MaxSpeed * swerveSpeedMultiplier);
        }
        else {
            drivetrain.applyRequest(() -> drive
                .withVelocityX(-driverController.getLeftY() * SwerveConstants.MaxSpeed * swerveSpeedMultiplier)
                .withVelocityY(-driverController.getLeftX() * SwerveConstants.MaxSpeed * swerveSpeedMultiplier)
                .withRotationalRate(0));
        }
    }
}