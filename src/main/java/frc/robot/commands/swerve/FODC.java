// FODC.java

package frc.robot.commands.swerve;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
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
    private final int lineCount =  72; // Example line count for snapping

    public FODC(boolean active, CommandSwerveDrivetrain drivetrain, SwerveRequest.FieldCentric drive,
                CommandXboxController driverController,
                double swerveSpeedMultiplier) {
        this.active = active;
        this.drivetrain = drivetrain;
        this.drive = drive;
        this.driverController = driverController;
        this.swerveSpeedMultiplier = swerveSpeedMultiplier;

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

        SmartDashboard.putNumber("Angle", angle);
        SmartDashboard.putNumber("Snapped Angle", snappedAngle);

        if (active) {
            drivetrain.applyRequest(() -> drive
                .withVelocityX(-driverController.getLeftY() * SwerveConstants.MaxSpeed * swerveSpeedMultiplier)
                .withVelocityY(-driverController.getLeftX() * SwerveConstants.MaxSpeed * swerveSpeedMultiplier)
                .withRotationalRate(-driverController.getRightX() * SwerveConstants.MaxAngularRate * swerveSpeedMultiplier));

            SmartDashboard.putNumber("Velocity X", -driverController.getLeftY() * SwerveConstants.MaxSpeed * swerveSpeedMultiplier);
            SmartDashboard.putNumber("Velocity Y", -driverController.getLeftX() * SwerveConstants.MaxSpeed * swerveSpeedMultiplier);
        } else {
            new ParallelCommandGroup(
                    drivetrain.applyRequest(() -> drive
                            .withVelocityX(-driverController.getLeftY() * SwerveConstants.MaxSpeed * swerveSpeedMultiplier)
                            .withVelocityY(-driverController.getLeftX() * SwerveConstants.MaxSpeed * swerveSpeedMultiplier)
                            .withRotationalRate(-driverController.getRightX() *   SwerveConstants.MaxAngularRate * swerveSpeedMultiplier)),
                    new RepeatCommand(
                            new InstantCommand(this::printCurrentStickValues)));
        }
    }

    private void printCurrentStickValues() {
        SmartDashboard.putNumber("Left Y", driverController.getLeftY());
        SmartDashboard.putNumber("Left X", driverController.getLeftX());
        SmartDashboard.putNumber("Right X", driverController.getRightX());
        SmartDashboard.putNumber("Right Y", driverController.getRightY());
    }
}