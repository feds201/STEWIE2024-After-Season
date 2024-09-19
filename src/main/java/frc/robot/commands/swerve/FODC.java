package frc.robot.commands.swerve;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.swerve.FODCSubsystem;

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
    private final DoubleSupplier c_rightX;
    private final DoubleSupplier c_rightY;

    private final FODCSubsystem fodcSubsystem;
    private double targetAngle;

    private final Pigeon2 pigeonIMU;
    private double lastOutput;

    public FODC(boolean active, CommandSwerveDrivetrain drivetrain, SwerveRequest.FieldCentric drive,
                CommandXboxController driverController , double swerveSpeedMultiplier , DoubleSupplier c_rightX , DoubleSupplier c_rightY , Pigeon2 pigeonIMU , FODCSubsystem fodcSubsystem) {
        this.active = active;
        this.drivetrain = drivetrain;
        this.driverController = driverController;
        this.swerveSpeedMultiplier = swerveSpeedMultiplier;
        this.pigeonIMU = pigeonIMU;
        this.fodcSubsystem = fodcSubsystem;
        this.c_rightX = c_rightX;
        this.c_rightY = c_rightY;

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

        if ( active ) {
            // Retrieve the filtered angle and angular velocity from the FODC subsystem
            double filteredAngle = fodcSubsystem.getFilteredAngle();
            double filteredAngularVelocity = fodcSubsystem.getFilteredAngularVelocity();

            // Example placeholder for updating the target angle
            // This can be based on joystick input or some other logic
            double rightStickX = applyDeadband(driverController.getRightX() , 0.10);
            double rightStickY = applyDeadband(driverController.getRightY() , 0.10);
            if ( rightStickX != 0 || rightStickY != 0 ) {
                targetAngle = Math.toDegrees(Math.atan2(-rightStickY , rightStickX));
            }


            // Snap target angle to nearest line
            double snappedAngle = snapToNearestLine(targetAngle , lineCount);
            double currentAngle = filteredAngle; // Use the filtered angle for comparison
            double angleError = snappedAngle - currentAngle;

            // Update SmartDashboard with the filtered values and calculated errors
            SmartDashboard.putNumber("Filtered Angle" , filteredAngle);
            SmartDashboard.putNumber("Filtered Angular Velocity" , filteredAngularVelocity);
            SmartDashboard.putNumber("Target Angle" , targetAngle);
            SmartDashboard.putNumber("Snapped Angle" , snappedAngle);
            SmartDashboard.putNumber("Current Angle" , currentAngle);
            SmartDashboard.putNumber("Angle Error" , angleError);

            // Apply the appropriate control logic based on the angle error
            if ( Math.abs(angleError) > 5.0 ) {
                // Significant error: Apply FODC logic to correct the orientation
                double rotationOutput = drivetrain.getPIDRotation(angleError , false);

                // Apply drivetrain control with rotational correction
                drivetrain.setControl(drive
                        .withVelocityX(-c_rightX.getAsDouble() * SwerveConstants.MaxSpeed * swerveSpeedMultiplier)
                        .withVelocityY(-c_rightY.getAsDouble() * SwerveConstants.MaxSpeed * swerveSpeedMultiplier)
                        .withRotationalRate(rotationOutput));
            } else {
                // Small error: Allow free movement (no rotational correction)
                drivetrain.setControl(drive
                        .withVelocityX(-c_rightX.getAsDouble() * SwerveConstants.MaxSpeed * swerveSpeedMultiplier)
                        .withVelocityY(-c_rightY.getAsDouble() * SwerveConstants.MaxSpeed * swerveSpeedMultiplier)
                        .withRotationalRate(0));
            }
        } else {
            new ParallelCommandGroup(
                    drivetrain.applyRequest(() -> drive
                            .withVelocityX(-driverController.getLeftY() * SwerveConstants.MaxSpeed * swerveSpeedMultiplier)
                            .withVelocityY(-driverController.getLeftX() * SwerveConstants.MaxSpeed * swerveSpeedMultiplier)
                            .withRotationalRate(0)));
        }
    }


    // Example method to dynamically adjust speed multiplier
    private double adjustSpeedMultiplierBasedOnPerformance(double filteredAngle , double angleError) {
        // Implement your logic to adjust speed multiplier based on performance metrics
        // This is a placeholder and should be customized based on your needs
        double baseMultiplier = 1.0;
        if ( Math.abs(angleError) > 10.0 ) {
            return baseMultiplier * 0.5; // Reduce speed if error is large
        } else {
            return baseMultiplier; // Normal speed
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