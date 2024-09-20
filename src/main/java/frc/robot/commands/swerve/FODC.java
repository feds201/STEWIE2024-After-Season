package frc.robot.commands.swerve;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.FODCSubsystem;

import java.util.function.DoubleSupplier;

public class FODC extends Command {
    private final CommandSwerveDrivetrain drivetrain;

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(SwerveConstants.MaxSpeed * 0.1)
            .withRotationalDeadband(SwerveConstants.MaxAngularRate * 0.1)
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);
    private final double swerveSpeedMultiplier;
    private final boolean active;
    private double lastAngle = 0.0;
    private final int lineCount = 72;
    private final DoubleSupplier driverRightStickX;
    private final DoubleSupplier driverRightStickY;

    private final DoubleSupplier driverLeftStickX;
    private final DoubleSupplier driverLeftStickY;


    private final FODCSubsystem fodcSubsystem;

    private double targetAngle;

    private final ShuffleboardTab tab = Shuffleboard.getTab("FODC");


    private final Pigeon2 pigeonIMU;
    private double lastOutput;


    private double filteredAngle;
    private double filteredAngularVelocity;

    private double dynamicSpeedMultiplier;
    private double snappedAngle;
    private double currentAngle;
    private double angleError;


    public FODC(
            boolean active , CommandSwerveDrivetrain drivetrain , SwerveRequest.FieldCentric drive , double swerveSpeedMultiplier ,
            DoubleSupplier driverRightStickX , DoubleSupplier driverRightStickY , DoubleSupplier driverLeftStickX , DoubleSupplier driverLeftStickY ,
            Pigeon2 pigeonIMU , FODCSubsystem fodcSubsystem) {
        this.active = active;
        this.drivetrain = drivetrain;
        this.swerveSpeedMultiplier = swerveSpeedMultiplier;
        this.driverLeftStickX = driverLeftStickX;
        this.driverLeftStickY = driverLeftStickY;
        this.pigeonIMU = pigeonIMU;
        this.driverRightStickX = driverRightStickX;
        this.driverRightStickY = driverRightStickY;
        this.fodcSubsystem = fodcSubsystem;

        // Ensure the command requires the drivetrain subsystem
        addRequirements(drivetrain);
    }

    private double applyDeadband(double value) {
        return Math.abs(value) > 0.1 ? value : 0.0;
    }

    private double snapToNearestLine(double angle) {
        double snapAngle = 360.0 / 72;
        return Math.round(angle / snapAngle) * snapAngle;
    }

    private double getRobotAngle() {
        return pigeonIMU.getAngle();
    }

    @Override
    public void initialize() {
        tab.addBoolean("FODC  Command" , () -> true);
        drivetrain.resetPID();
        double initialAngle = getRobotAngle();
        double snappedAngle = snapToNearestLine(initialAngle);
        drivetrain.setTarget(snappedAngle);
        tab.addDouble("Dynamic Speed Multiplier" , () -> dynamicSpeedMultiplier);
        tab.addDouble("Angle Error" , () -> angleError);
        tab.addDouble("Filtered Angle" , () -> filteredAngle);
        tab.addDouble("Filtered Angular Velocity" , () -> filteredAngularVelocity);
        tab.addDouble("Target Angle" , () -> targetAngle);
        tab.addDouble("Snapped Angle" , () -> snappedAngle);
        tab.addDouble("Current Angle" , () -> currentAngle);
        PIDController pidController = new PIDController(SwerveConstants.kRotationPForSwerve , SwerveConstants.kRotationIForSwerve , SwerveConstants.kRotationDForSwerve);
        pidController.setTolerance(SwerveConstants.kAlignmentTolerance);

        tab.add("FODC Controller" , pidController);

    }


    @Override
    public void execute() {

        if ( active ) {
            // Retrieve the filtered angle and angular velocity from the FODC subsystem
            filteredAngle = fodcSubsystem.getFilteredAngle();
            filteredAngularVelocity = fodcSubsystem.getFilteredAngularVelocity();

            // Example placeholder for updating the target angle
            // This can be based on joystick input or some other logic
            double rightStickX = applyDeadband((driverRightStickX.getAsDouble()));
            double rightStickY = applyDeadband((driverRightStickY.getAsDouble()));
            if ( rightStickX != 0 || rightStickY != 0 ) {
                targetAngle = Math.toDegrees(Math.atan2(-rightStickY , rightStickX));
            }


            // Snap target angle to nearest line
            snappedAngle = snapToNearestLine(targetAngle);
            currentAngle = getRobotAngle();
            angleError = snappedAngle - currentAngle;

            // Update SmartDashboard with the filtered values and calculated errors
            SmartDashboard.putNumber("Filtered Angle" , filteredAngle);
            SmartDashboard.putNumber("Filtered Angular Velocity" , filteredAngularVelocity);
            SmartDashboard.putNumber("Target Angle" , targetAngle);
            SmartDashboard.putNumber("Snapped Angle" , snappedAngle);
            SmartDashboard.putNumber("Current Angle" , currentAngle);
            SmartDashboard.putNumber("Angle Error" , angleError);

            double velocityMultiplier = adjustSpeedMultiplierBasedOnPerformance(filteredAngle , angleError , filteredAngularVelocity);
            // Apply the appropriate control logic based on the angle error
            if ( Math.abs(angleError) > 5.0 ) {
                // Significant error: Apply FODC logic to correct the orientation
                double rotationOutput = drivetrain.getPIDRotation(angleError , false);

                // Apply drivetrain control with rotational correction
                drivetrain.setControl(drive
                        .withVelocityX(-driverLeftStickY.getAsDouble() * SwerveConstants.MaxSpeed * swerveSpeedMultiplier * velocityMultiplier)
                        .withVelocityY(-driverLeftStickY.getAsDouble() * SwerveConstants.MaxSpeed * swerveSpeedMultiplier * velocityMultiplier)
                        .withRotationalRate(rotationOutput));
            } else {
                // Small error: Allow free movement (no rotational correction)
                drivetrain.setControl(drive
                        .withVelocityX(-driverLeftStickY.getAsDouble() * SwerveConstants.MaxSpeed * swerveSpeedMultiplier * velocityMultiplier)
                        .withVelocityY(-driverLeftStickX.getAsDouble() * SwerveConstants.MaxSpeed * swerveSpeedMultiplier * velocityMultiplier)
                        .withRotationalRate(0));
            }

        } else {
            new ParallelCommandGroup(
                    drivetrain.applyRequest(() -> drive
                            .withVelocityX(-driverLeftStickY.getAsDouble() * SwerveConstants.MaxSpeed * swerveSpeedMultiplier)
                            .withVelocityY(-driverLeftStickX.getAsDouble() * SwerveConstants.MaxSpeed * swerveSpeedMultiplier)
                            .withRotationalRate(0)));
        }
    }


    private double adjustSpeedMultiplierBasedOnPerformance(double filteredAngle , double angleError , double filteredAngularVelocity) {
        double baseMultiplier = 1.0; // Default speed multiplier
        double adjustedMultiplier;

        // If the angle error is large, reduce the speed multiplier
        if ( Math.abs(angleError) > 15.0 ) {
            adjustedMultiplier = baseMultiplier * 0.5; // Reduce speed significantly for large errors
        } else if ( Math.abs(angleError) > 5.0 ) {
            adjustedMultiplier = baseMultiplier * 0.75; // Reduce speed slightly for medium errors
        } else {
            adjustedMultiplier = baseMultiplier; // Normal speed for small errors
        }

        // Optionally, reduce speed further if angular velocity is too high
        if ( Math.abs(filteredAngularVelocity) > 50.0 ) {
            adjustedMultiplier *= 0.9; // Reduce speed by 10% if angular velocity is high
        }

        return adjustedMultiplier;
    }



    @Override
    public boolean isFinished() {
        return false; // Always return false to keep the command running
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("FODCCommand" , false);
    }
}


//    @Override
//    public void execute() {
//
//        if ( active ) {
//            // Retrieve the filtered angle and angular velocity from the FODC subsystem
//            double filteredAngle = fodcSubsystem.getFilteredAngle();
//            double filteredAngularVelocity = fodcSubsystem.getFilteredAngularVelocity();
//
//            // Example placeholder for updating the target angle
//            // This can be based on joystick input or some other logic
//            double rightStickX = applyDeadband(driverController.getRightX() , 0.10);
//            double rightStickY = applyDeadband(driverController.getRightY() , 0.10);
//            if ( rightStickX != 0 || rightStickY != 0 ) {
//                targetAngle = Math.toDegrees(Math.atan2(-rightStickY , rightStickX));
//            }
//
//
//            // Snap target angle to nearest line
//            double snappedAngle = snapToNearestLine(targetAngle , lineCount);
//            double currentAngle = filteredAngle; // Use the filtered angle for comparison
//            double angleError = snappedAngle - currentAngle;
//
//            // Update SmartDashboard with the filtered values and calculated errors
//            SmartDashboard.putNumber("Filtered Angle" , filteredAngle);
//            SmartDashboard.putNumber("Filtered Angular Velocity" , filteredAngularVelocity);
//            SmartDashboard.putNumber("Target Angle" , targetAngle);
//            SmartDashboard.putNumber("Snapped Angle" , snappedAngle);
//            SmartDashboard.putNumber("Current Angle" , currentAngle);
//            SmartDashboard.putNumber("Angle Error" , angleError);
//
//            // Apply the appropriate control logic based on the angle error
//            if ( Math.abs(angleError) > 5.0 ) {
//                // Significant error: Apply FODC logic to correct the orientation
//                double rotationOutput = drivetrain.getPIDRotation(angleError , false);
//
//                // Apply drivetrain control with rotational correction
//                drivetrain.setControl(drive
//                        .withVelocityX(-driverRightStickX.getAsDouble() * SwerveConstants.MaxSpeed * swerveSpeedMultiplier)
//                        .withVelocityY(-driverRightStickY.getAsDouble() * SwerveConstants.MaxSpeed * swerveSpeedMultiplier)
//                        .withRotationalRate(rotationOutput));
//            } else {
//                // Small error: Allow free movement (no rotational correction)
//                drivetrain.setControl(drive
//                        .withVelocityX(-driverRightStickX.getAsDouble() * SwerveConstants.MaxSpeed * swerveSpeedMultiplier)
//                        .withVelocityY(-driverRightStickY.getAsDouble() * SwerveConstants.MaxSpeed * swerveSpeedMultiplier)
//                        .withRotationalRate(0));
//            }
//        } else {
//            new ParallelCommandGroup(
//                    drivetrain.applyRequest(() -> drive
//                            .withVelocityX(-driverController.getLeftY() * SwerveConstants.MaxSpeed * swerveSpeedMultiplier)
//                            .withVelocityY(-driverController.getLeftX() * SwerveConstants.MaxSpeed * swerveSpeedMultiplier)
//                            .withRotationalRate(0)));
//        }
//    }

//    @Override
//    public void execute() {
//
//        if (active) {
//            // Retrieve the filtered angle and angular velocity from the FODC subsystem
//             filteredAngle = fodcSubsystem.getFilteredAngle();
//             filteredAngularVelocity = fodcSubsystem.getFilteredAngularVelocity();
//
//            // Calculate the target angle based on the joystick input
//            rightStickX = applyDeadband(driverController.getRightX(), 0.10);
//            rightStickY = applyDeadband(driverController.getRightY(), 0.10);
//            if (rightStickX != 0 || rightStickY != 0) {
//                targetAngle = Math.toDegrees(Math.atan2(-rightStickY, rightStickX));
//            }
//
//            // Snap target angle to nearest line
//             snappedAngle = snapToNearestLine(targetAngle, lineCount);
//             currentAngle = filteredAngle;
//             angleError = snappedAngle - currentAngle;
//
//            // Calculate dynamic speed multiplier based on performance
//            dynamicSpeedMultiplier = adjustSpeedMultiplierBasedOnPerformance(filteredAngle, angleError, filteredAngularVelocity);
//
//
//
//
//            // Apply the appropriate control logic based on the angle error
//            if (Math.abs(angleError) > 5.0) {
//                // Significant error: Apply rotational correction
//                double rotationOutput = drivetrain.getPIDRotation(angleError, false);
//
//                // Use dynamic speed multiplier in the drivetrain control
//                drivetrain.setControl(drive
//                        .withVelocityX(-driverRightStickX.getAsDouble() * SwerveConstants.MaxSpeed * dynamicSpeedMultiplier)
//                        .withVelocityY(-driverRightStickY.getAsDouble() * SwerveConstants.MaxSpeed * dynamicSpeedMultiplier)
//                        .withRotationalRate(rotationOutput));
//            } else {
//                // Small error: Allow free movement (no rotational correction)
//                drivetrain.setControl(drive
//                        .withVelocityX(-driverRightStickX.getAsDouble() * SwerveConstants.MaxSpeed * dynamicSpeedMultiplier)
//                        .withVelocityY(-driverRightStickY.getAsDouble() * SwerveConstants.MaxSpeed * dynamicSpeedMultiplier)
//                        .withRotationalRate(0));
//            }
//        } else {
//            new ParallelCommandGroup(
//                    drivetrain.applyRequest(() -> drive
//                            .withVelocityX(-driverController.getLeftY() * SwerveConstants.MaxSpeed * swerveSpeedMultiplier)
//                            .withVelocityY(-driverController.getLeftX() * SwerveConstants.MaxSpeed * swerveSpeedMultiplier)
//                            .withRotationalRate(0)));
//        }
//    }