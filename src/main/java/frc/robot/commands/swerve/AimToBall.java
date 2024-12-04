package frc.robot.commands.swerve;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision.VisionVariables;

// Moving Average Filter for smoothing
class MovingAverageFiltery {
    private final double[] window;
    private int size, index = 0;
    private double sum = 0.0;

    public MovingAverageFiltery(int size) {
        this.size = size;
        window = new double[ size ];
    }

    public double calculate(double newValue) {
        sum -= window[ index ]; // Remove oldest value from sum
        window[ index ] = newValue; // Add new value to window
        sum += newValue; // Add new value to sum
        index = (index + 1) % size; // Increment index and wrap around
        return sum / size; // Return the average
    }
}

public class AimToBall extends Command {
    private final CommandSwerveDrivetrain c_swerve;
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(SwerveConstants.MaxSpeed * 0.1)
            .withRotationalDeadband(SwerveConstants.MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final PIDController rotationalPID;
    private final PIDController strafePID;
    private final PIDController distancePID;
    private final DoubleSupplier c_limelightDistance;

    private MovingAverageFiltery X_filter;
    private MovingAverageFiltery Y_filter;

    public AimToBall(CommandSwerveDrivetrain swerve , DoubleSupplier limelightDistance) {
        c_swerve = swerve;
        c_limelightDistance = limelightDistance;
        X_filter = new MovingAverageFiltery(5);
        Y_filter = new MovingAverageFiltery(2);

        // Initialize PID controllers
        distancePID = new PIDController(2 , 0.0 , 0.0);
        strafePID = new PIDController(0.085 , 0.0 , 0);
        rotationalPID = new PIDController(.1 , 0.0 , 0);

        rotationalPID.setTolerance(1);
        strafePID.setTolerance(0);
        distancePID.setTolerance(0);

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        SmartDashboard.putBoolean("AimToBallCommand" , true);
        c_swerve.resetPID();
        rotationalPID.reset();
        strafePID.reset();
        distancePID.reset();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void execute() {
        double X_smooth = X_filter.calculate(VisionVariables.BackCam.target.getX());
        double Y_smooth = Y_filter.calculate(VisionVariables.BackCam.target.getDistance());
        // Calculate PID outputs

        double Output_linear = distancePID.calculate(Y_smooth);
        double Output_strafe = strafePID.calculate(X_smooth);
        double Output_rotation = (rotationalPID.calculate(X_smooth) - Output_strafe) - Output_strafe / 2; // basic principles of kinematics and dynamics

        // Log values to SmartDashboard

        SmartDashboard.putNumber("Limelight Smoothed X" , X_smooth);
        SmartDashboard.putNumber("Distance to Ball" , c_limelightDistance.getAsDouble());
        SmartDashboard.putNumber("Rotation Output" , Output_rotation);
        SmartDashboard.putNumber("Strafe Output" , Output_strafe);
        SmartDashboard.putNumber("X output" , Output_linear);


        // Send control to the swerve drivetrain
        c_swerve.setControl(drive
                .withVelocityX(Output_linear)
                .withVelocityY(-Output_strafe)
                .withRotationalRate(Output_rotation));
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("AimToBallCommand" , false);
    }
}
