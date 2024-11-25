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
    private final PIDController xPID;
    private final DoubleSupplier c_limelightDistance;

    private final MovingAverageFiltery xFilter = new MovingAverageFiltery(5); // Smooth over last 5 values
    private final double alpha = 0.1; // Example alpha for low-pass filtering

    public AimToBall(CommandSwerveDrivetrain swerve , DoubleSupplier limelightDistance) {
        c_swerve = swerve;
        c_limelightDistance = limelightDistance;

        // Initialize PID controllers
        rotationalPID = new PIDController(0.1 , 0.0 , 0.0); // Example PID values
        strafePID = new PIDController(0.085 , .000 , .00);
        xPID = new PIDController(.085,0,0);

        rotationalPID.setTolerance(1);
        strafePID.setTolerance(0);
        xPID.setTolerance(0);

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        SmartDashboard.putBoolean("AimToBallCommand" , true);
        c_swerve.resetPID();
        rotationalPID.reset();
        strafePID.reset();
        xPID.reset();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void execute() {
        // Get raw and smoothed X offset from the Limelight
        double rawX = VisionVariables.BackCam.target.getX();
        double rawY = VisionVariables.BackCam.target.getDistance();
        double smoothedX = xFilter.calculate(rawX);

        // Calculate PID outputs
        double rotationOutput = rotationalPID.calculate(smoothedX , 0); // Align X to 0
        double strafeOutput = strafePID.calculate(rawX , 0); // Strafe to center ball
        double xOutput = xPID.calculate(rawY, 2); //this setpoint will probably have to be changed
        // Log values to SmartDashboard
        SmartDashboard.putNumber("Limelight Raw X" , rawX);
        SmartDashboard.putNumber("Limelight Smoothed X" , smoothedX);
        SmartDashboard.putNumber("Distance to Ball" , c_limelightDistance.getAsDouble());
        SmartDashboard.putNumber("Rotation Output" , rotationOutput);
        SmartDashboard.putNumber("Strafe Output" , strafeOutput);
        SmartDashboard.putNumber("X output", xOutput);

        // Send control to the swerve drivetrain
        c_swerve.setControl(drive
                .withVelocityX(xOutput) // forward/backward movement based on distance from ball
                .withVelocityY(strafeOutput) // Strafe based on X offset
                .withRotationalRate(0)); // no rotational output for now
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("AimToBallCommand" , false);
    }
}
