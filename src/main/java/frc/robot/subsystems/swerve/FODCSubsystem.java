package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.swerve.FODC;

import java.util.function.DoubleSupplier;


public class FODCSubsystem extends SubsystemBase {
    private RiteshManFilter filter;
    private Pigeon2 pigeonIMU;
    private double previousTime;

    private double filteredAngle;


    public FODCSubsystem() {
        // Initialize the filter with a time step (for example, 0.1 seconds)
        filter = new RiteshManFilter(0.1);
        pigeonIMU = new Pigeon2(0);  // Ensure the device ID is correct
        previousTime = Timer.getFPGATimestamp();
    }


    @Override
    public void periodic() {
        // Call this method regularly to update the filtered angle and angular velocity
        updateKalmanFilter();
    }

    private void updateKalmanFilter() {
        double currentTime = Timer.getFPGATimestamp();
        double dt = currentTime - previousTime;
        previousTime = currentTime;

        // Retrieve the yaw angle and angular velocity from the Pigeon2
        double rawAngle = pigeonIMU.getYaw().getValue();  // Get the yaw value
        double rawAngularVelocity = pigeonIMU.getRate();  // Get the angular velocity (yaw axis)

        // if i am in simulation, make the yaw angle and angular velocity random
        if ( pigeonIMU.getSimState() != null ) {
            rawAngle = Math.random() * 360 - 180;
            rawAngularVelocity = Math.random() * 360 - 180;
        }

        // Update the filter with the noisy sensor data
        filter.predict(0 , 0);  // No control input for now
        filter.update(rawAngle , rawAngularVelocity);
    }

    // Method to get the filtered angle from the Kalman Filter
    public double getFilteredAngle() {
        return filter.getAngle();
    }

    // Method to get the filtered angular velocity from the Kalman Filter
    public double getFilteredAngularVelocity() {
        return filter.getAngularVelocity();
    }

    public ShuffleboardContainer getShuffleboardTab() {
        return Shuffleboard.getTab("FODC");
    }


}
