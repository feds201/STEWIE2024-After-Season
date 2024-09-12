// ShooterIRSensor.java

package frc.robot.subsystems.shooter;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DIOConstants.*;
import frc.robot.subsystems.SubsystemABC;

public class ShooterIRSensor extends SubsystemABC {
  private DigitalInput receiverShooter;
  private BooleanEntry beamBrokenShooter;

  public ShooterIRSensor() {
    setupNetworkTables("irsensor_shooter");
    receiverShooter = new DigitalInput(SensorConstants.shooterBreakBeamReceiverPort);
    beamBrokenShooter = ntTable.getBooleanTopic("shooter_loaded").getEntry(true);
    setupShuffleboard();
    seedNetworkTables();
  }

  public void reset() {
    beamBrokenShooter.set(true);
  }

  public Command reinitialize() {
    return new Command() {
      @Override
      public void execute() {
        reset();

      }
    };
  }

  @Override
  public void setupShuffleboard() {
    tab.add("BreakBeam", receiverShooter);
  }

  @Override
  public void periodic() {
    writePeriodicOutputs();
  }

  @Override
  public void seedNetworkTables() {
  }

  @Override
  public void writePeriodicOutputs() {
    readBeamBroken();
  }

  public void readBeamBroken() {
    beamBrokenShooter.set(!receiverShooter.get());
  }

  public boolean getBeamBroken() {
    return beamBrokenShooter.get();
  }
}