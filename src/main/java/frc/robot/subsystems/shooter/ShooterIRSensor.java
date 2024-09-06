// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DIOConstants.*;
import frc.robot.subsystems.SubsystemABC;

public class ShooterIRSensor extends SubsystemABC {
  /** Creates a new BreakBeamSensor. */
  // private final DigitalInput transmitter;
  private DigitalInput receiverShooter;
  private BooleanEntry beamBrokenShooter;

  public ShooterIRSensor() {
    setupNetworkTables("irsensor_shooter");

    // transmitter = new DigitalInput(SensorConstants.breakBeamTransmitterPort);
    receiverShooter = new DigitalInput(SensorConstants.shooterBreakBeamReceiverPort);

    beamBrokenShooter = ntTable.getBooleanTopic("shooter_loaded").getEntry(true);

    setupShuffleboard();
    seedNetworkTables();
  }

  public void reset() {
    beamBrokenShooter.set(true);
    beamBrokenShooter.close();
  }

  public Command reinitialize() {
    // Reset the current state
    reset();

    // Reinitialize the receiver and network table entry
    receiverShooter = new DigitalInput(SensorConstants.intakeBreakBeamReceiverPort);
    beamBrokenShooter = ntTable.getBooleanTopic("intake_loaded").getEntry(true);

    // Reinitialize Shuffleboard and NetworkTables
    setupShuffleboard();
    seedNetworkTables();
    return null;
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
