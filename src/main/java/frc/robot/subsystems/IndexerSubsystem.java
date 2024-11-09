// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.MotorIDConstants;
import frc.robot.Constants.MotorPIDConstants;

// Indexer Subsystem Code yippee
public class IndexerSubsystem extends SubsystemBase {

  // Init Stuff
  private TalonFX m_indexerBottomMotor;
  private TalonFX m_indexerTopMotor;
  private TalonFXConfiguration krakenConfig;
  private Slot0Configs slot0Configs;

  // Other setup items
  public IndexerSubsystem() {
    
    // KRAKENS
    m_indexerBottomMotor = new TalonFX(MotorIDConstants.k_indexerBottomMotorID);
    m_indexerTopMotor = new TalonFX(MotorIDConstants.k_indexerTopMotorID);

    // PID Stuff
    slot0Configs = new Slot0Configs();
    slot0Configs.kP = MotorPIDConstants.k_intakekP;
    slot0Configs.kI = MotorPIDConstants.k_intakekI;
    slot0Configs.kD = MotorPIDConstants.k_intakekD;

    // Kraken Configs
    krakenConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = MotorConstants.k_rampRate;
    krakenConfig.MotorOutput.PeakForwardDutyCycle = MotorConstants.k_closedMaxSpeed;
    krakenConfig.MotorOutput.PeakReverseDutyCycle = -MotorConstants.k_closedMaxSpeed;
    krakenConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    krakenConfig.CurrentLimits.SupplyCurrentLimit = MotorConstants.k_supplyCurrentLimit;

  }

  // Runs once per scheduler run
  public void periodic() {}
  
  // Indexes yippeee
  public void index() {
    m_indexerBottomMotor.set(IndexerConstants.k_indexerKrakenSpeed);
    m_indexerTopMotor.set(-IndexerConstants.k_indexerKrakenSpeed);
  }

  // Stops motors
  public void stop() {
    m_indexerBottomMotor.set(0);
    m_indexerTopMotor.set(0);
  }
}
