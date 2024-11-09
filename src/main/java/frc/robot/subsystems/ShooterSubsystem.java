// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorIDConstants;
import frc.robot.Constants.MotorPIDConstants;
import frc.robot.Constants.ShooterConstants;

// Shooter Subsystem Code yippee
public class ShooterSubsystem extends SubsystemBase {

  // Init Stuff
  private TalonFX m_shooterIndexerMotor;
  private TalonFX m_shooterMotor;
  private TalonFXConfiguration krakenConfig;
  private Slot0Configs slot0Configs;

/* 
  private VelocityVoltage velocityController = new VelocityVoltage(0)
        .withSlot(0)
        .withFeedForward(0.0);
*/

  // Other setup items
  public ShooterSubsystem() {
    /* 
    m_shooterIndexerMotor.getConfigurator().apply(new TalonFXConfiguration());
    m_shooterMotor.getConfigurator().apply(new TalonFXConfiguration());
    */

    // KRAKENS
    m_shooterIndexerMotor = new TalonFX(MotorIDConstants.k_shooterIndexerMotorID);
    m_shooterMotor = new TalonFX(MotorIDConstants.k_shooterMotorID);

    // PID Stuff
    // slot0Configs = new Slot0Configs();
    // slot0Configs.kP = MotorPIDConstants.k_intakekP;
    // slot0Configs.kI = MotorPIDConstants.k_intakekI;
    // slot0Configs.kD = MotorPIDConstants.k_intakekD;

    // Apply Configs, Inversion, Control requests
    m_shooterIndexerMotor.getConfigurator().apply(krakenConfig);
    m_shooterMotor.getConfigurator().apply(krakenConfig);
    m_shooterIndexerMotor.getConfigurator().apply(slot0Configs, 0.05);

    m_shooterMotor.setInverted(false);
    m_shooterIndexerMotor.setControl(new Follower(m_shooterMotor.getDeviceID(), true));

  }

  // Runs once per scheduler run
  public void periodic() {
    SmartDashboard.putNumber("Shoot Velocity", m_shooterMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Index Velocity", m_shooterIndexerMotor.getVelocity().getValueAsDouble());
  }

  /* 
  public void setVelocity(double velocity) {
    m_shooterMotor.setControl(velocityController.withVelocity(velocity));
    m_shooterIndexerMotor.setControl(velocityController.withVelocity(velocity));
}
  */


  // Spin up command
  public void spinUp() {
    m_shooterMotor.set(ShooterConstants.k_shooterKrakenSpeed);
  }

  // Shoots yippeee
  public void shoot() {
    m_shooterIndexerMotor.set(ShooterConstants.k_shooterIndexerKrakenSpeed);
    //m_shooterMotor.set(ShooterConstants.k_shooterKrakenSpeed);
  }

  // Stops motors
  public void stopIndex() {
    m_shooterIndexerMotor.set(0.0);
  }

  public void stopShoot(){
    m_shooterMotor.set(0);
  }

  public double getShooterMotorSpeed(){
    return m_shooterMotor.getVelocity().getValueAsDouble();
  }
  // private TalonFXConfiguration getMotorConfigs() {
  //       return new TalonFXConfiguration()
  //           .withCurrentLimits(
  //               new CurrentLimitsConfigs()
  //                   .withSupplyCurrentLimit(35.0)
  //           )
  //           .withSlot0(
  //               new Slot0Configs()
  //                   .withKP(0.0)
  //                   .withKI(0.0)
  //                   .withKD(0.0)
  //                   .withKS(0.0)
  //                   .withKV(0.0)
  //                   .withKA(0.0)
  //           )            
  //           .withMotorOutput(
  //               new MotorOutputConfigs()
  //                   .withInverted(InvertedValue.Clockwise_Positive)
  //                   .withNeutralMode(NeutralModeValue.Coast)
  //           );
  //   }
    
}
