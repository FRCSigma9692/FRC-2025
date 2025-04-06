package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class HangingSub implements Subsystem {
  
  public final TalonFX BackHang;
  public final TalonFX FrontHang;

  private final Servo left;
  private final Servo right;
  
  public HangingSub() {
    BackHang = new TalonFX(16,"Sigma9692");//Back
    FrontHang = new TalonFX(17,"Sigma9692");//Front

    left = new Servo(8);
    right = new Servo(9);

    TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    BackHang.getConfigurator().apply(motorConfig);
    FrontHang.getConfigurator().apply(motorConfig);

    var talonFXConfigs = new TalonFXConfiguration();

    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = 0.0001; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = 0.5; //2 // A position error of 2.5 rotations results in 12 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0; // A velocity error of 1 rps results in 0.1 V output
    // set Motion Magic settings
    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 20; // 
    motionMagicConfigs.MotionMagicExpo_kV = 0.00051; // 0.00051
    motionMagicConfigs.MotionMagicExpo_kA = 0.0005; // 0.005

    BackHang.getConfigurator().apply(talonFXConfigs);
    FrontHang.getConfigurator().apply(talonFXConfigs);
  }

  @Override
  public void periodic() {
    
    // This method will be called once per scheduler run
    
  }
  

  public void rotateMoter(double position){

    // LeftElevator.set(position);
    // RightElevator.set(position);

    final MotionMagicExpoVoltage m_request = new MotionMagicExpoVoltage(0);

    //       m_request.Velocity = 40; // rps
    //   m_request.Acceleration = 80; // rot/s^2
    //  m_request.Jerk = 400; // rot/s^3
     
      BackHang.setControl(m_request.withPosition(-(position)));
      FrontHang.setControl(m_request.withPosition(position));
    // SmartDashboard.putNumber("SetLeftHang", position);
    // SmartDashboard.putNumber("SetRightHang", position);

  }

  public void LegsIn(){

    // LeftElevator.set(position);
    // RightElevator.set(position);

    final MotionMagicExpoVoltage m_request = new MotionMagicExpoVoltage(0);

    //       m_request.Velocity = 40; // rps
    //   m_request.Acceleration = 80; // rot/s^2
    //  m_request.Jerk = 400; // rot/s^3
     
      BackHang.setControl(m_request.withPosition(-(0)));//-8
      FrontHang.setControl(m_request.withPosition(0));//-5
    // SmartDashboard.putNumber("SetLeftHang", position);
    // SmartDashboard.putNumber("SetRightHang", position);

  }

  public void rotateMoter2(double position){

    BackHang.set(position);
    //RightElevator.set(position);
    ///final MotionMagicExpoVoltage m_request = new MotionMagicExpoVoltage(0);

    //       m_request.Velocity = 40; // rps
    //   m_request.Acceleration = 80; // rot/s^2
    //  m_request.Jerk = 400; // rot/s^3
     
    //  RightElevator.setControl(m_request.withPosition(position));

    SmartDashboard.putNumber("SetLeftHang", position);
    SmartDashboard.putNumber("SetRightHang", position);

  }
  public void Servo(double speed){
    left.set(speed);
    right.set(speed);
  }
  

}

