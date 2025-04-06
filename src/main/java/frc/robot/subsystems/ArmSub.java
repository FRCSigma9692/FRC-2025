package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class ArmSub implements Subsystem {
  /** Creates a new MotorSubSystem. */

  private final TalonFX Arm;
  
  public ArmSub() {
    Arm = new TalonFX(20,"Sigma9692");

    
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    Arm.getConfigurator().apply(motorConfig);
    
    var talonFXConfigs = new TalonFXConfiguration();

    var slot0Configs = talonFXConfigs.Slot0;
 
    slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0Configs.kV = 0.004; // 012 A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = 0.00001; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = 0.2; //2 // A position error of 2.5 rotations results in 12 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0; // A velocity error of 1 rps results in 0.1 V output
    // set Motion Magic settings
    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 0; // 
    motionMagicConfigs.MotionMagicExpo_kV = 0.00005; // 0.00051
    motionMagicConfigs.MotionMagicExpo_kA = 0.00005; // 0.005

    Arm.getConfigurator().apply(talonFXConfigs);

  }

  @Override
  public void periodic() {
    
    // This method will be called once per scheduler run
    
  }
  
  public void Arm(double position){

    SmartDashboard.putNumber("Arm", Arm.getPosition().getValueAsDouble());
  

    final MotionMagicExpoVoltage m_request = new MotionMagicExpoVoltage(0);

  //       m_request.Velocity = 40; // rps
  //   m_request.Acceleration = 80; // rot/s^2
  //  m_request.Jerk = 400; // rot/s^3
   
    Arm.setControl(m_request.withPosition(position));

    SmartDashboard.putNumber("SetArm", position);

    // motor.Output = 0.5;
    // singleMotor.setControl(motor);
  }

  public void manualArm(double pow){
    Arm.set(-pow);
  }
  public void stop(){
    Arm.set(0);
  }
 

}

