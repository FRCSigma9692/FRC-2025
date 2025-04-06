package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class ElevatorSub implements Subsystem {
  /** Creates a new MotorSubSystem. */

  double time;
  
  SparkMax motor;
  double velo;
  PIDController pid;
  RelativeEncoder enc;
  public DigitalInput CoralIr;


  public DigitalInput HallEffect;

  public final TalonFX LeftElevator;
  public final TalonFX RightElevator;
  
  public ElevatorSub() {

    motor = new SparkMax(18,MotorType.kBrushless);
    SparkMaxConfig config = new SparkMaxConfig();

    CoralIr = new DigitalInput(2);

    config
    .inverted(true)
    .idleMode(IdleMode.kBrake);
config.encoder
    .positionConversionFactor(1000)
    .velocityConversionFactor(1000);
config.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pid(1.0, 0.0, 0.0);
    
motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    HallEffect = new DigitalInput(0);

    LeftElevator = new TalonFX(14,"Sigma9692");//Elevator 1
    RightElevator = new TalonFX(15,"Sigma9692");

    TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    LeftElevator.getConfigurator().apply(motorConfig);
    RightElevator.getConfigurator().apply(motorConfig);

    
    var talonFXConfigs = new TalonFXConfiguration();

    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = 0.25;// 0.25; // Add 0.25 V output to overcome static friction
    slot0Configs.kV = 0.12;// 0.010; // 012 A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = 0.01;// 0.0001; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = 0.55;// 0.35; //2 // A position error of 2.5 rotations results in 12 V output
    slot0Configs.kI = 0;// 0; // no output for integrated error
    slot0Configs.kD = 0;// 0.0; // A velocity error of 1 rps results in 0.1 V output
    // set Motion Magic settings
    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    // motionMagicConfigs.MotionMagicCruiseVelocity = 0; // 
    // motionMagicConfigs.MotionMagicExpo_kV = 0.00005; // 0.00051
    // motionMagicConfigs.MotionMagicExpo_kA = 0.0005; // 0.005

    motionMagicConfigs.MotionMagicCruiseVelocity = 3200; // 
    motionMagicConfigs.MotionMagicAcceleration = 450; // 0.00051
    motionMagicConfigs.MotionMagicJerk = 1200; // 0.005

    LeftElevator.getConfigurator().apply(talonFXConfigs);
    RightElevator.getConfigurator().apply(talonFXConfigs);
  }

  @Override
  public void periodic() {
    
    // This method will be called once per scheduler run
    
  }

  public void passTheCoral(){
 
     if(!CoralIr.get()){
       motor.set(0.4);
     }
     else{

       motor.set(0);
     }
   } 
 
 
   public void drop(double speed){
     //SmartDashboard.putBoolean("Coral Status : ", ir.get());
     motor.set(speed);
 
   } 

  public void rotateMoter(double position){

    

    final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

    // if(HallEffect.get()){
    //   SmartDashboard.putBoolean("HallEffect", HallEffect.get());
  
    //   LeftElevator.setPosition(0);
    //   RightElevator.setPosition(0);
    // }

  if(!CoralIr.get()){
    
    
  }
  else{
    LeftElevator.setControl(m_request.withPosition(-position));
    RightElevator.setControl(m_request.withPosition(position));

    SmartDashboard.putNumber("SetLeftElevator", position);
    SmartDashboard.putNumber("SetRightElevator", position);
  }
    SmartDashboard.putBoolean("HallEffect", HallEffect.get());

    // motor.Output = 0.5;
    // singleMotor.setControl(motor);
  }
  
  public void rotateMoterDynamic(double position){

    SmartDashboard.putNumber("LeftElevator", LeftElevator.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("RightElevator", RightElevator.getPosition().getValueAsDouble());

    final DynamicMotionMagicVoltage m_request = new DynamicMotionMagicVoltage(0, 1500,2000,8000);
    // LeftElevator.setControl(m_request,true);
  //       m_request.Velocity = 40; // rps
  //   m_request.Acceleration = 80; // rot/s^2
  //  m_request.Jerk = 400; // rot/s^3

//    SmartDashboard.putBoolean("SetLeIRRRRFalse", ir.get());

    //if(position)

    LeftElevator.setControl(m_request.withPosition(-position));
    RightElevator.setControl(m_request.withPosition(position)); 
    //SmartDashboard.putBoolean("SetLeIRRRRTure", ir.get());
    SmartDashboard.putNumber("SetLeftElevator", position);
    SmartDashboard.putNumber("SetRightElevator", position);
    

    // motor.Output = 0.5;
    // singleMotor.setControl(motor);
  }


  public void powerBase(double speed){
    SmartDashboard.putNumber("The Command is running", speed);
    if(speed < 0 && (Math.abs(LeftElevator.getPosition().getValueAsDouble()) + RightElevator.getPosition().getValueAsDouble())/2 > 0){
      LeftElevator.set(-speed );
      RightElevator.set(speed);
    }
    else if(speed > 0 && (Math.abs(LeftElevator.getPosition().getValueAsDouble()) + RightElevator.getPosition().getValueAsDouble())/2 < 50){
      LeftElevator.set(-speed );
      RightElevator.set(speed);
    }
    else{
      stop();
    }
  }

  public void stop(){
    LeftElevator.set(0);
    RightElevator.set(0);
  }



}

