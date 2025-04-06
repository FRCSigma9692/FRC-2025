package frc.robot.subsystems;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class CoralSub implements Subsystem {

  DigitalInput ir;
  double time;
  
  SparkMax motor;
  double velo;
  
  public CoralSub() {
    motor = new SparkMax(19,MotorType.kBrushless);

    SparkMaxConfig config = new SparkMaxConfig();

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

    ir = new DigitalInput(0);
  }

  public void passTheCoral(){
   
    SmartDashboard.putBoolean("Coral Status : ", ir.get());

    if(ir.get() == false){
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
}
