package frc.robot.subsystems;
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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class TriggerSub implements Subsystem {

  double time;
  boolean check = false;
  public SparkMax triggermotor;
 // public SparkMax triggermotorR;
  double velo;
  PIDController pid;
  public RelativeEncoder enc;

  Timer timer;

  
  
  public TriggerSub() {
    triggermotor = new SparkMax(22,MotorType.kBrushless);
   // triggermotorR = new SparkMax(21,MotorType.kBrushless);
    
    SparkMaxConfig config = new SparkMaxConfig();
    enc = triggermotor.getEncoder();
    pid = new PIDController(0.0010, 0, 0.00006);

    config
    .inverted(true)
    .idleMode(IdleMode.kBrake);
config.encoder
    .positionConversionFactor(1000)
    .velocityConversionFactor(1000);
config.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pid(1.0, 0.0, 0.0);
    
triggermotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
//triggermotorR.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  public void Algae(){
   
  //  SmartDashboard.putBoolean("Coral Status : ", ir.get());

     } 

    //  public boolean forward(){
    //   if(backIR.get())
    //   {
    //     motor.set(-0.9);
    //     return false;
    //   }
    //   else{
    //     return true;
    //   }
    //  }

    //  public boolean back(){
    //   if (frontIR.get() ) {
    //     motor.set(0.9);
    //     return false;
    //   }
    //   else{
    //     return true;
    //   }
    //  }


  public void LeftIntake(double speed){
  
    // SmartDashboard.putBoolean("Trigger Forward: ", front.get());
    // SmartDashboard.putBoolean("Trigger Back: ", back.get());

        
  }

  public void Intake(double speed){
  
    // SmartDashboard.putBoolean("Trigger Forward: ", front.get());
    // SmartDashboard.putBoolean("Trigger Back: ", back.get());
    SmartDashboard.putNumber("JOysTiCK", speed);
        //triggermotorR.set(speed);
        triggermotor.set(speed);
        
  }
 public void TriggerPID(double s){
    if(enc.getPosition()>=28000)
    triggermotor.set(0);
    else
    
    triggermotor.set(s);
   // triggermotor.setVoltage(pid.calculate(enc.getPosition(),20000));

  }


  
  public void stop(){
    triggermotor.set(0.0);
   // triggermotorR.set(0.0);
  }
}
