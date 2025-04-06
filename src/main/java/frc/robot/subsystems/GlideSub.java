// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

// import au.grapplerobotics.ConfigurationFailedException;
//import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class GlideSub implements Subsystem {

  //public LaserCan lc ;

  public SparkMax glidemotor;
  PIDController pid;
  public RelativeEncoder enc;
  SparkMaxConfig config;

  double velo;

  public GlideSub() {


   //lc = new LaserCan(22);


    // try {
    //   lc.setRangingMode(LaserCan.RangingMode.SHORT);
    //   lc.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
    //   lc.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    // } catch (ConfigurationFailedException e) {
    //  // System.out.println("Configuration failed! " + e);
    // }

    glidemotor = new SparkMax(19,MotorType.kBrushless);
    pid = new PIDController(0.1, 0, 0.000002);
    enc = glidemotor.getEncoder();

//     config = new SparkMaxConfig();
//    config
//     .inverted(true)
//     .idleMode(IdleMode.kBrake);
// config.encoder
//     .positionConversionFactor(1000)
//     .velocityConversionFactor(1000);
// config.closedLoop
//     .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
//     .pid(1.0, 0.0, 0.0);
    
//motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

}

  @Override
  public void periodic() {
    // SmartDashboard.putBoolean("LeftLimitSwitch", left.get());
    //SmartDashboard.putBoolean("RightLimitSwitch", right.get());
    SmartDashboard.putNumber("Encoder Fior Glide",enc.getPosition());

    // LaserCan.Measurement measurement = lc.getMeasurement();
      // if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      //   System.out.println("The target is " + measurement.distance_mm + "mm away!");
      // // glidemotor.set(0);
      // }
      // else{
      //   System.out.println("Oh no! The target is out of range, or we can't get a reliable measurement!");
      //  // glidemotor.set(speed);

    //  }
  }

  // public void moveBit(){
  
  //     enc.setPosition(0.0);
  //     SmartDashboard.putNumber("Encoder Fior Glide",enc.getPosition());
  //     glidemotor.setVoltage(pid.calculate(enc.getPosition(),500));
  //   }
  //   else if(right.get()){
  //     enc.setPosition(0.0);
  //     SmartDashboard.putNumber("Encoder Fior Glide",enc.getPosition());
  //     glidemotor.setVoltage(pid.calculate(enc.getPosition(),-500));
  //   }
  //   //motor.setVoltage(pid.calculate(enc.getPosition(),200));



  // }


  public void ZeroPos(double ele){
    if(ele <= 8 && !glidemotor.getReverseLimitSwitch().isPressed()){
      glidemotor.set(-0.8);
    }
    else{
      glidemotor.set(0);
    }
  }

  public void glide(double speed){
      
   
     // motor.setVoltage(pid.calculate(enc.getPosition(),position));
      SmartDashboard.putNumber("Encoder Fior Glide",enc.getPosition());
      // SmartDashboard.putBoolean("LeftLimitSwitch", left.get());
      // SmartDashboard.putBoolean("RightLimitSwitch", right.get());
      //enc.setPosition(0);
      // LaserCan.Measurement measurement = lc.getMeasurement();
      // if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      //   System.out.println("The target is " + measurement.distance_mm + "mm away!");
      //  glidemotor.set(0);
      // }
      // else{
      //   System.out.println("Oh no! The target is out of range, or we can't get a reliable measurement!");
      if(Math.abs(speed) < 0.05){
        //speed = 0;
        glidemotor.set(0);
      }
      else{
        glidemotor.set(speed);  
      }
      
   //   }  
} 

  public void glideTo(double speed){
    if((enc.getPosition()) > 65000 ){
      //speed = 0;
      glidemotor.set(-speed);
    }
    else if(enc.getPosition()<61000){
      glidemotor.set(speed);  
    }
    else{
      glidemotor.set(0);
    }

  }

  public void drop(double speed){
    SmartDashboard.putNumber("Encoder Fior Glide",enc.getPosition());
   // SmartDashboard.putBoolean("Coral Status : ", ir.get());
    glidemotor.set(speed);
  } 
} 
