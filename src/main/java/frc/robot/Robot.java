// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;

//import au.grapplerobotics.CanBridge;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ElevatorCmd;

public class Robot extends TimedRobot {

  //GlideSub gd ;

  private Command m_autonomousCommand;
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-l");;
  //TalonFX sk = new TalonFX(15);
  NetworkTableEntry tray = table.getEntry("ry");
  private final RobotContainer m_robotContainer;
  double ry;
  public Robot() {
    m_robotContainer = new RobotContainer();
 // CanBridge.runTCP();
  }

  @Override
  public void robotPeriodic() {
    if(m_robotContainer.Trigger.triggermotor.getReverseLimitSwitch().isPressed()){
      m_robotContainer.Trigger.enc.setPosition(0);
    }
    SmartDashboard.putNumber("TriggerPosition : ",m_robotContainer.Trigger.enc.getPosition());
    SmartDashboard.putBoolean("TriggerFrontSwitch : ", m_robotContainer.Trigger.triggermotor.getForwardLimitSwitch().isPressed());
    SmartDashboard.putBoolean("TriggerRearSwitch : ", m_robotContainer.Trigger.triggermotor.getReverseLimitSwitch().isPressed());
    // Elevator
    SmartDashboard.putNumber("LeftElevatorMotor : ", m_robotContainer.elevator.LeftElevator.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("RightElevatorMotor : ", m_robotContainer.elevator.RightElevator.getPosition().getValueAsDouble());
    
    SmartDashboard.putBoolean("Coral is in between the elevator : ", m_robotContainer.elevator.CoralIr.get());
    // Glide
    SmartDashboard.putNumber("GlidePosition : ",m_robotContainer.glide.enc.getPosition());
    SmartDashboard.putBoolean("GlideRightSwitch : ", m_robotContainer.glide.glidemotor.getForwardLimitSwitch().isPressed());
    SmartDashboard.putBoolean("GlideLeftSwitch : ", m_robotContainer.glide.glidemotor.getReverseLimitSwitch().isPressed());
    // Hanging
    SmartDashboard.putNumber("HangRear : ", -m_robotContainer.hanging.BackHang.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("HangFront : ", m_robotContainer.hanging.FrontHang.getPosition().getValueAsDouble());//fghjkhgvc

    SmartDashboard.putBoolean("ReverseLimit", m_robotContainer.Trigger.triggermotor.getReverseLimitSwitch().isPressed());
    SmartDashboard.putBoolean("FWDlimit", m_robotContainer.Trigger.triggermotor.getForwardLimitSwitch().isPressed());
  
    if(m_robotContainer.glide.glidemotor.getReverseLimitSwitch().isPressed()){
      m_robotContainer.glide.enc.setPosition(0);
    }

    // try{
    // SmartDashboard.putNumber("LASERCAN", m_robotContainer.glide.lc.getMeasurement().distance_mm);}
    // catch(Exception e){
    //   SmartDashboard.putString("LASERCAN","nil");
    // }
    //sk.setControl(ControlModeValue.DutyCycleOut,0.3);
    
   
   // SmartDashboard.putBoolean("override", m_robotContainer.override.getAsBoolean());
    CommandScheduler.getInstance().run(); 
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
   //m_robotContainer.elevator.runOnce(new ElevatorCmd(m_robotContainer.elevator, 0));
    //  dz//ONLY IF RUNNING BLUE!
    
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    
   m_robotContainer.elevator.rotateMoter(0);
   m_robotContainer.glide.glide(-0.8);

    Pathfinding.setPathfinder(new LocalADStar());
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    //m_robotContainer.glide.setDefaultCommand(new RunCommand(()-> m_robotContainer.glide.ZeroPos(m_robotContainer.elevator.RightElevator.getPosition().getValueAsDouble()), m_robotContainer.glide));
   //m_robotContainer.coral.setDefaultCommand(new RunCommand(()-> m_robotContainer.coral.passTheCoral(),m_robotContainer.coral));
   m_robotContainer.Trigger.setDefaultCommand(new RunCommand(()-> m_robotContainer.Trigger.LeftIntake(0),m_robotContainer.Trigger));
   //PS5 
   if(m_robotContainer.User1.L1().getAsBoolean()){
   //XBOX   
  //if(m_robotContainer.User1.leftBumper().getAsBoolean()){
      m_robotContainer.setSpd = 0.2;
    }
    else{
      m_robotContainer.setSpd = 0.75;
    }
   
    ///SmartDashboard.putBoolean("LeftSwitchRobot", gd.right.get());
    
    //double tXleft =  LimelightHelpers.getTX("limelight-l");
    //ry = Math.toDegrees(LimelightHelpers.getTargetPose3d_CameraSpace("limelight-l").getRotation().getY());
    ///SmartDashboard.putNumber("AprilTagAngleRobotREL", ry);
  }///////

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
