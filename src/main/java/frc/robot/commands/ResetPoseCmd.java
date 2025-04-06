package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class ResetPoseCmd extends Command{
    
   CommandSwerveDrivetrain drivetrain;
  
   Pose2d pose;
   public ResetPoseCmd(Pose2d pos, CommandSwerveDrivetrain obj){
        pose = pos;
        drivetrain = obj;
        addRequirements(drivetrain);
   } 

    @Override
    public void initialize() {
      
    }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
    drivetrain.resetPose(pose);
    SmartDashboard.putString("POSE",drivetrain.getState().Pose.getX()+""+drivetrain.getState().Pose.getY()+""+
    drivetrain.getState().Pose.getRotation());
   }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) { 
    
   }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(drivetrain.getState().Pose.getRotation().getRadians() == pose.getRotation().getRadians())
        return true;
    else
    return false;
  }
}
