package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class ResetCmd extends Command{
    


   CommandSwerveDrivetrain drivetrain;
  // LimelightHelpers.PoseEstimate mt2;

  // Pose2d pose;
   public ResetCmd(CommandSwerveDrivetrain obj){
        drivetrain = obj;
        //addRequirements(drivetrain);
   } 

    @Override
    public void initialize() {
      //SmartDashboard.putString("MEGATAG.2", drivetrain.getState().Pose.toString());
    }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  //  boolean doRejectUpdate = false;
  //     LimelightHelpers.SetRobotOrientation("limelight-l",  drivetrain.getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
  //       mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-l");
  //       if(Math.abs(drivetrain.getPigeon2().getAngularVelocityZDevice().getValueAsDouble()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
  //       {
  //           doRejectUpdate = true;
  //       }
  //       if(mt2.tagCount == 0)
  //       {
  //           doRejectUpdate = true;
  //       }
  //       if(!doRejectUpdate)
  //       {
  //         drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
  //         drivetrain.addVisionMeasurement(
  //               mt2.pose,
  //               mt2.timestampSeconds);
  //       }
  //       SmartDashboard.putString("MEGATAG.2", drivetrain.getState().Pose.toString());
     }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) { 
    
   }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if(drivetrain.getState().Pose.getRotation().getRadians() == mt2.pose.getRotation().getRadians())
    //     return true;
    // else
    return false;
  }
}
