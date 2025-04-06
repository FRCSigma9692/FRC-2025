package frc.robot.subsystems;
import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.commands.LimelightCmd;
import frc.robot.generated.TunerConstants;

public class LimelightSub extends SubsystemBase {

  double kP = 0.05;
  double tXleft;
  double rYleft;
  double tXright;
  double rot;
  boolean tV;
  double x,y,strafe,head;
  double lastHeadError = 0, lastStrafeError = 0, lastDistError = 0;
  double dist = 11.75;

  LimelightCmd l;

  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeed At12VoltsMps desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
  CommandSwerveDrivetrain drivetrain; 
  private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0).withRotationalDeadband(MaxAngularRate * 0) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  
  public LimelightSub(CommandSwerveDrivetrain drivetrain) {
  this.drivetrain = drivetrain;

  tV = LimelightHelpers.getTV("limelight-l");
  }

  public void periodic(){
    tXleft =  LimelightHelpers.getTX("limelight-l");
    tV = LimelightHelpers.getTV("limelight-l");
    head = Math.toDegrees(LimelightHelpers.getTargetPose3d_CameraSpace("limelight-l").getRotation().getY());
  }
  
  public double getDistance(){
    double ty = LimelightHelpers.getTY("limelight-l");

    // how many degrees back is your limelight rotated from perfectly vertical?
    double limelightMountAngleDegrees = 0; 

    // distance from the center of the Limelight lens to the floor
    double limelightLensHeightInches = 15.62; 

    // distance from the target to the floor
    double goalHeightInches = 12; 

    double angleToGoalDegrees = limelightMountAngleDegrees + ty;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

    double distance = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
    return distance;
    }
  
    public double cord(){
      if(tXleft <= -1.3 && tXleft >= -3.3 ){
        return 0.0;
      }
      double StrafeOut =  -((tXleft-(-2.3)) * 0.025)-(0.048*(((tXleft-(-2.3)) -lastStrafeError)/0.02));
      //double StrafeOut =  -((tXleft-0.5) * 0.03)-(0.048*(((tXleft-0.5) -lastStrafeError)/0.02));
      lastStrafeError = (tXleft-(-2.3));
      return StrafeOut;
    }

    public boolean Autoalignment(){
      if(tV == false){
        return false;
      }
      tXleft =  LimelightHelpers.getTX("limelight-l");
      tV = LimelightHelpers.getTV("limelight-l");
      head = Math.toDegrees(LimelightHelpers.getTargetPose3d_CameraSpace("limelight-l").getRotation().getY());
      
      SmartDashboard.putNumber("Output Turning ", cord());
      SmartDashboard.putNumber("RoctationAngleeeeeeeeeee", head);
      SmartDashboard.putNumber("Distance", getDistance()-dist);

      SmartDashboard.putString("Ends", "no");
      //PIDController pidHeading = new PIDController(0.08, 0, 0);
        if(head >= 1.5 || head <= -1.5){
          rot = -(0.02 * head) -(0.015*((head -lastHeadError)/0.02));
          //output = -(0.015 * head) -(0.015*((head -lastHeadError)/0.02));
         // output = -pidHeading.calculate(head,0);
          lastHeadError= head;
          rot = Math.max(-1, Math.min(1, rot));
          
        }
        else{
          rot = 0;
        }

        strafe = cord();

        if(getDistance() <= dist-1 || getDistance() >= dist+1 && (Math.abs(getDistance()) <=100000000) )
        {
          y = ((getDistance() -dist) * 0.03) + 0.03*(((getDistance() - dist) - lastDistError)/0.02);
          //y = ((getDistance() - 11) * 0.025)+ 0.03*(((getDistance() - 11) - lastDistError)/0.02);
          lastDistError = (getDistance() - dist);
          // if(Math.abs(head)>40){
          //   y = 0;
          // }
        }
        else{
          lastDistError = 0;
        }

        if(strafe <= 0.15 && strafe >= -0.15 &&
            getDistance()-dist <= 0.5 && getDistance()-dist >= -0.5 &&
            head <= 1.5 && head >= -1.5){
              drivetrain.setControl(drive.withVelocityX(0)
              .withVelocityY(0)
              .withRotationalRate(0));
              SmartDashboard.putString("Ends", "Yes");
              return false;
        }

     drivetrain.setControl(drive.withVelocityX(y * MaxSpeed * 0.4)
     .withVelocityY(strafe * MaxSpeed * 0.4)
     .withRotationalRate(rot * MaxAngularRate * 0.4));
      return true;
    }
  }
