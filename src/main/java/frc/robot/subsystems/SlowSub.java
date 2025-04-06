package frc.robot.subsystems;
import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;

public class SlowSub extends SubsystemBase {

  public CommandXboxController User1 = new CommandXboxController(0);
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeed At12VoltsMps desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
  CommandSwerveDrivetrain drivetrain; 
  private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0).withRotationalDeadband(MaxAngularRate * 0) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  
  public SlowSub(CommandSwerveDrivetrain drivetrain) {
  this.drivetrain = drivetrain;

  }

  public void periodic(){
   
  }

    public boolean slowDrive(double x,double y,double rot){
     drivetrain.setControl(drive.withVelocityX(-User1.getLeftX() * MaxSpeed * 0.3)
     .withVelocityY(-User1.getLeftY() * MaxSpeed * 0.3)
     .withRotationalRate(-User1.getRightX() * MaxAngularRate * 0.3));
      return true;
    }

  }
