package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSub;

public class LimelightCmd extends Command {

  LimelightSub l;

  /** Creates a new MotorCommand. */
  public LimelightCmd(LimelightSub l) {
    this.l = l; 
    addRequirements(l);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    l.Autoalignment();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    
    }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(l.Autoalignment() == true){
      return false;
    }
    else{
      return true;
    }
    
  }
}


