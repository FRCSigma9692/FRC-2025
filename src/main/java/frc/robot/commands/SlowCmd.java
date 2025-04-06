package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SlowSub;

public class SlowCmd extends Command {

  SlowSub l;
  double x,y,rot;

  /** Creates a new MotorCommand. */
  public SlowCmd(SlowSub l,double x,double y,double rot) {
    this.l = l; 
    this.x = x;
    this.y = y;
    this.rot = rot;
    addRequirements(l);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    l.slowDrive(x,y,rot);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}


