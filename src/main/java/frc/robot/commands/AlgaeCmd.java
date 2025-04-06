// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.AlgaeSub;

// public class AlgaeCmd extends Command {

//   AlgaeSub n;

//   public AlgaeCmd(AlgaeSub n) {
//     this.n = n;
//     addRequirements(n);
//   }

//   @Override
//   public void initialize() {}

//   @Override
//   public void execute() {
//     n.forward();
//   }

//   @Override
//   public void end(boolean interrupted) {
//     //n.passTheCoral();
//   }

//   @Override
//   public boolean isFinished() {
//     if(Math.abs(n.enc.getPosition()) <= (178000) && Math.abs(n.enc.getPosition()) >= (162000))
//     return true;
//     n.motor.set(0);
//     n.back();
//     return false;
//   }
// }
