// package frc.robot.subsystems;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.SparkBase.ResetMode;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// import com.revrobotics.spark.config.SparkMaxConfig;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Subsystem;

// public class AlgaeSub implements Subsystem {

//   double time;
//   boolean check = false;
//   public SparkMax motor;
//   double velo;
//   PIDController pid;
//   public RelativeEncoder enc;

//   DigitalInput backIR;
//   DigitalInput frontIR;
  
//   public AlgaeSub() {
//     motor = new SparkMax(21,MotorType.kBrushless);

//     backIR = new DigitalInput(4);
//     frontIR = new DigitalInput(5);

//     SparkMaxConfig config = new SparkMaxConfig();
//     enc = motor.getEncoder();
//     pid = new PIDController(0.00016, 0, 0.000002);

//     config
//     .inverted(true)
//     .idleMode(IdleMode.kBrake);
// config.encoder
//     .positionConversionFactor(1000)
//     .velocityConversionFactor(1000);
// config.closedLoop
//     .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
//     .pid(1.0, 0.0, 0.0);
    
// motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

//   }

//   public void Algae(){
   
//   //  SmartDashboard.putBoolean("Coral Status : ", ir.get());

//      } 

//      public void forward(){
//         motor.setVoltage(pid.calculate(enc.getPosition(), -170000));
//      }

//      public void back(){
//       motor.setVoltage(pid.calculate(enc.getPosition(), -110000));
//      }


//   public void Intake(double speed){
  
//     SmartDashboard.putNumber("Algae Status : ", enc.getPosition());
   
//     if(!frontIR.get() && speed >= -0.9 && speed < 0)
//     {
//       motor.set(speed);
//     }
//     else if (!backIR.get() && speed <= 0.9 && speed > 0) {
//       motor.set(speed);
//     }
//     else{
//       motor.set(0);
//     }
    
//   } 
// }
