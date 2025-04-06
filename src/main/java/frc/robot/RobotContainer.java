
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AlgaeCmdDrop;
import frc.robot.commands.CoralAutoFirst;
//import frc.robot.commands.AlgaeCmd;
import frc.robot.commands.CoralAutoNext;
import frc.robot.commands.CoralDropCmd;
import frc.robot.commands.ElevatorCmd;
import frc.robot.commands.GlideCmd;
import frc.robot.commands.HangingCmd;
import frc.robot.commands.LimelightCmd;
import frc.robot.commands.RemoveLetch;
import frc.robot.commands.ResetPoseCmd;
import frc.robot.commands.TriggerCmd;
import frc.robot.commands.TriggerCmdSensor;
import frc.robot.generated.TunerConstants;
//import frc.robot.subsystems.AlgaeSub;
import frc.robot.subsystems.TriggerSub;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSub;
import frc.robot.subsystems.GlideSub;
import frc.robot.subsystems.LimelightSub;
import frc.robot.subsystems.HangingSub;

public class RobotContainer {
    //- All Variable Declaration --------------------------------
    // Robot Speed 
    public double setSpd = 0.9;

    // Limelight Alignment
    public double forwardAlineDistance = 12.5; // forward alignment Distace

    // Hanging Servo 0.5 is stop - Power Based
    public double letchReverse = 0.3; // 1 - Dpad Right
    public double letchForward = 0.8; // 1 - Dpad Left
    
    // Hanging Legs - Position Based 
    public double legsIn = 0; // robot Down - Default // 1 - Dpad Down 
    public double legsOut = 55; // robot LiftUp       // 1 - Dpad Up

    // Glide Mechanism - Power Based 
    public double glideManualSpd = 0.5;
    public double glideSpd = 0.8; // until right limit switch is not pressed // 2 - Dpad Left/Right
    public double glideSpdAuto = 0.8;

    // Elevator - Position based
    public double eleManualSpd = 0.3;
    public double L1 = 0;  // 2 - Button X
    public double L2 = 11; // 2 - Button A
    public double L3 = 26; // 2 - Button B
    public double L4 = 52; // 2 - Button Y

    // Coral - Power Based
    public double coralDrop = 0.7;   //0.9 IN AUTO        // 2 - Right Trigger
    public double coralTakeBackIn = 0.25; // 2 - Right Bumper
    public double coralDropTimeAuto = 0.3; // In Auto Only 

    // Trigger - Power Based
    public double triggerShoot = 0.35;  // 2 - Left Trigger
    public double triggerReset = 0.15; // 2 - Left Bumper
    public double triggerLimit = 28000;
    public double triggerSensorBasedSpdAuto = 0.55;

    // Algae Drop - Position Based // 2 - Dpad Down
    public double algaeDown = 12.5; 
    public double algaeUp = 28.5; 

// ----------------------------------------------------------   
    public BooleanSupplier override = ()-> true;
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
 
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public CommandPS4Controller User1 = new CommandPS4Controller(0);
    //public final CommandXboxController User1 = new CommandXboxController(0);
    private final CommandXboxController User2 = new CommandXboxController(1);
    
    public final LimelightSub lime = new LimelightSub(drivetrain);

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.03).withRotationalDeadband(MaxAngularRate * 0.03) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    //private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    // -------- NEW --------------------------------------------------------------
   

    public final HangingSub hanging = new HangingSub();
    
    public final ElevatorSub elevator = new ElevatorSub();

    public final GlideSub glide = new GlideSub();

    //public final ArmSub arm = new ArmSub();

   // public final AlgaeSub Algae = new AlgaeSub();

    public final TriggerSub Trigger = new TriggerSub();

   // Pose2d targetPose = 

// Create the constraints to use while pathfinding

    private final SendableChooser<Command> autoChooser;
    // --------------------------------------------------------------------------

    public RobotContainer() {
        // Rotation2d k = drivetrain.getState().Pose.sset(180);
        // k = Rotation2d(180);
        NamedCommands.registerCommand("CC", new CoralAutoFirst(elevator).andThen(new CoralAutoNext(elevator)));
        NamedCommands.registerCommand("PassSensorBased", new TriggerCmdSensor(Trigger,triggerSensorBasedSpdAuto,elevator));
        NamedCommands.registerCommand("ReverseSensorBased", new TriggerCmdSensor(Trigger,-triggerReset,elevator));
        NamedCommands.registerCommand("L1", new ElevatorCmd(elevator, L1));
        NamedCommands.registerCommand("L2", new ElevatorCmd(elevator, L2));
        NamedCommands.registerCommand("L3", new ElevatorCmd(elevator, L3));
        NamedCommands.registerCommand("L4", new ElevatorCmd(elevator, L4));
        NamedCommands.registerCommand("Lime", new LimelightCmd(lime));
        NamedCommands.registerCommand("ResetPoseBlue", new ResetPoseCmd(new Pose2d(new Translation2d(7.167,5.295),
                                             new Rotation2d(Math.toRadians(180))), drivetrain));
        // NamedCommands.registerCommand("ResetPoseRed", new ResetPoseCmd(new Pose2d(new Translation2d(7.227,2.530),
        //                                      new Rotation2d(Math.toRadians(0))), drivetrain));
        NamedCommands.registerCommand("PassTheCoral", new TriggerCmd(Trigger,triggerShoot));//0.55
        NamedCommands.registerCommand("ReverseTrigger", new TriggerCmd(Trigger,-triggerReset));
        // NamedCommands.registerCommand("Coral", new CoralCmd(coral));
        NamedCommands.registerCommand("CoralDrop", new CoralDropCmd(elevator,coralDropTimeAuto,coralDrop));//.andThen(new CoralDropCmd(coral,1,-0.2)));
        //NamedCommands.registerCommand("CoralIn", new CoralDropCmd(coral,0.2,0.2));  
        NamedCommands.registerCommand("RGlide", new  GlideCmd(glide, glideSpdAuto));
        NamedCommands.registerCommand("LGlide", new  GlideCmd(glide, -glideSpdAuto));
        //NamedCommands.registerCommand("Glide", new GlideCmd(glide,0.5));  
        //NamedCommands.registerCommand("Trigger", new TriggerCmd(Trigger,0.55).andThen(new TriggerCmd(Trigger, -0.25)));

        autoChooser = AutoBuilder.buildAutoChooser("Right Field + IntakeRedLongIsland");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();
    }

    private void configureBindings() {
        // Robot Drive Command on User1
        // Note that X is defined as forward according to WPILib conven tion,
        // and Y is defined as to the left according to WPILib convention.
        // PS5
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-User1.getRawAxis(1)* MaxSpeed * setSpd ) // Drive forward with negative Y (forward)
                    .withVelocityY(-User1.getRawAxis(0) * MaxSpeed * setSpd ) // Drive left with negative X (left)
                    .withRotationalRate(-User1.getRawAxis(2) * MaxAngularRate * setSpd ) // Drive counterclockwise with negative X (left)
            )
        );
        // XBOX
        // drivetrain.setDefaultCommand(
        //     drivetrain.applyRequest(() ->
        //         drive.withVelocityX(-User1.getLeftY()* MaxSpeed * setSpd) // Drive forward with negative Y (forward)
        //             .withVelocityY(-User1.getLeftX() * MaxSpeed *setSpd) // Drive left with negative X (left)
        //             .withRotationalRate(-User1.getRightX() * MaxAngularRate * setSpd) // Drive counterclockwise with negative X (left)
        //     )
        // );

//        coral.setDefaultCommand(new RunCommand(()-> coral.passTheCoral(),coral));

        // // User1 Commands 
        // User1.leftBumper().whileTrue(new RunCommand(() -> {setSpd = 0.2;}, drivetrain ));
        // User1.leftBumper().whileFalse(new RunCommand(() -> {setSpd = 0.4;}, drivetrain ));
        

         //User1.triangle().whileTrue(drivetrain.pathFind(8.069, 6.160, 180,2,1.5, override));//CHange for blue
        User1.L2().whileTrue(drivetrain.pathFind(1.27, 7.07, 35,1,1, override)); //blue
        //User1.L2().whileTrue(drivetrain.pathFind(16.258, 0.937, -144,1,1, override)); //RED, need to change
        // User1.pov(90).onTrue(drivetrain.pathFind(3.8, 5.187, -60, override));//ID 19
        // User1.y().onTrue(drivetrain.pathFind(5.409, 5.355, -120, override));// ID 20
        // User1.x().onTrue(drivetrain.pathFind(5.469, 2.755, 120, override));// ID 22
        // User1.pov(270).onTrue(drivetrain.pathFind(3.681, 2.680, 60, override));//ID 17
        // User1.pov(180).onTrue(drivetrain.pathFind(6.040, 4.002, 180, override));//ID 21
        // User1.pov(0).onTrue(drivetrain.pathFind(2.885, 4.078, 0, override));//ID 18
        
        // PS5
        User1.R1().whileTrue(new LimelightCmd(lime));
        User1.button(12).whileTrue(drivetrain.applyRequest(() -> brake));
        User1.cross().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        //XBOX
        // User1.a().whileTrue(new LimelightCmd2(lime));
        // User1.b().whileTrue(drivetrain.applyRequest(() -> brake));
        // User1.rightBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
   
        // Servo 
        User1.pov(90).whileTrue(new RemoveLetch(hanging,letchReverse));
        User1.pov(270).whileTrue(new RemoveLetch(hanging,letchForward));

        // Hanging
        User1.pov(180).whileTrue(new RunCommand(()-> hanging.LegsIn()));//-10
        User1.pov(0).whileTrue(new HangingCmd(hanging, legsOut));//These are legal values

         //Hang Manually
        // User1.pov(0).whileTrue(new HangingCmd2(hanging, 0.1));
        // User1.pov(180).whileTrue(new HangingCmd2(hanging, -0.1));
        
        // User1.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-User1.getLeftY(), -User1.getLeftX()))
        // ));
        // User1.back().and(User1.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // User1.back().and(User1.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // User1.start().and(User1.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // User1.start().and(User1.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        
       
        // Default Commands
        //c.setDefaultCommand(new CoralCmd(c));

        // Algae Arm
        // User2.pov(0).whileTrue(new ArmCmd(arm, 0.1));
        // User2.pov(180).whileTrue(new ArmCmd(arm, -0.1));

        // Coral Left-Right
        glide.setDefaultCommand(new RunCommand(()-> glide.glide(User2.getLeftX()*0.5), glide));
        User2.pov(90).onTrue(new GlideCmd(glide, glideSpd));//.andThen(new RunCommand(() -> glide.moveBit(), glide)));
        User2.pov(270).onTrue(new GlideCmd(glide, -glideSpd));//.andThen(new RunCommand(() -> glide.moveBit(), glide)));

        // Arm
        // User2.pov(0).onTrue(new ArmCmd(arm, 
        // -3));
       //User2.pov(180).onTrue(new GlideAuto(glide)); 

        //Elevator
        elevator.setDefaultCommand(new RunCommand(()-> elevator.powerBase(-User2.getRightY() * 0.3),  elevator));
        User2.x().onTrue(new ParallelCommandGroup(new ElevatorCmd(elevator, L1),(new GlideCmd(glide, -glideSpd)))); // 0 Level ONE
        User2.pov(180).whileTrue(new RunCommand(()-> glide.glideTo(0.4))); // 0 Level ONE
        User2.a().onTrue(new ElevatorCmd(elevator, L2)); // 10 Level TWO
        User2.b().onTrue(new ElevatorCmd(elevator, L3)); // 26 Level THREE
        User2.y().onTrue(new ElevatorCmd(elevator, L4)); // 50 Level FOUR 50 with thread 40 without thread
        // Elevator Manually
        User2.axisGreaterThan(5, 0).whileTrue(new RunCommand(()-> elevator.powerBase(User2.getRightY()*-0.3),elevator));
        User2.axisLessThan(5,0).whileTrue(new RunCommand(()-> elevator.powerBase(User2.getRightY()*-0.3),elevator));

        // User2.a().whileTrue(new ElevatorCmd(elevator,-0.1));

        //Drop Coral
        // User2.leftBumper().onTrue(new CoralDropCmd(coral, 0.7, 1)); // Coral DROP
        User2.rightBumper().whileTrue(new RunCommand(() -> elevator.drop(-coralTakeBackIn), elevator));
        User2.rightTrigger().whileTrue(new RunCommand(() -> elevator.drop(coralDrop), elevator)); 


        User2.pov(0).onTrue(new TriggerCmd(Trigger,triggerShoot));
        elevator.setDefaultCommand(new RunCommand(()-> elevator.passTheCoral(),elevator));
        
        User2.leftTrigger().onTrue(new TriggerCmd(Trigger,triggerShoot).andThen(new TriggerCmd(Trigger, -triggerReset))); 
        User2.leftBumper().onTrue(new TriggerCmd(Trigger,-triggerReset)); 
        // User2.leftTrigger().onTrue(new RunCommand(() -> Trigger.Intake((User2.getLeftTriggerAxis()) * -0.9), Trigger)); 
//--------
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        // drivetrain.resetPose(new Pose2d(drivetrain.getState().Pose.getTranslation(), 
        //     new Rotation2d(Math.toRadians(180))));
        return autoChooser.getSelected();
    }
}
