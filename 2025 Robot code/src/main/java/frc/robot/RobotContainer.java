// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.util.function.DoubleToIntFunction;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.JoystickConstants;
import frc.robot.commands.StartMotionSequence;
import frc.robot.commands.TelopSwerve;
import frc.robot.commands.algea.IntakeAlgae;
import frc.robot.commands.algea.Proceser;
import frc.robot.commands.algea.EXO.OzOutake;
import frc.robot.commands.auto.DriveAndDropToOne;
import frc.robot.commands.climbing.Climb;
import frc.robot.commands.climbing.init;
import frc.robot.commands.coral.lili.LIPlaceCoral;
import frc.robot.commands.coral.nora.CoralStation;
//import frc.robot.commands.coral.nora.INtakeFromHuman;
import frc.robot.commands.coral.nora.L1;
import frc.robot.commands.coral.nora.L2;
import frc.robot.commands.coral.nora.L3;
import frc.robot.commands.drive.DriveStraight;
import frc.robot.commands.drive.DriveTwoardsAprillTag;
import frc.robot.commands.drive.DriveXMeters;
import frc.robot.commands.drive.Turn;
import frc.robot.service.MotionService;
import frc.robot.subsystems.*;
import frc.robot.subsystems.drive.DriveTrain;

import static frc.robot.datamodel.MotionDirective.drive;
import static frc.robot.datamodel.MotionDirective.turn;

// @Component
public class RobotContainer {

  Field2d visionPoseEstimate = new Field2d();
  private double up = 0.0;
  /* Controllers */
  private final Joystick driver = new Joystick(1);
  private final Joystick operator = new Joystick(2);

  /* Driver Buttons */
  private final JoystickButton zeroGyro = new JoystickButton(driver, JoystickConstants.BACK_BUTTON);

  /* Subsystems */
  //private final NoraArmSubsystem elevatorSubsystem = new NoraArmSubsystem();
  private final DriveTrain s_swerve = new DriveTrain();
  private final Vision vision = new Vision();
  // private final VisionSubsystem visionSubsystem = new VisionSubsystem(vision);
  private final NickClimbingSubsystem climbingSubsystem = new NickClimbingSubsystem();
  private final OzzyGrabberSubsystem grabber = new OzzyGrabberSubsystem();
  private final LiliCoralSubystem c = new LiliCoralSubystem();
  private final NoraArmSubsystem n = new NoraArmSubsystem();
  /* Pathplanner stuff */
  private final SendableChooser<Command> autoChoosers;

  private Field2d autoField;

  private final MotionService motionService = new MotionService(s_swerve);

  public RobotContainer() {
    System.out.println("Starting RobotContainer()");
    configureBindings();

    autoChoosers = AutoBuilder.buildAutoChooser();

    configureLogging();

    SmartDashboard.putData("Auto Chooser", autoChoosers);
    SmartDashboard.putData("Vision Pose Estimate", visionPoseEstimate);
    System.out.println("Ended RobotContainer()");
  }

  private void configureLogging() {
    autoField = new Field2d();
    SmartDashboard.putData("AutoLog", autoField);

    // Logging callback for current robot pose
    PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
      // Do whatever you want with the pose here
      autoField.setRobotPose(pose);
    });

    // Logging callback for target robot pose
    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
      // Do whatever you want with the pose here
      autoField.getObject("target pose").setPose(pose);
    });

    // Logging callback for the active path, this is sent as a list of poses
    PathPlannerLogging.setLogActivePathCallback((poses) -> {
      // Do whatever you want with the poses here
      autoField.getObject("path").setPoses(poses);
    });
  }

  public void teleopInit() {
    new init(climbingSubsystem);
    s_swerve.setDefaultCommand(
        new TelopSwerve(
            s_swerve,
            () -> driver.getRawAxis(Constants.JoystickConstants.LEFT_Y_AXIS),
            () -> driver.getRawAxis(Constants.JoystickConstants.LEFT_X_AXIS),
            () -> -driver.getTwist(),
            () -> operator.getRawButton(JoystickConstants.START_BUTTON)));
  }

  public void periodic() {
    motionService.periodic();

    SmartDashboard.putNumber("Encoder", (climbingSubsystem.ClimbingMotor1.getEncoder().getPosition() + climbingSubsystem.ClimbingMotor2.getEncoder().getPosition()));

    var visionEst = vision.getEstimatedGlobalPose();
    visionEst.ifPresent(
        est -> {
          // Change our trust in the measurement based on the tags we can see
          var estStdDevs = vision.getEstimationStdDevs();

          s_swerve.addVisionMeasurement(
              est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);

          visionPoseEstimate.setRobotPose(est.estimatedPose.toPose2d());
        });
  }
  double first = 0.0;
  public void autonomousInit() {
    first = 0.0;
  }
  public void autonomousPeriodic() {
    if(first == 0) {
      new DriveAndDropToOne(s_swerve, c).schedule();
      first++;
    }
  }

  private void configureBindings() {
    System.out.println("Starting configureBindings()");

    /* Driver Controls */
      zeroGyro.onTrue(new InstantCommand(() -> s_swerve.ResetDrives()));
    /* Operator Controls */
    //  new JoystickButton(operator, JoystickConstants.GREEN_BUTTON)
    //    .onTrue(new DriveTwoardsAprillTag(vision, s_swerve));

      new JoystickButton(operator, JoystickConstants.BLUE_BUTTON).
        and(grabber.BeamBreak()).
        onTrue(new Proceser(grabber, new JoystickButton(operator, JoystickConstants.BLUE_BUTTON))).
        or(new JoystickButton(operator, JoystickConstants.BLUE_BUTTON)).
        and(grabber.FalseBeamnBreak()).
        onTrue(new IntakeAlgae(grabber));

      // new JoystickButton(operator, 1).//JoystickConstants.GREEN_BUTTON).
      //   and(c.Coral()).
      //   onTrue(new LIPlaceCoral(c, s_swerve));

      new JoystickButton(operator, JoystickConstants.YELLOW_BUTTON).onTrue(new OzOutake(grabber));

      new JoystickButton(operator, JoystickConstants.RED_BUTTON).onTrue(new Climb(climbingSubsystem));
      
      new JoystickButton(operator, JoystickConstants.POV_LEFT).onTrue(new CoralStation(n)/*new INtakeFromHuman(n, visionSubsystem)*/);

      new JoystickButton(operator, JoystickConstants.BACK_BUTTON).onTrue(new DriveStraight(s_swerve));
      //new JoystickButton(operator, JoystickConstants.BACK_BUTTON).onTrue(new Turn(s_swerve));

      new JoystickButton(operator, JoystickConstants.GREEN_BUTTON).onTrue(new StartMotionSequence(motionService, turn(90), drive(0.5), turn(90), drive(0.5)));
    // Supplier<Pose2d> bestTargetSupplier = () -> {
    //   var target = vision.getTargets();
    //   if (target != null && kTagLayout.getTagPose(target.fiduc                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           ialId).isPresent()) {
    //     SmartDashboard.putString("Targeting tag", String.valueOf(target.getFiducialId()));
    //     return kTagLayout.getTagPose(target.fiducialId).get().toPose2d();
    //   }
    //   return null;
    // };
    
    // alternative option using PathPlanner - only if target is far enough
    // new JoystickButton(operator, JoystickConstants.BLUE_BUTTON)
    // // //.and(() -> {
    // // // return
    // bestTargetSupplier.get().getTranslation().getDistance(s_swerve.getPose().getTranslation())
    // > 2.0;
    // // //})
    // .onTrue(new PathFindToPose(s_swerve, bestTargetSupplier));

    System.out.println("Ended configureBindings()");
  }

  public void teleopPeriodic() {
    climbingSubsystem.JoyClimb(operator.getRawAxis(JoystickConstants.LEFT_Y_AXIS));
    if(operator.getRawButtonPressed(JoystickConstants.POV_UP)){
      up++;
    }
    if(up == 1) {
      new L1(n);
    } else if (up == 2) {
      new L2(n);
    } else if (up == 3) {
      new L3(n);
    } else if (up > 3) {
      up = 0;
    }
  }

  public Command getAutonomousCommand() {
    return autoChoosers.getSelected();
  }

  public void onDisable() {
    motionService.stop(true);
  }
}
