// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.Vision.kTagLayout;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;

// import org.springframework.stereotype.Component;



import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.JoystickConstants;
import frc.robot.Subsystems.*;
import frc.robot.Subsystems.drive.DriveTrain;
import frc.robot.commands.TelopSwerve;
import frc.robot.commands.automation.reef.GoToReefSideAndPlace;
import frc.robot.commands.drive.DriveToPose;
import frc.robot.commands.drive.PathFindToPose;
import frc.robot.utils.MapleShooterOptimization;

// @Component
public class RobotContainer {

  Field2d visionPoseEstimate = new Field2d();
  /* Controllers */
    private final Joystick driver = new Joystick(JoystickConstants.DRIVER_USB);
    private final Joystick operator = new Joystick(JoystickConstants.OPERATOR_USB);

  /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, JoystickConstants.BACK_BUTTON);

  /* Subsystems */
    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    private final DriveTrain s_swerve = new DriveTrain();
    private final TestMode test = new TestMode();
    private final Vision vision = new Vision();
    private final VisionSubsystem visionSubsystem = new VisionSubsystem(vision);
  /* Pathplanner stuff */
  private final SendableChooser<Command> autoChoosers;

  public RobotContainer() {
    configureBindings();
      
    autoChoosers = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChoosers);
    SmartDashboard.putData("Vision Pose Estimate", visionPoseEstimate);
  }

  public void teleopInit() {
    s_swerve.setDefaultCommand(
        new TelopSwerve(
          s_swerve,
          () -> driver.getRawAxis(Constants.JoystickConstants.LEFT_Y_AXIS),
          () -> -driver.getRawAxis(Constants.JoystickConstants.LEFT_X_AXIS), 
          () -> -driver.getTwist()
          )
      );
  }
  
  public void periodic() {
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
  
  public void testPeriodic() {
   
  }

  private void configureBindings() {

    /* Driver Controls */
    zeroGyro.onTrue(new InstantCommand(() -> s_swerve.ResetDrives()));

    /* Operator Controls */
      /* Automation */
        new JoystickButton(operator, JoystickConstants.GREEN_BUTTON).
          toggleOnTrue(new GoToReefSideAndPlace(elevatorSubsystem, 
            s_swerve, vision, 0, 4, visionSubsystem));
    /* drive towards targeted april tag
    Supplier<Pose2d> bestTargetSupplier = () -> {
      var target = vision.getTargets();
      if (target != null && kTagLayout.getTagPose(target.fiducialId).isPresent()) {
        return kTagLayout.getTagPose(target.fiducialId).get().toPose2d();
      }
      return null;
    };
    new JoystickButton(operator, JoystickConstants.GREEN_BUTTON)
        .onTrue(new DriveToPose(s_swerve, bestTargetSupplier, null, 0));

    // alternative option using PathPlanner - only if target is far enough
    new JoystickButton(operator, JoystickConstants.BLUE_BUTTON)
        .and(() -> {
          return bestTargetSupplier.get().getTranslation().getDistance(s_swerve.getPose().getTranslation()) > 2.0;
        }).onTrue(new PathFindToPose(s_swerve, bestTargetSupplier, 1));
        */
      }

   public Command getAutonomousCommand() {
     return autoChoosers.getSelected();
   }
}
