// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

// import org.springframework.stereotype.Component;



import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constanst.JoystickConstants;
import frc.robot.Subsystems.*;
import frc.robot.commands.Blue_Side_G;
import frc.robot.commands.TelopSwerve;
import frc.robot.commands.drive.DriveToPose;

// @Component
public class RobotContainer {

  Field2d field = new Field2d();
  /* Test mode choosers */
    /* Initail */
      private final SendableChooser<String> TestMode = new SendableChooser<>();
      private final String Xbox = "Use Xbox controller";
      private final String Input = "Use Inputs";
      private String TestModeSelected;
    

  /* Controllers */
    private final Joystick driver = new Joystick(JoystickConstants.DRIVER_USB);
    private final Joystick operator = new Joystick(JoystickConstants.OPERATOR_USB);

  /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, JoystickConstants.BACK_BUTTON);

  /* Subsystems */
    
    private final DriveTrain s_swerve = new DriveTrain();
    private final TestMode test = new TestMode();
    // private final CameraSubsystem cam = new CameraSubsystem();
  /* util */    
    private final Vision vision = new Vision();
    private final VisionSubsystem cam = new VisionSubsystem(vision);
  /* Pathplanner stuff */
  private final SendableChooser<Command> autoChoosers;

  public RobotContainer() {



      //cam.getCurrentCommand();
   
      
    
    
    configureBindings();
      
    autoChoosers = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChoosers);
  }

  public void teleopInit() {
    s_swerve.setDefaultCommand(
        new TelopSwerve(
          s_swerve,
          () -> driver.getRawAxis(Constanst.JoystickConstants.LEFT_Y_AXIS),
          () -> -driver.getRawAxis(Constanst.JoystickConstants.LEFT_X_AXIS), 
          () -> -driver.getTwist()
          )
      );
  }
  
  public void teleopPeriodic() {
    var visionEst = vision.getEstimatedGlobalPose();
    visionEst.ifPresent(
            est -> {
                // Change our trust in the measurement based on the tags we can see
                var estStdDevs = vision.getEstimationStdDevs();

                s_swerve.addVisionMeasurement(
                        est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);

                field.setRobotPose(est.estimatedPose.toPose2d());
                SmartDashboard.putData(field);
            });
  }
  public void testPeriodic() {
   
  }

  private void configureBindings() {
    
    /* Driver Controls */
      zeroGyro.onTrue (new InstantCommand(() -> s_swerve.ResetDrives()));

    /* Operator Controls */
      /* Automation */
      new JoystickButton(operator, JoystickConstants.GREEN_BUTTON).onTrue(new DriveToPose(s_swerve, null, null, 0));
  }

   public Command getAutonomousCommand() {
     return autoChoosers.getSelected();
   }
}
