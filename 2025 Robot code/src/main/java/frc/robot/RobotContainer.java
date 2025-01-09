// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

// import org.springframework.stereotype.Component;



import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Commands.DriveTrain;
import frc.robot.Commands.TelopSwerve;
import frc.robot.Constanst.JoystickConstants;
import frc.robot.Subsystems.*;

// @Component
public class RobotContainer {

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

  /* Pathplanner stuff */
  private final SendableChooser<Command> autoChoosers;

  public RobotContainer() {

    /* Starting the Test Mode selectors*/
      test.start();
      TestMode.setDefaultOption("Use Xbox controller", Xbox);
      TestMode.addOption("Use Inputs", Input);
      TestModeSelected = TestMode.getSelected();
      SmartDashboard.putData(TestMode);
   
      s_swerve.setDefaultCommand(
        new TelopSwerve(
          s_swerve,
          () -> driver.getRawAxis(Constanst.JoystickConstants.LEFT_Y_AXIS),
          () -> -driver.getRawAxis(Constanst.JoystickConstants.LEFT_X_AXIS), 
          () -> -driver.getTwist()
          )
      );
    
    
    configureBindings();
      
    autoChoosers = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChoosers);
  }
  
  public void testPeriodic() {
   switch (TestModeSelected) {
        case Input:
          s_swerve.setDefaultCommand(
            new TelopSwerve(
              s_swerve,
              () -> test.translate(),
              () -> test.strafe(),
              () -> test.rotate()
              )
          );


          break;
      
        default:
          s_swerve.setDefaultCommand(
            new TelopSwerve(
              s_swerve,
              () -> driver.getRawAxis(Constanst.JoystickConstants.LEFT_Y_AXIS),
              () -> -driver.getRawAxis(Constanst.JoystickConstants.LEFT_X_AXIS), 
              () -> -driver.getTwist()
              )
          );

          break;
      }
  }

  private void configureBindings() {
    
    /* Driver Controls */
      zeroGyro.onTrue (new InstantCommand(() -> s_swerve.ResetDrives()));

    /* Operator Controls */
      /* Flag Controls */
        // new JoystickButton(operator, Constanst.JoystickConstants.GREEN_BUTTON).onTrue(new RunGreenFlagCommand(flag));
        // new JoystickButton(operator, Constanst.JoystickConstants.RED_BUTTON).onTrue(new RunRedFlagCommand(flag));
        // new JoystickButton(operator, Constanst.JoystickConstants.YELLOW_BUTTON).onTrue(new RunYellowFlagCommand(flag));
  }

   public Command getAutonomousCommand() {
     return autoChoosers.getSelected();
   }
}
