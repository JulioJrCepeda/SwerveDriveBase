// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AlignWheelsCommand;
import frc.robot.subsystems.Drivetrain;

import java.util.function.DoubleSupplier;

//import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.auto.NamedCommands;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  private final Drivetrain m_drivetrain = new Drivetrain();

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  // private final SendableChooser<Command> autoChooser;

  public RobotContainer() {

    CommandScheduler.getInstance().schedule(new AlignWheelsCommand(m_drivetrain));


    // autoChooser = AutoBuilder.buildAutoChooser();

    m_drivetrain.setDefaultCommand(
        new RunCommand(
          () -> m_drivetrain.drive(
            applyDeadband(m_driverController::getLeftY, 0.1), 
            applyDeadband(m_driverController::getLeftX, 0.1), 
            applyDeadband(m_driverController::getRightX, 0.1)
          ),
          m_drivetrain
        )
      );

      // SmartDashboard.putData("Auto Chooser", autoChooser);


      configureBindings();
  }
  private DoubleSupplier applyDeadband(DoubleSupplier supplier, double deadband) {
    return () -> {
        double value = supplier.getAsDouble();
        if (Math.abs(value) > deadband) {
            return value;
        } else {
            return 0.0;
        }
     };
   }

  private void configureBindings() {

    //TODO  m_driverController.start().onTrue(
		// 	 new InstantCommand(() -> m_drivetrain.resetPose(new Pose2d()), m_drivetrain)
		//  );
    m_driverController.a().onTrue(new AlignWheelsCommand(m_drivetrain));
  }

  public Command getAutonomousCommand() {
    return null; //TODO use -> autoChooser.getSelected();
  }
} 
