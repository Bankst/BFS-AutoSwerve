// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.auton.Paths;
import frc.robot.subsystems.SwerveSubsystem;

import static frc.robot.auton.AutonFactory.autonEventMap;

public class RobotContainer {
  /* Controllers */
  private final CommandXboxController driver = new CommandXboxController(0);

  public final AprilTagCamera vision = new AprilTagCamera();
  public final SwerveSubsystem swerve = new SwerveSubsystem(autonEventMap, vision);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    swerve.setDefaultCommand(
      swerve.teleopDriveCmd(
        () -> -driver.getLeftY(),
        () -> -driver.getLeftX(),
        () -> driver.getRightX(),
        driver.leftBumper()::getAsBoolean,
        () -> true // openLoop
      ));
  }

  public Command getAutonomousCommand() {
    return swerve.getFullAuto(Paths.PP.testPath);
  }
}
