// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.orca.Orca;
import frc.robot.subsystems.orca.OrcaConstants;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveIOCTRE;

public class RobotContainer {
  private final CommandXboxController controller = new CommandXboxController(0);
  private final Orca orca;
  private final Swerve swerve;

  public RobotContainer() {
    swerve = buildSwerveSubsystem();
    orca = buildOrcaSubsystem();
    configureBindings();
  }

  private void configureBindings() {

    swerve.setDefaultCommand(
        swerve
            .runEnd(
                () ->
                    swerve.acceptControllerInput(
                        controller.getLeftY(), controller.getLeftX(), controller.getRightX()),
                () -> swerve.acceptControllerInput(0, 0, 0))
            .withName("Accept Teleop Input"));

    controller.start().onTrue(Commands.runOnce(swerve::zeroHeading).withName("Reset Heading"));

    RobotModeTriggers.disabled()
        .onTrue(
            Commands.runOnce(() -> orca.setWantedState(Orca.WantedState.IDLE))
                .ignoringDisable(true)
                .withName("Idle"));

    RobotModeTriggers.teleop()
        .and(
            () ->
                orca.getWantedState() == Orca.WantedState.IDLE
                    || orca.getWantedState() == Orca.WantedState.TELEOP)
        .onTrue(
            Commands.runOnce(() -> orca.setWantedState(Orca.WantedState.TELEOP))
                .withName("Drive Teleop"));

    if (OrcaConstants.USE_SYS_ID_MODE) {
      controller
          .rightBumper()
          .onTrue(
              Commands.runOnce(
                  () ->
                      orca.setWantedState(
                          OrcaConstants
                              .WANTED_SYS_ID_STATE))); // Im not sure whether to change this or not
      controller.x().and(controller.a()).whileTrue(swerve.getDynamicForwardCommand());
      controller.a().and(controller.b()).whileTrue(swerve.getDynamicReverseCommand());
      controller.b().and(controller.y()).whileTrue(swerve.getQuasistaticForwardCommand());
      controller.y().and(controller.x()).whileTrue(swerve.getQuasistaticReverseCommand());
      controller
          .leftBumper()
          .whileTrue(
              swerve
                  .getWheelRadiusCharacterizationCommand()
                  .beforeStarting(
                      () -> orca.setWantedState(Orca.WantedState.WHEEL_RADIUS_CHARACTERIZATION)));
      return;
    }

    controller
        .a()
        .onTrue(
            Commands.runOnce(() -> orca.setWantedState(Orca.WantedState.ROTATION_LOCK))
                .beforeStarting(() -> swerve.setWantedRotation(new Rotation2d(Math.PI / 4)))
                .withName("Rotation Lock"))
        .onFalse(Commands.runOnce(() -> orca.setWantedState(Orca.WantedState.IDLE)));

    controller
        .b()
        .onTrue(
            Commands.runOnce(() -> orca.setWantedState(Orca.WantedState.WHEEL_LOCK_WITH_X))
                .withName("Wheel Lock With X"))
        .onFalse(Commands.runOnce(() -> orca.setWantedState(Orca.WantedState.IDLE)));

    controller
        .x()
        .onTrue(
            Commands.runOnce(() -> orca.setWantedState(Orca.WantedState.TAXI))
                .withName("Drive Taxi"))
        .onFalse(Commands.runOnce(() -> orca.setWantedState(Orca.WantedState.IDLE)));

    // // Run SysId routines when holding back/start and X/Y.
    // // Note that each routine should be run exactly once in a single log.
    // controller.back().and(controller.y()).whileTrue(swerve.sysIdDynamic(Direction.kForward));
    // controller.back().and(controller.x()).whileTrue(swerve.sysIdDynamic(Direction.kReverse));
    // controller.start().and(controller.y()).whileTrue(swerve.sysIdQuasistatic(Direction.kForward));
    // controller.start().and(controller.x()).whileTrue(swerve.sysIdQuasistatic(Direction.kReverse));

  }

  public Command getAutonomousCommand() {
    /* Run the routine selected from the auto chooser */
    return null;
  }

  private Swerve buildSwerveSubsystem() {
    SwerveModuleConstants<?, ?, ?>[] moduleConstants = new SwerveModuleConstants[4];

    moduleConstants[0] = TunerConstants.FrontLeft;
    moduleConstants[1] = TunerConstants.FrontRight;
    moduleConstants[2] = TunerConstants.BackLeft;
    moduleConstants[3] = TunerConstants.BackRight;

    return new Swerve(new SwerveIOCTRE(TunerConstants.DrivetrainConstants, moduleConstants));
  }

  public Swerve getSwerveSubsystem() {
    return this.swerve;
  }

  private Orca buildOrcaSubsystem() {
    return new Orca(this);
  }

  public Orca getOrcaSubsystem() {
    return this.orca;
  }
}
