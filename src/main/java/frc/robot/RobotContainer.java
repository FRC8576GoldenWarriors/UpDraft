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
import frc.robot.subsystems.orca.Orca.WantedState;
import frc.robot.subsystems.orca.OrcaConstants;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConstants;
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

    configureDefaultBindings();

    if (OrcaConstants.USE_SYS_ID_MODE) {
      configureSysIDBindings();
    } else {
      configureOrcaBindings();
    }
  }

  private void configureDefaultBindings() {
    swerve.setDefaultCommand(
        swerve
            .runEnd(
                () ->
                    swerve.acceptControllerInput(
                        -controller.getLeftY(), -controller.getLeftX(), -controller.getRightX()),
                () -> swerve.acceptControllerInput(0, 0, 0))
            .withName("AcceptTeleopInput"));

    controller.start().onTrue(Commands.runOnce(orca::resetSwerveHeading).withName("ResetHeading"));

    RobotModeTriggers.disabled()
        .onTrue(orca.commandWantedState(WantedState.IDLE, "Idle").ignoringDisable(true));

    RobotModeTriggers.teleop()
        .and(() -> orca.stateEqualsAny(WantedState.IDLE, WantedState.TELEOP))
        .onTrue(orca.commandWantedState(WantedState.TELEOP, "DriveTeleop"));
  }

  private void configureSysIDBindings() {
    controller
        .rightBumper()
        .onTrue(orca.commandWantedState(OrcaConstants.WANTED_SYS_ID_STATE, "SysID"));

    controller
        .leftBumper()
        .onTrue(orca.commandWantedState(WantedState.WHEEL_RADIUS_CHARACTERIZATION, "WheelRadius"))
        .whileTrue(orca.characterizeWheelRadius().withName("CharacterizeWheelRadius"));

    controller.povRight().whileTrue(swerve.getDynamicForwardCommand());
    controller.povUp().whileTrue(swerve.getDynamicReverseCommand());
    controller.povLeft().whileTrue(swerve.getQuasistaticForwardCommand());
    controller.povDown().whileTrue(swerve.getQuasistaticReverseCommand());

    controller
        .x()
        .onTrue(orca.commandWantedState(OrcaConstants.WANTED_SYS_ID_STATE, "SysID"))
        .onTrue(
            Commands.runOnce(
                () ->
                    swerve.setWantedAbsoluteModuleRotations(
                        SwerveConstants.MODULE_ROTATIONS_FOR_TRANSLATION)));
  }

  private void configureOrcaBindings() {
    controller
        .a()
        .onTrue(
            Commands.sequence(
                Commands.runOnce(() -> orca.setWantedRotation(Rotation2d.k180deg)),
                orca.commandWantedState(WantedState.ROTATION_LOCK, "RotationLock")))
        .onFalse(orca.commandWantedState(WantedState.IDLE));

    controller
        .b()
        .onTrue(orca.commandWantedState(WantedState.WHEEL_LOCK_WITH_X, "WheelLockWithX"))
        .onFalse(orca.commandWantedState(WantedState.IDLE));

    controller
        .x()
        .onTrue(orca.commandWantedState(WantedState.TAXI, "DriveTaxi"))
        .onFalse(orca.commandWantedState(WantedState.IDLE));
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
