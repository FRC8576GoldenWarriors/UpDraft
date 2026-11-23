// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Transport.Transport;
import frc.robot.subsystems.Transport.TransportIOTalonFx;
import frc.robot.subsystems.cradle.Cradle;
import frc.robot.subsystems.cradle.CradleCANrangeIO;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.orca.Orca;
import frc.robot.subsystems.orca.Orca.WantedState;
import frc.robot.subsystems.orca.OrcaConstants;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveIOCTRE;

public class RobotContainer {
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final Orca orca;
  private final Swerve swerve;
  private final Intake intake;
  private final Transport transport;
  private final Cradle cradle;

  public RobotContainer() {
    swerve = buildSwerveSubsystem();
    intake = buildIntakeSubsystem();
    transport = buildTransportSubsystem();
    cradle = buildCradleSubsystem();

    orca = buildOrcaSubsystem();
    configureBindings();
  }

  private void configureBindings() {

    swerve.setDefaultCommand(
        swerve
            .runEnd(
                () ->
                    swerve.acceptControllerInput(
                        driverController.getLeftY(),
                        driverController.getLeftX(),
                        driverController.getRightX()),
                () -> swerve.acceptControllerInput(0, 0, 0))
            .withName("Accept Teleop Input"));

    driverController
        .start()
        .onTrue(Commands.runOnce(swerve::zeroHeading).withName("Reset Heading"));

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

    driverController
        .povDown()
        .onTrue(Commands.runOnce(() -> orca.setWantedState(Orca.WantedState.ZERO)))
        .onFalse(Commands.runOnce(() -> orca.setWantedState(Orca.WantedState.IDLE)));

    if (OrcaConstants.USE_SYS_ID_MODE) {

      driverController.a().whileTrue(intake.pivotRoutine.getForwardQuasistaticCommand());
      driverController.b().whileTrue(intake.pivotRoutine.getBackwardQuasistaticCommand());
      driverController.y().whileTrue(intake.pivotRoutine.getForwardDynamicCommand());
      driverController.x().whileTrue(intake.pivotRoutine.getBackwardDynamicCommand());

      // driverController
      //     .rightBumper()
      //     .onTrue(
      //         Commands.runOnce(
      //             () ->
      //                 orca.setWantedState(
      //                     OrcaConstants
      //                         .WANTED_SYS_ID_STATE))); // Im not sure whether to change this or
      // not
      // driverController.x().and(driverController.a()).whileTrue(swerve.getDynamicForwardCommand());
      // driverController.a().and(driverController.b()).whileTrue(swerve.getDynamicReverseCommand());
      // driverController
      //     .b()
      //     .and(driverController.y())
      //     .whileTrue(swerve.getQuasistaticForwardCommand());
      // driverController
      //     .y()
      //     .and(driverController.x())
      //     .whileTrue(swerve.getQuasistaticReverseCommand());
      // driverController
      //     .leftBumper()
      //     .whileTrue(
      //         swerve
      //             .getWheelRadiusCharacterizationCommand()
      //             .beforeStarting(
      //                 () ->
      // orca.setWantedState(Orca.WantedState.WHEEL_RADIUS_CHARACTERIZATION)));
      return;
    }

    driverController
        .a()
        .onTrue(
            Commands.runEnd(
                    () -> orca.setWantedState(WantedState.HOME_INTAKE),
                    () -> orca.setWantedState(WantedState.IDLE))
                .until(
                    () ->
                        intake.isIntakeHomed()
                            && intake.getWantedState() == Intake.WantedState.HOME));

    driverController
        .leftTrigger()
        .onTrue(Commands.runOnce(() -> orca.setWantedState(WantedState.INTAKE)));
    // .onFalse(Commands.runOnce(() -> orca.setWantedState(WantedState.RETRACT)));

    driverController
        .rightTrigger(0.25)
        .and(() -> intake.getWantedState() == Intake.WantedState.RETRACT)
        .and(() -> intake.nearSetpoint())
        .onTrue(Commands.runOnce(() -> orca.setWantedState(WantedState.OUTTAKE)))
        .onFalse(Commands.runOnce(() -> orca.setWantedState(WantedState.IDLE)));

    driverController
        .x()
        .onTrue(Commands.runOnce(() -> orca.setWantedState(WantedState.RETRACT)))
        .onFalse(Commands.runOnce(() -> orca.setWantedState(WantedState.IDLE)));
    driverController
        .y()
        .onTrue(Commands.runOnce(() -> intake.setWantedState(Intake.WantedState.POSITION_MANUALLY)))
        .onFalse(Commands.runOnce(() -> intake.setWantedState(Intake.WantedState.IDLE)));
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

  private Intake buildIntakeSubsystem() {
    return new Intake(new IntakeIOTalonFX());
  }

  public Intake getIntakeSubsystem() {
    return this.intake;
  }

  private Transport buildTransportSubsystem() {
    return new Transport(new TransportIOTalonFx());
  }

  public Transport getTransportSubsystem() {
    return this.transport;
  }

  private Cradle buildCradleSubsystem() {
    return new Cradle(new CradleCANrangeIO());
  }

  public Cradle getCradleSubsystem() {
    return this.cradle;
  }
}
