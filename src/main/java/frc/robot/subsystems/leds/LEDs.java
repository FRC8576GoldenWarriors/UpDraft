// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import frc.robot.util.loggedimpl.LoggedSubsystem;
import frc.robot.util.loggedimpl.LoggedSystemState;
import frc.robot.util.loggedimpl.LoggedWantedState;
import java.util.ArrayList;
import java.util.List;

public class LEDs extends LoggedSubsystem {

  private final AddressableLED ledStrip;
  private final AddressableLEDBuffer ledBuffer;
  private final List<AddressableLEDBufferView> ledSubsections;

  public enum WantedState implements LoggedWantedState {
    IDLE
  }

  private enum SystemState implements LoggedSystemState {
    IDLING
  }

  private WantedState wantedState = WantedState.IDLE;
  private SystemState systemState = SystemState.IDLING;

  public LEDs() {
    super(LEDConstants.LOG_PATH, null, true);
    ledStrip = new AddressableLED(LEDConstants.PORT);

    int ledsPerSection = LEDConstants.STRIP_LENGTH / LEDConstants.SECTION_AMOUNT;

    ledBuffer = new AddressableLEDBuffer(LEDConstants.STRIP_LENGTH);
    ledStrip.setLength(ledBuffer.getLength());

    ledSubsections = new ArrayList<>();

    for (int i = 0; i < LEDConstants.SECTION_AMOUNT; i++) {
      ledSubsections.add(ledBuffer.createView(i * ledsPerSection, (i + 1) * ledsPerSection));
    }

    ledStrip.start();
  }

  public void setWantedState(WantedState wantedState) {
    this.wantedState = wantedState;
  }

  public WantedState getWantedState() {
    return wantedState;
  }

  @Override
  public void periodic() {
    systemState = handleStateTransition();

    recordWantedState(wantedState);
    recordSystemState(systemState);

    applyStates();

    ledStrip.setData(ledBuffer);
  }

  private SystemState handleStateTransition() {
    return switch (wantedState) {
      case IDLE -> SystemState.IDLING;
    };
  }

  private void applyStates() {
    switch (systemState) {
      case IDLING -> idling();
    }
  }

  private void idling() {
    for (AddressableLEDBufferView view : ledSubsections) LEDConstants.Patterns.IDLE.applyTo(view);
  }
}
