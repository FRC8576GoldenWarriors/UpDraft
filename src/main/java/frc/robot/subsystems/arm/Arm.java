package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Rotations;

import frc.robot.util.loggedtunable.LoggedTunableDouble;

public class Arm extends ArmStateMachine {

  private ArmIO io;

  private ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  private WantedState wantedState = WantedState.IDLE;
  private SystemState systemState = SystemState.IDLING;

  private final LoggedTunableDouble manualPositionSetpoint;

  private boolean isHomed = false;

  public enum WantedState {
    IDLE,
    PICKUP_CORAL,
    CORAL_L1,
    CORAL_L2,
    CORAL_L3,
    CORAL_L4,
    ALAGE_L1,
    ALAGE_L2,
    ALAGE_FLOOR,
    ALAGE_LOLIPOP,
    BARGE,
    HOME,
    STANDBY,
    POSITION_MANUALLY,
    ZERO
  }

  private enum SystemState {
    IDLING,
    PICKING_UP_CORAL,
    MOVING_TO_CORAL_L1,
    MOVING_TO_CORAL_L2,
    MOVING_TO_CORAL_L3,
    MOVING_TO_CORAL_L4,
    MOVING_TO_ALAGE_L1,
    MOVING_TO_ALAGE_L2,
    MOVING_TO_ALAGE_FLOOR,
    MOVING_TO_ALAGE_LOLIPOP,
    MOVING_TO_BARGE,
    HOMING,
    STANDING_BY,
    POSITIONING_MANUALLY,
    ZEROING
  }

  public Arm(ArmIO io) {
    super(ArmConstants.LOG_PATH, io.getCurrentSignals());
    this.io = io;
    manualPositionSetpoint =
        newSubsystemLoggedTunableDouble(
            "ManualSetpoint", (value) -> {}, ArmConstants.TUNING_MODE_ENABLED);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    processInputs(inputs);

    systemState = handleStateTransition();
    record("WantedState", wantedState);
    record("SystemState", systemState);
    record("isHomed", isHomed);

    applyStates();
  }

  private SystemState handleStateTransition() {
    return switch (wantedState) {
      case IDLE -> SystemState.IDLING;
      case PICKUP_CORAL -> SystemState.PICKING_UP_CORAL;
      case CORAL_L1 -> SystemState.MOVING_TO_CORAL_L1;
      case CORAL_L2 -> SystemState.MOVING_TO_CORAL_L2;
      case CORAL_L3 -> SystemState.MOVING_TO_CORAL_L3;
      case CORAL_L4 -> SystemState.MOVING_TO_CORAL_L4;
      case ALAGE_L1 -> SystemState.MOVING_TO_ALAGE_L1;
      case ALAGE_L2 -> SystemState.MOVING_TO_ALAGE_L2;
      case ALAGE_FLOOR -> SystemState.MOVING_TO_ALAGE_FLOOR;
      case ALAGE_LOLIPOP -> SystemState.MOVING_TO_ALAGE_LOLIPOP;
      case BARGE -> SystemState.MOVING_TO_BARGE;
      case HOME -> SystemState.HOMING;
      case STANDBY -> SystemState.STANDING_BY;
      case POSITION_MANUALLY -> SystemState.POSITIONING_MANUALLY;
      case ZERO -> SystemState.ZEROING;
      default -> SystemState.IDLING;
    };
  }

  private void applyStates() {
    switch (systemState) {
      case IDLING -> idling();
      case PICKING_UP_CORAL -> pickingUpCoral();
      case MOVING_TO_CORAL_L1 -> movingToCoralL1();
      case MOVING_TO_CORAL_L2 -> movingToCoralL2();
      case MOVING_TO_CORAL_L3 -> movingToCoralL3();
      case MOVING_TO_CORAL_L4 -> movingToCoralL4();
      case MOVING_TO_ALAGE_L1 -> movingToAlageL1();
      case MOVING_TO_ALAGE_L2 -> movingToAlageL2();
      case MOVING_TO_ALAGE_FLOOR -> movingToAlageFloor();
      case MOVING_TO_ALAGE_LOLIPOP -> movingToAlageLolipop();
      case MOVING_TO_BARGE -> movingToBarge();
      case HOMING -> homing();
      case STANDING_BY -> standingBy();
      case POSITIONING_MANUALLY -> positioningManually();
      case ZEROING -> zeroing();
      default -> idling();
    }
  }

  @Override
  protected void idling() {
    io.idle();
  }

  @Override
  protected void pickingUpCoral() {
    io.setWantedArmPosition(ArmConstants.PICKUP_CORAL_POSITION);
  }

  @Override
  protected void movingToCoralL1() {
    io.setWantedArmPosition(ArmConstants.CORAL_L1_POSITION);
  }

  @Override
  protected void movingToCoralL2() {
    io.setWantedArmPosition(ArmConstants.CORAL_L2_POSITION);
  }

  @Override
  protected void movingToCoralL3() {
    io.setWantedArmPosition(ArmConstants.CORAL_L3_POSITION);
  }

  @Override
  protected void movingToCoralL4() {
    io.setWantedArmPosition(ArmConstants.CORAL_L4_POSITION);
  }

  @Override
  protected void movingToAlageL1() {
    io.setWantedArmPosition(ArmConstants.ALAGE_L1_POSITION);
  }

  @Override
  protected void movingToAlageL2() {
    io.setWantedArmPosition(ArmConstants.ALAGE_L2_POSITION);
  }

  @Override
  protected void movingToAlageFloor() {
    io.setWantedArmPosition(ArmConstants.ALAGE_FLOOR_POSITION);
  }

  @Override
  protected void movingToAlageLolipop() {
    io.setWantedArmPosition(ArmConstants.ALAGE_LOLIPOP_POSITION);
  }

  @Override
  protected void movingToBarge() {
    io.setWantedArmPosition(ArmConstants.BARGE_POSITION);
  }

  @Override
  protected void standingBy() {
    io.setWantedArmPosition(ArmConstants.STANDBY_POSITION);
  }

  protected void zeroing() {
    io.zero();
  }

  @Override
  protected void homing() {
    isHomed = io.home(inputs.pivotCurrent, inputs.pivotVelocity);
    if (isHomed) {
      setWantedState(WantedState.IDLE);
    }
  }

  @Override
  protected void positioningManually() {
    io.setWantedArmPosition(Rotations.of(manualPositionSetpoint.get()));
  }

  public void setWantedState(WantedState wantedState) {
    this.wantedState = wantedState;
  }

  public WantedState getWantedState() {
    return wantedState;
  }

  public boolean isHomed() {
    return isHomed;
  }

  public boolean nearSetpoint() {
    return io.nearSetpoint();
  }
}
