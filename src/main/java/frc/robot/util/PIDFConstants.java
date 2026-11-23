package frc.robot.util;

import com.ctre.phoenix6.configs.Slot0Configs;

public class PIDFConstants {

  private double kP;
  private double kI;
  private double kD;
  private double kS;
  private double kG;
  private double kV;
  private double kA;

  public PIDFConstants() {
    kP = kI = kD = kS = kG = kV = kA = 0.0;
  }

  public PIDFConstants(double kP, double kI, double kD) {
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    kS = kG = kV = kA = 0.0;
  }

  public PIDFConstants(
      double kP, double kI, double kD, double kS, double kG, double kV, double kA) {
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    this.kS = kS;
    this.kG = kG;
    this.kV = kV;
    this.kA = kA;
  }

  public PIDFConstants(double[] pidfConstants) {
    if (pidfConstants == null || pidfConstants.length < 7)
      throw new IllegalArgumentException(
          "Number of provided PIDSGVA values does not match! Make sure you are passing 7 values");

    this.kP = pidfConstants[0];
    this.kI = pidfConstants[1];
    this.kD = pidfConstants[2];
    this.kS = pidfConstants[3];
    this.kG = pidfConstants[4];
    this.kV = pidfConstants[5];
    this.kA = pidfConstants[6];
  }

  public static Slot0Configs toSlot0Configs(PIDFConstants pidfConstants) {
    return new Slot0Configs()
        .withKP(pidfConstants.kP)
        .withKI(pidfConstants.kI)
        .withKD(pidfConstants.kD)
        .withKS(pidfConstants.kS)
        .withKG(pidfConstants.kG)
        .withKV(pidfConstants.kV)
        .withKA(pidfConstants.kA);
  }

  public static PIDFConstants fromSlot0Configs(Slot0Configs slot0Configs) {
    return new PIDFConstants(
        slot0Configs.kP,
        slot0Configs.kI,
        slot0Configs.kD,
        slot0Configs.kS,
        slot0Configs.kG,
        slot0Configs.kV,
        slot0Configs.kA);
  }

  public static double[] toDoubleArray(PIDFConstants pidfConstants) {
    return new double[] {
      pidfConstants.kP,
      pidfConstants.kI,
      pidfConstants.kD,
      pidfConstants.kS,
      pidfConstants.kG,
      pidfConstants.kV,
      pidfConstants.kA
    };
  }
}
