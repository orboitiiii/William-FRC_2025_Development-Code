package frc.robot.slsh.math.plant;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.slsh.math.system.LinearSystem_NoGC;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.simple.SimpleMatrix;

/**
 * 為常見 FRC 機構建立狀態空間模型的工廠類別（零記憶體分配版本）。
 *
 * <p>這個工具類別使用第一原理建模，從物理參數推導線性系統。 它產生的所有系統都使用 {@link LinearSystem_NoGC} 和 {@link SimpleMatrix}，
 * 以便與無垃圾回收的控制迴圈完全相容。
 */
public final class StateSpaceFactory_NoGC {
  private StateSpaceFactory_NoGC() {
    // Utility class
  }

  // =================================================================================
  //
  // 單自由度系統 (Single-DOF Systems)
  //
  // =================================================================================

  /**
   * 為由直流馬達驅動的通用單自由度系統建立一個狀態空間模型。
   *
   * <p>狀態: [位置, 速度]ᵀ, 輸入: [電壓], 輸出: [位置, 速度]ᵀ.
   *
   * @param motor 馬達或齒輪箱
   * @param J_eff_KgMetersSquared 系統在輸出端的等效轉動慣量
   * @param gearing 齒輪比 (輸出/輸入)
   * @return 代表給定物理常數的 LinearSystem_NoGC
   */
  private static LinearSystem_NoGC createSingleDOFSystem(
      DCMotor motor, double J_eff_KgMetersSquared, double gearing) {
    if (J_eff_KgMetersSquared <= 0.0) {
      throw new IllegalArgumentException("等效慣量必須大於零。");
    }
    if (gearing <= 0.0) {
      throw new IllegalArgumentException("齒輪比必須大於零。");
    }

    SimpleMatrix A = new SimpleMatrix(2, 2);
    A.set(0, 1, 1.0);
    A.set(
        1,
        1,
        -Math.pow(gearing, 2)
            * motor.KtNMPerAmp
            / (motor.KvRadPerSecPerVolt * motor.rOhms * J_eff_KgMetersSquared));

    SimpleMatrix B = new SimpleMatrix(2, 1);
    B.set(1, 0, gearing * motor.KtNMPerAmp / (motor.rOhms * J_eff_KgMetersSquared));

    SimpleMatrix C = SimpleMatrix.identity(2);
    SimpleMatrix D = new SimpleMatrix(2, 1);

    return new LinearSystem_NoGC(A, B, C, D);
  }

  /** 建立電梯系統的狀態空間模型。 */
  public static LinearSystem_NoGC createElevatorSystem(
      DCMotor motor, double massKg, double radiusMeters, double gearing) {
    double effectiveInertia = massKg * radiusMeters * radiusMeters;
    return createSingleDOFSystem(motor, effectiveInertia, gearing);
  }

  /** 建立單關節手臂的狀態空間模型。 */
  public static LinearSystem_NoGC createSingleJointedArmSystem(
      DCMotor motor, double JKgMetersSquared, double gearing) {
    return createSingleDOFSystem(motor, JKgMetersSquared, gearing);
  }

  /** 建立飛輪系統的狀態空間模型。 狀態: [角速度], 輸入: [電壓], 輸出: [角速度]. */
  public static LinearSystem_NoGC createFlywheelSystem(
      DCMotor motor, double JKgMetersSquared, double gearing) {
    if (JKgMetersSquared <= 0.0) {
      throw new IllegalArgumentException("J 必須大於零。");
    }
    if (gearing <= 0.0) {
      throw new IllegalArgumentException("齒輪比必須大於零。");
    }
    SimpleMatrix A = new SimpleMatrix(1, 1);
    A.set(
        0,
        0,
        -Math.pow(gearing, 2)
            * motor.KtNMPerAmp
            / (motor.KvRadPerSecPerVolt * motor.rOhms * JKgMetersSquared));

    SimpleMatrix B = new SimpleMatrix(1, 1);
    B.set(0, 0, gearing * motor.KtNMPerAmp / (motor.rOhms * JKgMetersSquared));

    SimpleMatrix C = SimpleMatrix.identity(1);
    SimpleMatrix D = new SimpleMatrix(1, 1);

    return new LinearSystem_NoGC(A, B, C, D);
  }

  // =================================================================================
  //
  // 系統增強 (System Augmentation)
  //
  // =================================================================================

  /** 增強一個 2-狀態, 1-輸入, 2-輸出的系統以估計輸入誤差。 */
  public static LinearSystem_NoGC augmentWithInputError(LinearSystem_NoGC system) {
    int states = system.getNumStates();
    int inputs = system.getNumInputs();
    int outputs = system.getNumOutputs();

    SimpleMatrix oldA = system.getA();
    SimpleMatrix oldB = system.getB();
    SimpleMatrix oldC = system.getC();
    SimpleMatrix oldD = system.getD();

    int newStates = states + inputs;

    SimpleMatrix newA = new SimpleMatrix(newStates, newStates);
    SimpleMatrix newB = new SimpleMatrix(newStates, inputs);
    SimpleMatrix newC = new SimpleMatrix(outputs, newStates);
    SimpleMatrix newD = new SimpleMatrix(outputs, inputs);

    // A_aug = [A, B]
    //         [0, 0]
    CommonOps_DDRM.insert(oldA.getDDRM(), newA.getDDRM(), 0, 0);
    CommonOps_DDRM.insert(oldB.getDDRM(), newA.getDDRM(), 0, states);

    // B_aug = [B]
    //         [0]
    CommonOps_DDRM.insert(oldB.getDDRM(), newB.getDDRM(), 0, 0);

    // C_aug = [C, D]
    CommonOps_DDRM.insert(oldC.getDDRM(), newC.getDDRM(), 0, 0);
    CommonOps_DDRM.insert(oldD.getDDRM(), newC.getDDRM(), 0, states);

    // D_aug = [D]
    CommonOps_DDRM.insert(oldD.getDDRM(), newD.getDDRM(), 0, 0);

    return new LinearSystem_NoGC(newA, newB, newC, newD);
  }

  // =================================================================================
  //
  // 非線性系統 (Nonlinear Systems) - 二連桿手臂
  //
  // =================================================================================

  /** 一個持有二連桿手臂所有物理參數的 record。 這個 NoGC 版本完全使用 SimpleMatrix。 */
  public record TwoJointedArmPlant_NoGC(
      DCMotor motor,
      double m1,
      double l1,
      double r1,
      double I1,
      double m2,
      double l2,
      double r2,
      double I2,
      double G1,
      double G2,
      int numMotors1,
      int numMotors2,
      double g) {

    /**
     * 線性化手臂在一個靜態操作點（零速度）周圍的動態。
     *
     * @param operatingAngles 一個 [theta1, theta2] 向量 (rad)，代表線性化的中心點。
     * @return 代表手臂在該點附近動態的 LinearSystem_NoGC。
     */
    public LinearSystem_NoGC linearize(SimpleMatrix operatingAngles) {
      final double t1 = operatingAngles.get(0, 0);
      final double t2 = operatingAngles.get(1, 0);

      final double s1 = Math.sin(t1);
      final double c1 = Math.cos(t1);
      final double s2 = Math.sin(t2);
      final double c2 = Math.cos(t2);
      final double s12 = Math.sin(t1 + t2);
      final double c12 = Math.cos(t1 + t2);

      // --- 在操作點重新計算矩陣 ---
      final double M11 = m1 * r1 * r1 + m2 * (l1 * l1 + r2 * r2) + I1 + I2 + 2 * m2 * l1 * r2 * c2;
      final double M12 = m2 * r2 * r2 + I2 + m2 * l1 * r2 * c2;
      final double M22 = m2 * r2 * r2 + I2;
      SimpleMatrix M = new SimpleMatrix(new double[][] {{M11, M12}, {M12, M22}});
      SimpleMatrix M_inv = M.invert();

      final double R = motor.rOhms;
      final double Kt = motor.KtNMPerAmp;
      final double Kv = motor.KvRadPerSecPerVolt;
      SimpleMatrix K_b =
          new SimpleMatrix(
              new double[][] {
                {G1 * G1 * numMotors1 * Kt / (Kv * R), 0},
                {0, G2 * G2 * numMotors2 * Kt / (Kv * R)}
              });

      SimpleMatrix B_m =
          new SimpleMatrix(
              new double[][] {
                {G1 * numMotors1 * Kt / R, 0},
                {0, G2 * numMotors2 * Kt / R}
              });

      final double d_tau_g1_d_t1 = -(m1 * r1 + m2 * l1) * g * s1 - m2 * r2 * g * s12;
      final double d_tau_g1_d_t2 = -m2 * r2 * g * s12;
      final double d_tau_g2_d_t1 = -m2 * r2 * g * s12;
      final double d_tau_g2_d_t2 = -m2 * r2 * g * s12;
      SimpleMatrix d_tau_g_d_theta =
          new SimpleMatrix(
              new double[][] {
                {d_tau_g1_d_t1, d_tau_g1_d_t2},
                {d_tau_g2_d_t1, d_tau_g2_d_t2}
              });

      // --- 組裝線性化的狀態空間矩陣 A 和 B ---
      SimpleMatrix A21 = M_inv.mult(d_tau_g_d_theta).scale(-1.0);
      SimpleMatrix A22 = M_inv.mult(K_b).scale(-1.0);

      SimpleMatrix A = new SimpleMatrix(4, 4);
      CommonOps_DDRM.insert(SimpleMatrix.identity(2).getDDRM(), A.getDDRM(), 0, 2);
      CommonOps_DDRM.insert(A21.getDDRM(), A.getDDRM(), 2, 0);
      CommonOps_DDRM.insert(A22.getDDRM(), A.getDDRM(), 2, 2);

      SimpleMatrix B2 = M_inv.mult(B_m);
      SimpleMatrix B = new SimpleMatrix(4, 2);
      CommonOps_DDRM.insert(B2.getDDRM(), B.getDDRM(), 2, 0);

      SimpleMatrix C = SimpleMatrix.identity(4);
      SimpleMatrix D = new SimpleMatrix(4, 2);

      return new LinearSystem_NoGC(A, B, C, D);
    }
  }

  /** 用於創建 TwoJointedArmPlant_NoGC 的工廠方法。 */
  public static TwoJointedArmPlant_NoGC createTwoJointedArmPlant(
      DCMotor motor,
      double m1,
      double l1,
      double r1,
      double I1,
      double m2,
      double l2,
      double r2,
      double I2,
      double G1,
      double G2,
      int numMotors1,
      int numMotors2) {
    return new TwoJointedArmPlant_NoGC(
        motor, m1, l1, r1, I1, m2, l2, r2, I2, G1, G2, numMotors1, numMotors2, 9.80665);
  }
}
