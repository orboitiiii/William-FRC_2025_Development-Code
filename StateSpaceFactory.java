// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.slsh.math.plant;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;

/**
 * A factory for creating state-space models for common FRC mechanisms.
 *
 * <p>This utility class uses first-principles modeling to derive linear systems from physical
 * parameters. It also provides methods for augmenting systems with input error estimation states
 * for robust control.
 */
public final class StateSpaceFactory {
  private StateSpaceFactory() {
    // Utility class
  }

  // =================================================================================
  //
  // Private Helper for Single-DOF Systems
  //
  // =================================================================================

  /**
   * Creates a state-space model for a generic single-degree-of-freedom system driven by a DC motor.
   *
   * <p>States are [position, velocity]ᵀ, input is [voltage], and outputs are [position, velocity]ᵀ.
   *
   * @param motor The motor (or gearbox) attached to the system.
   * @param J_eff_KgMetersSquared The effective moment of inertia of the system at the output shaft.
   *     For linear systems like elevators, this is calculated as mass * radius^2.
   * @param gearing The gearing between the motor and the output, as a ratio of output to input.
   * @return A LinearSystem representing the given characterized constants.
   */
  private static LinearSystem<N2, N1, N2> createSingleDOFSystem(
      DCMotor motor, double J_eff_KgMetersSquared, double gearing) {
    if (J_eff_KgMetersSquared <= 0.0) {
      throw new IllegalArgumentException("Effective inertia must be greater than zero.");
    }
    if (gearing <= 0.0) {
      throw new IllegalArgumentException("Gearing must be greater than zero.");
    }

    return new LinearSystem<>(
        MatBuilder.fill(
            Nat.N2(),
            Nat.N2(),
            0,
            1,
            0,
            -Math.pow(gearing, 2)
                * motor.KtNMPerAmp
                / (motor.KvRadPerSecPerVolt * motor.rOhms * J_eff_KgMetersSquared)),
        VecBuilder.fill(0, gearing * motor.KtNMPerAmp / (motor.rOhms * J_eff_KgMetersSquared)),
        Matrix.eye(Nat.N2()),
        new Matrix<>(Nat.N2(), Nat.N1()));
  }

  // =================================================================================
  //
  // Public Factory Methods for Standard and Augmented Systems
  //
  // =================================================================================

  /**
   * Create a state-space model of an elevator system. States: [position, velocity]ᵀ, Input:
   * [voltage], Outputs: [position, velocity]ᵀ.
   */
  public static LinearSystem<N2, N1, N2> createElevatorSystem(
      DCMotor motor, double massKg, double radiusMeters, double gearing) {
    // Effective inertia for a linear system is J_eff = m * r^2
    double effectiveInertia = massKg * radiusMeters * radiusMeters;
    return createSingleDOFSystem(motor, effectiveInertia, gearing);
  }

  /**
   * Creates an augmented state-space model of an elevator system for input error estimation.
   * States: [position, velocity, voltageError]ᵀ.
   */
  public static LinearSystem<N3, N1, N2> createElevatorSystemWithInputError(
      DCMotor motor, double massKg, double radiusMeters, double gearing) {
    return augmentWithInputError_two_one_two(
        createElevatorSystem(motor, massKg, radiusMeters, gearing));
  }

  /**
   * Create a state-space model of a single-jointed arm system. States: [angle, angular velocity]ᵀ,
   * Input: [voltage], Outputs: [angle, angular velocity]ᵀ.
   */
  public static LinearSystem<N2, N1, N2> createSingleJointedArmSystem(
      DCMotor motor, double JKgMetersSquared, double gearing) {
    return createSingleDOFSystem(motor, JKgMetersSquared, gearing);
  }

  /**
   * Creates an augmented state-space model of a single-jointed arm system for input error
   * estimation. States: [angle, angular velocity, voltageError]ᵀ.
   */
  public static LinearSystem<N3, N1, N2> createSingleJointedArmSystemWithInputError(
      DCMotor motor, double JKgMetersSquared, double gearing) {
    return augmentWithInputError_two_one_two(
        createSingleJointedArmSystem(motor, JKgMetersSquared, gearing));
  }

  /**
   * Create a state-space model of a flywheel system. States: [angular velocity], Input: [voltage],
   * Outputs: [angular velocity].
   */
  public static LinearSystem<N1, N1, N1> createFlywheelSystem(
      DCMotor motor, double JKgMetersSquared, double gearing) {
    if (JKgMetersSquared <= 0.0) {
      throw new IllegalArgumentException("J must be greater than zero.");
    }
    if (gearing <= 0.0) {
      throw new IllegalArgumentException("Gearing must be greater than zero.");
    }
    return new LinearSystem<>(
        VecBuilder.fill(
            -Math.pow(gearing, 2)
                * motor.KtNMPerAmp
                / (motor.KvRadPerSecPerVolt * motor.rOhms * JKgMetersSquared)),
        VecBuilder.fill(gearing * motor.KtNMPerAmp / (motor.rOhms * JKgMetersSquared)),
        Matrix.eye(Nat.N1()),
        new Matrix<>(Nat.N1(), Nat.N1()));
  }

  /**
   * Creates an augmented state-space model of a flywheel system for input error estimation. States:
   * [angular velocity, voltageError]ᵀ.
   */
  public static LinearSystem<N2, N1, N1> createFlywheelSystemWithInputError(
      DCMotor motor, double JKgMetersSquared, double gearing) {
    return augmentWithInputError_one_one_one(
        createFlywheelSystem(motor, JKgMetersSquared, gearing));
  }

  /**
   * Create a state-space model of a differential swerve drive module.
   *
   * <p>States: [azimuth angle, azimuth velocity, wheel velocity]ᵀ, Inputs: [top voltage, bottom
   * voltage]ᵀ, Outputs: [azimuth angle, azimuth velocity, wheel velocity]ᵀ.
   */
  public static LinearSystem<N3, N2, N3> createDifferentialSwerveSystem(
      DCMotor motor,
      double J_a_KgMetersSquared,
      double J_w_KgMetersSquared,
      double G_a,
      double G_w) {

    final double C_a = (G_a * motor.KtNMPerAmp) / (J_a_KgMetersSquared * motor.rOhms);
    final double C_w = (G_w * motor.KtNMPerAmp) / (J_w_KgMetersSquared * motor.rOhms);
    final double Kv = motor.KvRadPerSecPerVolt;

    Matrix<N3, N3> A =
        MatBuilder.fill(
            Nat.N3(),
            Nat.N3(),
            0.0,
            1.0,
            0.0,
            0.0,
            -G_a * C_a / Kv,
            0.0,
            0.0,
            0.0,
            -G_w * C_w / Kv);

    Matrix<N3, N2> B =
        MatBuilder.fill(Nat.N3(), Nat.N2(), 0.0, 0.0, 0.5 * C_a, -0.5 * C_a, 0.5 * C_w, 0.5 * C_w);

    return new LinearSystem<>(A, B, Matrix.eye(Nat.N3()), new Matrix<>(Nat.N3(), Nat.N2()));
  }

  /**
   * Creates an augmented state-space model of a differential swerve drive module for input error
   * estimation. States: [azimuth angle, azimuth velocity, wheel velocity, topVoltageError,
   * bottomVoltageError]ᵀ.
   */
  public static LinearSystem<N5, N2, N3> createDifferentialSwerveSystemWithInputError(
      DCMotor motor,
      double J_a_KgMetersSquared,
      double J_w_KgMetersSquared,
      double G_a,
      double G_w) {
    return augmentWithInputError_three_two_three(
        createDifferentialSwerveSystem(motor, J_a_KgMetersSquared, J_w_KgMetersSquared, G_a, G_w));
  }

  // =================================================================================
  //
  // Nonlinear System Plant Classes
  //
  // =================================================================================

  /**
   * A record to hold all physical parameters for a two-jointed arm. This is used because the arm's
   * dynamics are nonlinear and its system matrices change with its state.
   */
  public record TwoJointedArmPlant(
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
     * Calculates the feedforward voltages for the two joints required to hold a given state or
     * achieve a given acceleration.
     *
     * @param angles A vector [theta1, theta2] in radians.
     * @param velocities A vector [omega1, omega2] in rad/s.
     * @param accelerations A vector [alpha1, alpha2] in rad/s^2.
     * @return A vector [voltage1, voltage2] of feedforward voltages.
     */
    public Matrix<N2, N1> calculateFeedforward(
        Matrix<N2, N1> angles, Matrix<N2, N1> velocities, Matrix<N2, N1> accelerations) {
      final double t1 = angles.get(0, 0);
      final double t2 = angles.get(1, 0);
      final double v1 = velocities.get(0, 0);
      final double v2 = velocities.get(1, 0);

      final double c1 = Math.cos(t1);
      final double s2 = Math.sin(t2);
      final double c2 = Math.cos(t2);
      final double c12 = Math.cos(t1 + t2);

      // Inertia Matrix M(theta) from PDF
      final double M11 = m1 * r1 * r1 + m2 * (l1 * l1 + r2 * r2) + I1 + I2 + 2 * m2 * l1 * r2 * c2;
      final double M12 = m2 * r2 * r2 + I2 + m2 * l1 * r2 * c2;
      final double M22 = m2 * r2 * r2 + I2;
      Matrix<N2, N2> M = MatBuilder.fill(Nat.N2(), Nat.N2(), M11, M12, M12, M22);

      // Coriolis and Centrifugal Matrix C(theta, theta_dot) from PDF
      final double h = m2 * l1 * r2;
      Matrix<N2, N2> C =
          MatBuilder.fill(Nat.N2(), Nat.N2(), -h * s2 * v2, -h * s2 * (v1 + v2), h * s2 * v1, 0);

      // Gravity Torque Vector tau_g(theta) from PDF
      Matrix<N2, N1> tau_g =
          VecBuilder.fill((m1 * r1 + m2 * l1) * g * c1 + m2 * r2 * g * c12, m2 * r2 * g * c12);

      // Motor constants and Back-EMF torque matrix K_b from PDF
      final double R = motor.rOhms;
      final double Kt = motor.KtNMPerAmp;
      final double Kv = motor.KvRadPerSecPerVolt;
      Matrix<N2, N2> K_b =
          MatBuilder.fill(
              Nat.N2(),
              Nat.N2(),
              G1 * G1 * numMotors1 * Kt / (Kv * R),
              0,
              0,
              G2 * G2 * numMotors2 * Kt / (Kv * R));

      // Input voltage to torque matrix B from PDF
      Matrix<N2, N2> B_matrix =
          MatBuilder.fill(
              Nat.N2(), Nat.N2(), G1 * numMotors1 * Kt / R, 0, 0, G2 * numMotors2 * Kt / R);

      // Full dynamics equation: u = B^-1 * (M * alpha + C * v + tau_g + K_b * v)
      Matrix<N2, N1> totalTorque =
          M.times(accelerations).plus(C.times(velocities)).plus(tau_g).plus(K_b.times(velocities));

      return B_matrix.inv().times(totalTorque);
    }

    /**
     * Linearizes the arm dynamics around a stationary operating point (zero velocity).
     *
     * @param operatingAngles A vector [theta1, theta2] in radians representing the linearization
     *     point.
     * @return A LinearSystem representing the arm's dynamics near the operating point. The states
     *     are [theta1, theta2, omega1, omega2]ᵀ. The inputs are [voltage1, voltage2]ᵀ.
     */
    public LinearSystem<N4, N2, N4> linearize(Matrix<N2, N1> operatingAngles) {
      final double t1 = operatingAngles.get(0, 0);
      final double t2 = operatingAngles.get(1, 0);

      final double s1 = Math.sin(t1);
      final double c1 = Math.cos(t1);
      final double s2 = Math.sin(t2);
      final double c2 = Math.cos(t2);
      final double s12 = Math.sin(t1 + t2);
      final double c12 = Math.cos(t1 + t2);

      // --- Recalculate matrices at the operating point ---

      // Inertia Matrix M(theta)
      final double M11 = m1 * r1 * r1 + m2 * (l1 * l1 + r2 * r2) + I1 + I2 + 2 * m2 * l1 * r2 * c2;
      final double M12 = m2 * r2 * r2 + I2 + m2 * l1 * r2 * c2;
      final double M22 = m2 * r2 * r2 + I2;
      Matrix<N2, N2> M = MatBuilder.fill(Nat.N2(), Nat.N2(), M11, M12, M12, M22);
      Matrix<N2, N2> M_inv = M.inv();

      // Back-EMF torque matrix K_b
      final double R = motor.rOhms;
      final double Kt = motor.KtNMPerAmp;
      final double Kv = motor.KvRadPerSecPerVolt;
      Matrix<N2, N2> K_b =
          MatBuilder.fill(
              Nat.N2(),
              Nat.N2(),
              G1 * G1 * numMotors1 * Kt / (Kv * R),
              0,
              0,
              G2 * G2 * numMotors2 * Kt / (Kv * R));

      // Input voltage to torque matrix B_m
      Matrix<N2, N2> B_m =
          MatBuilder.fill(
              Nat.N2(), Nat.N2(), G1 * numMotors1 * Kt / R, 0, 0, G2 * numMotors2 * Kt / R);

      // Jacobian of the gravity vector tau_g w.r.t. theta
      final double d_tau_g1_d_t1 = -(m1 * r1 + m2 * l1) * g * s1 - m2 * r2 * g * s12;
      final double d_tau_g1_d_t2 = -m2 * r2 * g * s12;
      final double d_tau_g2_d_t1 = -m2 * r2 * g * s12;
      final double d_tau_g2_d_t2 = -m2 * r2 * g * s12;
      Matrix<N2, N2> d_tau_g_d_theta =
          MatBuilder.fill(
              Nat.N2(), Nat.N2(), d_tau_g1_d_t1, d_tau_g1_d_t2, d_tau_g2_d_t1, d_tau_g2_d_t2);

      // --- Assemble the linearized state-space matrices A and B ---

      // A = [[0, I], [A21, A22]]
      // A21 = -M^-1 * (d(tau_g)/d(theta))
      Matrix<N2, N2> A21 = M_inv.times(d_tau_g_d_theta).times(-1.0);
      // A22 = -M^-1 * K_b
      Matrix<N2, N2> A22 = M_inv.times(K_b).times(-1.0);

      Matrix<N4, N4> A = new Matrix<>(Nat.N4(), Nat.N4());
      A.assignBlock(0, 2, Matrix.eye(Nat.N2())); // Top-right block is identity
      A.assignBlock(2, 0, A21); // Bottom-left block
      A.assignBlock(2, 2, A22); // Bottom-right block

      // B = [[0], [B2]]
      // B2 = M^-1 * B_m
      Matrix<N2, N2> B2 = M_inv.times(B_m);

      Matrix<N4, N2> B = new Matrix<>(Nat.N4(), Nat.N2());
      B.assignBlock(2, 0, B2); // Bottom block

      // Assume we can measure all states
      Matrix<N4, N4> C = Matrix.eye(Nat.N4());
      Matrix<N4, N2> D = new Matrix<>(Nat.N4(), Nat.N2());

      return new LinearSystem<>(A, B, C, D);
    }
  }

  /** Factory method to create a TwoJointedArmPlant. */
  public static TwoJointedArmPlant createTwoJointedArmPlant(
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
    return new TwoJointedArmPlant(
        motor, m1, l1, r1, I1, m2, l2, r2, I2, G1, G2, numMotors1, numMotors2, 9.80665);
  }

  // =================================================================================
  //
  // Augmentation Helpers (Private and Overloaded for type safety)
  //
  // =================================================================================

  /** Augments a 1-state, 1-input system. */
  private static LinearSystem<N2, N1, N1> augmentWithInputError_one_one_one(
      LinearSystem<N1, N1, N1> system) {
    Matrix<N2, N2> a = new Matrix<>(Nat.N2(), Nat.N2());
    a.set(0, 0, system.getA(0, 0));
    a.set(0, 1, system.getB(0, 0));

    Matrix<N2, N1> b = new Matrix<>(Nat.N2(), Nat.N1());
    b.set(0, 0, system.getB(0, 0));

    Matrix<N1, N2> c = new Matrix<>(Nat.N1(), Nat.N2());
    c.set(0, 0, system.getC(0, 0));

    return new LinearSystem<>(a, b, c, system.getD());
  }

  /** Augments a 2-state, 1-input system. */
  private static LinearSystem<N3, N1, N2> augmentWithInputError_two_one_two(
      LinearSystem<N2, N1, N2> system) {
    Matrix<N3, N3> a = new Matrix<>(Nat.N3(), Nat.N3());
    a.assignBlock(0, 0, system.getA());
    a.assignBlock(0, 2, system.getB());

    Matrix<N3, N1> b = new Matrix<>(Nat.N3(), Nat.N1());
    b.assignBlock(0, 0, system.getB());

    Matrix<N2, N3> c = new Matrix<>(Nat.N2(), Nat.N3());
    c.assignBlock(0, 0, system.getC());

    return new LinearSystem<>(a, b, c, system.getD());
  }

  /** Augments a 3-state, 2-input system. */
  private static LinearSystem<N5, N2, N3> augmentWithInputError_three_two_three(
      LinearSystem<N3, N2, N3> system) {
    Matrix<N5, N5> a = new Matrix<>(Nat.N5(), Nat.N5());
    a.assignBlock(0, 0, system.getA());
    a.assignBlock(0, 3, system.getB());

    Matrix<N5, N2> b = new Matrix<>(Nat.N5(), Nat.N2());
    b.assignBlock(0, 0, system.getB());

    Matrix<N3, N5> c = new Matrix<>(Nat.N3(), Nat.N5());
    c.assignBlock(0, 0, system.getC());

    return new LinearSystem<>(a, b, c, system.getD());
  }
}
