package frc.robot.slsh.math.filter;

import frc.robot.slsh.math.system.LinearSystem_NoGC;
import frc.robot.slsh.util.StateSpaceUtil_NoGC;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.interfaces.linsol.LinearSolverDense;
import org.ejml.simple.SimpleMatrix;

/**
 * 卡爾曼濾波器（Kalman Filter）的無記憶體分配版本。
 *
 * <p>用於從帶有雜訊的感測器讀數中估計系統的真實狀態。
 *
 * <p>這個類別的建構子和所有即時方法（predict, correct）都以無記憶體分配的方式執行。
 */
public class KalmanFilter_NoGC {
  private final LinearSystem_NoGC m_system;
  private final SimpleMatrix m_Q; // 過程雜訊協方差
  private final SimpleMatrix m_R; // 量測雜訊協方差

  // 狀態估計向量 x̂ (x-hat)
  private final SimpleMatrix m_xhat;
  // 誤差協方差矩陣 P
  private final SimpleMatrix m_P;

  // --- 預先分配的暫存矩陣 ---
  // Discretization 步驟
  private final SimpleMatrix m_discA;
  private final SimpleMatrix m_discB;
  private final SimpleMatrix m_M_temp;
  private final SimpleMatrix m_phi_temp;
  private double m_lastDt = -1.0;

  // Predict 步驟
  private final SimpleMatrix m_temp_predict_x1;
  private final SimpleMatrix m_temp_predict_x2;
  private final SimpleMatrix m_temp_predict_p1;

  // Correct 步驟
  private final SimpleMatrix m_C;
  private final SimpleMatrix m_temp_correct_p1;
  private final SimpleMatrix m_temp_correct_p2;
  private final SimpleMatrix m_S; // (C * P * Cᵀ + R)
  private final SimpleMatrix m_S_inv;
  private final SimpleMatrix m_K; // 卡爾曼增益
  private final SimpleMatrix m_temp_correct_y;
  private final SimpleMatrix m_I; // 單位矩陣
  private final SimpleMatrix m_P_backup; // 用於 P = (I-KC)P' 計算的暫存

  private final LinearSolverDense<DMatrixRMaj> m_solver;

  public KalmanFilter_NoGC(LinearSystem_NoGC system, SimpleMatrix Q, SimpleMatrix R) {
    this.m_system = system;
    this.m_Q = Q;
    this.m_R = R;

    int states = system.getNumStates();
    int outputs = system.getNumOutputs();
    int M_size = states + system.getNumInputs();

    m_xhat = new SimpleMatrix(states, 1);
    m_P = SimpleMatrix.identity(states);
    m_C = system.getC();

    // Discretization
    m_discA = new SimpleMatrix(states, states);
    m_discB = new SimpleMatrix(states, system.getNumInputs());
    m_M_temp = new SimpleMatrix(M_size, M_size);
    m_phi_temp = new SimpleMatrix(M_size, M_size);

    // Predict
    m_temp_predict_x1 = new SimpleMatrix(states, 1);
    m_temp_predict_x2 = new SimpleMatrix(states, 1);
    m_temp_predict_p1 = new SimpleMatrix(states, states);

    // Correct
    m_temp_correct_p1 = new SimpleMatrix(states, outputs);
    m_temp_correct_p2 = new SimpleMatrix(outputs, states);
    m_S = new SimpleMatrix(outputs, outputs);
    m_S_inv = new SimpleMatrix(outputs, outputs);
    m_K = new SimpleMatrix(states, outputs);
    m_temp_correct_y = new SimpleMatrix(outputs, 1);
    m_I = SimpleMatrix.identity(states);
    m_P_backup = new SimpleMatrix(states, states);

    m_solver = org.ejml.dense.row.factory.LinearSolverFactory_DDRM.pseudoInverse(true);
  }

  public void reset() {
    m_xhat.zero();
    CommonOps_DDRM.setIdentity(m_P.getDDRM());
  }

  /** 預測模型的下一個狀態。 x̂' = A * x̂ + B * u P' = A * P * Aᵀ + Q */
  public void predict(SimpleMatrix u, double dtSeconds) {
    // 只有在 dt 改變時才重新進行離散化
    if (dtSeconds != m_lastDt) {
      StateSpaceUtil_NoGC.discretizeAB_NoGC(
          m_system.getA(), m_system.getB(), dtSeconds, m_discA, m_discB, m_M_temp, m_phi_temp);
      m_lastDt = dtSeconds;
    }

    DMatrixRMaj A = m_discA.getDDRM();
    DMatrixRMaj B = m_discB.getDDRM();
    DMatrixRMaj xhat = m_xhat.getDDRM();
    DMatrixRMaj u_ddrm = u.getDDRM();
    DMatrixRMaj P = m_P.getDDRM();
    DMatrixRMaj Q = m_Q.getDDRM();

    // -- 更新 x̂ --
    // temp1 = A * x̂
    CommonOps_DDRM.mult(A, xhat, m_temp_predict_x1.getDDRM());
    // temp2 = B * u
    CommonOps_DDRM.mult(B, u_ddrm, m_temp_predict_x2.getDDRM());
    // x̂ = temp1 + temp2
    CommonOps_DDRM.add(m_temp_predict_x1.getDDRM(), m_temp_predict_x2.getDDRM(), xhat);

    // -- 更新 P --
    // temp_p1 = A * P
    CommonOps_DDRM.mult(A, P, m_temp_predict_p1.getDDRM());

    // P = temp_p1 * Aᵀ + Q (無垃圾版本)
    // 1. P = Q (複製內容)
    System.arraycopy(Q.getData(), 0, P.getData(), 0, Q.getNumElements());
    // 2. P = P + (A*P)*Aᵀ
    CommonOps_DDRM.multAddTransB(m_temp_predict_p1.getDDRM(), A, P);
  }

  /** 使用感測器量測值來修正狀態估計。 K = P'Cᵀ(CP'Cᵀ + R)⁻¹ x̂ = x̂' + K(y − Cx̂') P = (I − KC)P' */
  public void correct(SimpleMatrix u, SimpleMatrix y) {
    DMatrixRMaj C = m_C.getDDRM();
    DMatrixRMaj P = m_P.getDDRM();
    DMatrixRMaj R = m_R.getDDRM();
    DMatrixRMaj xhat = m_xhat.getDDRM();
    DMatrixRMaj y_ddrm = y.getDDRM();

    DMatrixRMaj K = m_K.getDDRM();
    DMatrixRMaj S = m_S.getDDRM();
    DMatrixRMaj S_inv = m_S_inv.getDDRM();
    DMatrixRMaj temp_p1 = m_temp_correct_p1.getDDRM();
    DMatrixRMaj temp_p2 = m_temp_correct_p2.getDDRM();
    DMatrixRMaj temp_y = m_temp_correct_y.getDDRM();

    // -- 計算卡爾曼增益 K --
    // temp_p1 = P * Cᵀ
    CommonOps_DDRM.multTransB(P, C, temp_p1);
    // temp_p2 = C * P
    CommonOps_DDRM.mult(C, P, temp_p2);
    // S = temp_p2 * Cᵀ + R (無垃圾版本)
    // 1. S = R (複製內容)
    System.arraycopy(R.getData(), 0, S.getData(), 0, R.getNumElements());
    // 2. S = S + (C*P)*Cᵀ
    CommonOps_DDRM.multAddTransB(temp_p2, C, S);

    // S_inv = S⁻¹
    m_solver.setA(S);
    m_solver.invert(S_inv);

    // K = temp_p1 * S_inv
    CommonOps_DDRM.mult(temp_p1, S_inv, K);

    // -- 使用 K 更新 x̂ --
    // temp_y = y - C * x̂
    CommonOps_DDRM.mult(C, xhat, temp_y);
    CommonOps_DDRM.subtract(y_ddrm, temp_y, temp_y);
    // x̂ = x̂ + K * temp_y
    CommonOps_DDRM.multAdd(K, temp_y, xhat);

    // -- 使用 K 更新 P --
    // P = (I − KC)P'
    // temp_p1 (re-used) = K * C
    CommonOps_DDRM.mult(K, C, m_temp_predict_p1.getDDRM()); // 複用 predict 的暫存
    // temp_p1 = I - temp_p1
    CommonOps_DDRM.subtract(
        m_I.getDDRM(), m_temp_predict_p1.getDDRM(), m_temp_predict_p1.getDDRM());

    // P = temp_p1 * P' (需要另一個暫存來避免原地修改)
    System.arraycopy(P.getData(), 0, m_P_backup.getDDRM().getData(), 0, P.getNumElements());
    CommonOps_DDRM.mult(m_temp_predict_p1.getDDRM(), m_P_backup.getDDRM(), P);
  }

  public SimpleMatrix getXhat() {
    return m_xhat;
  }

  public double getXhat(int row) {
    return m_xhat.get(row, 0);
  }

  public void setXhat(SimpleMatrix xhat) {
    System.arraycopy(
        xhat.getDDRM().getData(), 0, this.m_xhat.getDDRM().getData(), 0, xhat.getNumElements());
  }

  public void setXhat(int row, double val) {
    this.m_xhat.set(row, 0, val);
  }
}
