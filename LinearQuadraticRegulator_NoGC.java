package frc.robot.slsh.math.controller;

import frc.robot.slsh.math.system.LinearSystem_NoGC;
import frc.robot.slsh.util.StateSpaceUtil_NoGC;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.simple.SimpleMatrix;

/**
 * 線性二次調節器（LQR）的無記憶體分配版本。
 *
 * <p>控制器定律為 u = K(r - x)。
 *
 * <p>這個類別在建構時計算一次性的控制器增益 K，並在 `calculate` 方法中以無記憶體分配的方式執行運算。
 */
public class LinearQuadraticRegulator_NoGC {
  // 控制器增益矩陣
  private final SimpleMatrix m_K;
  // 目標參考狀態向量 r
  private final SimpleMatrix m_r;
  // 控制器輸出向量 u
  private final SimpleMatrix m_u;

  // 預先分配的暫存矩陣，用於儲存計算過程中的誤差 (r - x)
  private final SimpleMatrix m_error;

  /**
   * 建構一個 LQR 控制器。
   *
   * @param system 要控制的線性系統
   * @param Q 狀態成本矩陣
   * @param R 輸入成本矩陣
   * @param dtSeconds 離散化時間步長
   */
  public LinearQuadraticRegulator_NoGC(
      LinearSystem_NoGC system, SimpleMatrix Q, SimpleMatrix R, double dtSeconds) {

    int states = system.getNumStates();
    int inputs = system.getNumInputs();

    // --- 為初始化過程預先分配所有需要的矩陣 ---
    // K_out 是最終結果
    m_K = new SimpleMatrix(inputs, states);

    // 離散化所需
    SimpleMatrix discA = new SimpleMatrix(states, states);
    SimpleMatrix discB = new SimpleMatrix(states, inputs);
    SimpleMatrix M_temp = new SimpleMatrix(states + inputs, states + inputs);
    // 修正：增加 phi_temp 暫存矩陣以匹配方法簽名
    SimpleMatrix phi_temp = new SimpleMatrix(states + inputs, states + inputs);

    // K 計算所需
    SimpleMatrix temp_S = new SimpleMatrix(states, states);
    SimpleMatrix temp_S_B = new SimpleMatrix(states, inputs);
    SimpleMatrix temp_B_S_B = new SimpleMatrix(inputs, inputs);
    SimpleMatrix temp_B_S_B_plus_R = new SimpleMatrix(inputs, inputs);
    SimpleMatrix temp_B_S_A = new SimpleMatrix(inputs, states);

    // --- 執行無垃圾初始化 ---
    // 1. 離散化 A 和 B
    StateSpaceUtil_NoGC.discretizeAB_NoGC(
        system.getA(), system.getB(), dtSeconds, discA, discB, M_temp, phi_temp // 修正：傳入 phi_temp
        );

    // 2. 計算 LQR 增益 K
    StateSpaceUtil_NoGC.calculateK_NoGC(
        discA,
        discB,
        Q,
        R,
        m_K, // 輸出結果
        temp_S,
        temp_S_B,
        temp_B_S_B,
        temp_B_S_B_plus_R,
        temp_B_S_A // 暫存
        );

    // --- 為控制迴圈預先分配矩陣 ---
    m_r = new SimpleMatrix(states, 1);
    m_u = new SimpleMatrix(inputs, 1);
    m_error = new SimpleMatrix(states, 1);

    reset();
  }

  /** 重置參考和輸出。 */
  public void reset() {
    m_r.zero();
    m_u.zero();
  }

  /**
   * 以無記憶體分配的方式計算控制器輸出。 u = K * (r - x)
   *
   * @param x 當前狀態向量
   */
  public void calculate(SimpleMatrix x) {
    // 取得矩陣的底層 DMatrixRMaj 以使用 CommonOps_DDRM
    DMatrixRMaj r_ddrm = m_r.getDDRM();
    DMatrixRMaj x_ddrm = x.getDDRM();
    DMatrixRMaj error_ddrm = m_error.getDDRM();
    DMatrixRMaj K_ddrm = m_K.getDDRM();
    DMatrixRMaj u_ddrm = m_u.getDDRM();

    // 1. 計算誤差：error = r - x
    CommonOps_DDRM.subtract(r_ddrm, x_ddrm, error_ddrm);

    // 2. 計算輸出：u = K * error
    CommonOps_DDRM.mult(K_ddrm, error_ddrm, u_ddrm);
  }

  public SimpleMatrix getU() {
    return m_u;
  }

  public SimpleMatrix getR() {
    return m_r;
  }

  public void setR(SimpleMatrix r) {
    System.arraycopy(r.getDDRM().getData(), 0, this.m_r.getDDRM().getData(), 0, r.getNumElements());
  }
}
