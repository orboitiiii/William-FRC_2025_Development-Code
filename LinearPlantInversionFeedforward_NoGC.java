package frc.robot.slsh.math.controller;

import frc.robot.slsh.math.system.LinearSystem_NoGC;
import frc.robot.slsh.util.StateSpaceUtil_NoGC;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.interfaces.linsol.LinearSolverDense;
import org.ejml.simple.SimpleMatrix;

/**
 * 工廠反演前饋（Plant-inversion feedforward）的無記憶體分配版本。
 *
 * <p>前饋定律為 u_ff = B⁺ (r_k+1 - A * r_k)，其中 B⁺ 是 B 的偽逆。
 *
 * <p>這個類別的建構子和 `calculate` 方法都以無記憶體分配的方式執行。
 */
public class LinearPlantInversionFeedforward_NoGC {
  // 離散化後的 A, B 矩陣
  private final SimpleMatrix m_A;
  private final SimpleMatrix m_B;

  // 參考狀態向量
  private SimpleMatrix m_r;
  // 前饋輸出向量
  private final SimpleMatrix m_uff;

  // 預先分配的暫存矩陣
  private final SimpleMatrix m_temp_state_term;

  // 預先分配的線性解算器
  private final LinearSolverDense<DMatrixRMaj> m_solver;

  /**
   * 建構一個前饋控制器。
   *
   * @param system 要控制的線性系統
   * @param dtSeconds 離散化時間步長
   */
  public LinearPlantInversionFeedforward_NoGC(LinearSystem_NoGC system, double dtSeconds) {
    int states = system.getNumStates();
    int inputs = system.getNumInputs();

    // --- 為初始化和控制迴圈預先分配所有需要的矩陣 ---

    // 初始化所需
    m_A = new SimpleMatrix(states, states);
    m_B = new SimpleMatrix(states, inputs);
    SimpleMatrix M_temp = new SimpleMatrix(states + inputs, states + inputs);
    SimpleMatrix phi_temp = new SimpleMatrix(states + inputs, states + inputs);

    // 控制迴圈所需
    m_r = new SimpleMatrix(states, 1);
    m_uff = new SimpleMatrix(inputs, 1);
    m_temp_state_term = new SimpleMatrix(states, 1);

    // --- 執行無垃圾初始化 ---
    // 1. 離散化 A 和 B
    StateSpaceUtil_NoGC.discretizeAB_NoGC(
        system.getA(),
        system.getB(),
        dtSeconds,
        m_A,
        m_B, // 輸出結果
        M_temp,
        phi_temp // 暫存
        );

    // --- 初始化其他元件 ---
    // 預先配置線性解算器以求解 u_ff
    m_solver = LinearSolverFactory_DDRM.pseudoInverse(true);

    reset();
  }

  /** 重置參考和輸出。 */
  public void reset() {
    this.m_r.zero();
    this.m_uff.zero();
  }

  public void reset(SimpleMatrix initialState) {
    System.arraycopy(
        initialState.getDDRM().getData(),
        0,
        this.m_r.getDDRM().getData(),
        0,
        initialState.getNumElements());
    this.m_uff.zero();
  }

  /**
   * 以無記憶體分配的方式計算前饋輸出。 u_ff = B⁺(r_k+1 − Ar_k)
   *
   * @param r 當前參考狀態 r_k
   * @param nextR 下一個參考狀態 r_k+1
   */
  public void calculate(SimpleMatrix r, SimpleMatrix nextR) {
    // 取得底層 DMatrixRMaj
    DMatrixRMaj A_ddrm = m_A.getDDRM();
    DMatrixRMaj r_ddrm = r.getDDRM();
    DMatrixRMaj nextR_ddrm = nextR.getDDRM();
    DMatrixRMaj temp_ddrm = m_temp_state_term.getDDRM();
    DMatrixRMaj uff_ddrm = m_uff.getDDRM();
    DMatrixRMaj B_ddrm = m_B.getDDRM();

    // 1. 計算 A * r_k，結果存入 temp
    CommonOps_DDRM.mult(A_ddrm, r_ddrm, temp_ddrm);

    // 2. 計算 r_k+1 - (A * r_k)，結果存回 temp
    CommonOps_DDRM.subtract(nextR_ddrm, temp_ddrm, temp_ddrm);

    // 3. 求解 u_ff： B * u_ff = temp
    if (!m_solver.setA(B_ddrm)) {
      throw new RuntimeException("無法設定解算器的 A 矩陣");
    }
    m_solver.solve(temp_ddrm, uff_ddrm);

    // 更新當前參考
    System.arraycopy(
        nextR.getDDRM().getData(), 0, this.m_r.getDDRM().getData(), 0, nextR.getNumElements());
  }

  public SimpleMatrix getUff() {
    return m_uff;
  }

  public SimpleMatrix getR() {
    return m_r;
  }
}
