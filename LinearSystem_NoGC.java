package frc.robot.slsh.math.system;

import org.ejml.simple.SimpleMatrix;

/**
 * 代表一個線性時不變（LTI）系統的無記憶體分配版本。
 *
 * <p>這個類別直接使用 EJML 的 SimpleMatrix，並且不包含泛型，以實現最高效能。 它僅作為一個資料容器，持有系統的 A, B, C, D 矩陣。
 *
 * @param states 狀態數量
 * @param inputs 輸入數量
 * @param outputs 輸出數量
 */
public class LinearSystem_NoGC {
  private final SimpleMatrix m_A;
  private final SimpleMatrix m_B;
  private final SimpleMatrix m_C;
  private final SimpleMatrix m_D;

  private final int m_states;
  private final int m_inputs;
  private final int m_outputs;

  /**
   * 建構一個新的 LinearSystem_NoGC。
   *
   * @param A 系統矩陣 A
   * @param B 輸入矩陣 B
   * @param C 輸出矩陣 C
   * @param D 前饋矩陣 D
   */
  public LinearSystem_NoGC(SimpleMatrix A, SimpleMatrix B, SimpleMatrix C, SimpleMatrix D) {
    this.m_A = A;
    this.m_B = B;
    this.m_C = C;
    this.m_D = D;

    this.m_states = A.getNumRows();
    this.m_inputs = B.getNumCols();
    this.m_outputs = C.getNumRows();

    // 驗證矩陣維度
    if (A.getNumRows() != A.getNumCols()
        || A.getNumRows() != B.getNumRows()
        || A.getNumRows() != C.getNumCols()
        || B.getNumCols() != D.getNumCols()
        || C.getNumRows() != D.getNumRows()) {
      throw new IllegalArgumentException("矩陣維度不匹配！");
    }
  }

  public SimpleMatrix getA() {
    return m_A;
  }

  public SimpleMatrix getB() {
    return m_B;
  }

  public SimpleMatrix getC() {
    return m_C;
  }

  public SimpleMatrix getD() {
    return m_D;
  }

  public int getNumStates() {
    return m_states;
  }

  public int getNumInputs() {
    return m_inputs;
  }

  public int getNumOutputs() {
    return m_outputs;
  }
}
