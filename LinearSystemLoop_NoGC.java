package frc.robot.slsh.math.system;

import frc.robot.slsh.math.controller.LinearPlantInversionFeedforward_NoGC;
import frc.robot.slsh.math.controller.LinearQuadraticRegulator_NoGC;
import frc.robot.slsh.math.filter.KalmanFilter_NoGC;
import java.util.function.Consumer;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.simple.SimpleMatrix;

/**
 * 整合 LQR、前饋和卡爾曼濾波器的完整狀態空間控制迴圈的無記憶體分配版本。
 *
 * <p>這是在機器人程式碼中直接使用的最上層物件。
 */
public class LinearSystemLoop_NoGC {
  private final LinearQuadraticRegulator_NoGC m_controller;
  private final LinearPlantInversionFeedforward_NoGC m_feedforward;
  private final KalmanFilter_NoGC m_observer;

  // 目標參考狀態向量
  private final SimpleMatrix m_nextR;
  // 最終輸出到馬達的電壓向量 u
  private final SimpleMatrix m_u;

  // 用於限制電壓輸出的函式
  private final Consumer<SimpleMatrix> m_clampFunction;

  /**
   * 建構一個狀態空間迴圈。
   *
   * @param controller LQR 控制器 (NoGC version)
   * @param feedforward 前饋控制器 (NoGC version)
   * @param observer 卡爾曼濾波器 (NoGC version)
   * @param clampFunction 一個 Consumer<SimpleMatrix>，用於原地限制電壓大小
   */
  public LinearSystemLoop_NoGC(
      LinearQuadraticRegulator_NoGC controller,
      LinearPlantInversionFeedforward_NoGC feedforward,
      KalmanFilter_NoGC observer,
      Consumer<SimpleMatrix> clampFunction) {
    this.m_controller = controller;
    this.m_feedforward = feedforward;
    this.m_observer = observer;
    this.m_clampFunction = clampFunction;

    int numInputs = feedforward.getUff().getNumRows();
    int numStates = feedforward.getR().getNumRows();

    m_nextR = new SimpleMatrix(numStates, 1);
    m_u = new SimpleMatrix(numInputs, 1);
  }

  public SimpleMatrix getXHat() {
    return m_observer.getXhat();
  }

  public double getXHat(int row) {
    return m_observer.getXhat(row);
  }

  public SimpleMatrix getNextR() {
    return m_nextR;
  }

  public void setNextR(SimpleMatrix nextR) {
    // 修正：使用 System.arraycopy 進行無垃圾矩陣複製
    System.arraycopy(
        nextR.getDDRM().getData(), 0, this.m_nextR.getDDRM().getData(), 0, nextR.getNumElements());
  }

  public SimpleMatrix getU() {
    return m_u;
  }

  public void reset(SimpleMatrix initialState) {
    m_nextR.zero();
    m_controller.reset();
    m_feedforward.reset(initialState);
    // 修正：使用正確的 setXhat 方法進行無垃圾狀態設定
    m_observer.setXhat(initialState);
  }

  /**
   * 使用量測值 y 來修正狀態估計。
   *
   * @param y 感測器量測向量
   */
  public void correct(SimpleMatrix y) {
    m_observer.correct(m_u, y);
  }

  /**
   * 預測下一個狀態並計算下一個控制輸出。
   *
   * @param dtSeconds 時間步長
   */
  public void predict(double dtSeconds) {
    // 1. 設定 LQR 的目標參考
    m_controller.setR(m_nextR);

    // 2. 計算 LQR 控制輸出
    m_controller.calculate(m_observer.getXhat());

    // 3. 計算前饋控制輸出
    m_feedforward.calculate(m_feedforward.getR(), m_nextR);

    // 4. 合併 LQR 和前饋輸出
    // u = u_lqr + u_ff
    // 修正：使用 System.arraycopy 先複製，再用 CommonOps_DDRM.add 進行無垃圾加法
    System.arraycopy(
        m_controller.getU().getDDRM().getData(),
        0,
        this.m_u.getDDRM().getData(),
        0,
        m_controller.getU().getNumElements());

    CommonOps_DDRM.add(m_u.getDDRM(), m_feedforward.getUff().getDDRM(), m_u.getDDRM());

    // 5. 限制電壓輸出
    m_clampFunction.accept(m_u);

    // 6. 使用最終的（被限制後的）電壓來預測觀測器的下一個狀態
    m_observer.predict(m_u, dtSeconds);
  }

  /**
   * 一個範例電壓限制函式，可以原地修改矩陣。
   *
   * @param maxVoltage 最大電壓
   * @return 一個 Consumer，可傳入 LinearSystemLoop_NoGC 的建構子
   */
  public static Consumer<SimpleMatrix> createVoltageClamp(double maxVoltage) {
    return u -> {
      for (int i = 0; i < u.getNumElements(); i++) {
        double val = u.get(i);
        if (val > maxVoltage) {
          u.set(i, 0, maxVoltage);
        } else if (val < -maxVoltage) {
          u.set(i, 0, -maxVoltage);
        }
      }
    };
  }
}
