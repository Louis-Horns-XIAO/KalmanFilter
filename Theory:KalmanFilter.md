# Kalman Filter For GNSS / INS Fusion

## Basic Theory

A Classic Kalman Filter consist of two steps: Prediction stage and Correction stage.

1. Prediction Stage:

   In Prediction stage, the prediction of state parameters( time k ) are calculated by the present( time k-1 ) state parameters and the control data. Function followed are built that describe how the state evolves from time k-1 to k as well as the way control data changes the state simultaneously.
  ```math
   \check x_k = A_k\hat x_{k-1}+Bu_k \tag1
  ```

   -   $ \hat x_{k-1}$: input state, the optical estimated state of time k-1
   -   $P_{k-1}$: Covariance matrix of the input state
   -   $\check x_k$:  Predicted state.
   -   $\check P_{k} $: Covariance matrix of the predicted state.
   -   $u_k$: Control data
   -   $w_k$: Control noise

   Function 1 is deducted from the following Function,describe the true state evolution with state noise:
   ```math
   x_k = A_kx_{k-1}+u_k+w_k
   ```
   
2. Correction Stage:

   In Correction stage, K (Kalman gain) are calculate by the Covariance matrix of the predicted state and the measurement noise, based on the measurement function(followed) that describe the constrain between measured data and the true state.
  ```math
   z_k = C_kx_k+v_k
   ```
   Kalman Gain are calculated as follows:
   ```math
   K = \check P_kC_k^T(C_k\check P_kC_k^T +Q)^{-1}
   ```
   Then the estimated state of time k and the Covariance matrix of the estimated state can be calculated:
   ```math
   \hat x_k = \check x_k + K(z_k-C_k\check x_k)\\
   \hat P_k = (I-KC_k)\check P_k
   ```
   

- $ \hat x_{k}$: output state, the optical estimated state of time k,

- $\hat P_{k}$: Covariance matrix of the output state

- $\check x_k$:  Predicted state.

- $\check P_{k} $: Covariance matrix of the predicted state.

- $z_k$: measurement data

- $K$: Kalman gain

- $v_k$: measurement noise

  It is obvious that Kalman gain can decide whether the predict state contributes more to the estimated state than the measurement data or not. the bias between measurement data and estimated measurement data from  predict state transform to an adjustment to $\check x_k$ .

In general, the sensitivity of the Kalman filter to the error of initial value is actually good, and in the case of correctly determining the covariance matrix, the entire algorithm can finally converge to a higher precision position.
总的来说，觉得卡尔曼滤波对初值的敏感程度其实还好，在正确判定协方差矩阵的情况下，整个算法最终能收敛到较高精度的位置。	

If a more outrageous initial value is selected, although the Kalman gain does not change, the final bias will gradually converge as the difference between the predicted value and the true value narrows. So Q and R should at least match the actual situation.
若是选取较为离谱的初值，虽然卡尔曼增益不会变化，但最终bias会随着预测值与真实值差别的缩小而逐渐收敛。因此Q和R应该至少与实际情况匹配。

After adding random noise to the velocity V, it has little effect on the convergence of the result, and the effect on the convergence speed is actually not larg.
在对速度V添加随机噪声后，对结果的收敛影响不大，对收敛速度的影响其实也不大。
