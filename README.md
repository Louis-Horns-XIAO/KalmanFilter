# KalmanFilter
small practice about Kalman Filter, use Eigen.   

Assuming that :  
 + GPS measurement data [X, Y, Z] of a time series,  obtained.  
 + velocity measurement data (from inertial measurement units or else ),    obtained and transformed to **coordinate difference** between adjecent time points [X, Y, Z].   
 (looks like  position of a time series as well)
   
 The state prediction function  
 ```math
  \begin{bmatrix} \check X  \\ \check  Y\\ \check Z \end{bmatrix}_k = \begin{bmatrix} X \\Y\\Z \end{bmatrix}_{k-1}+\begin{bmatrix} v_X\\v_Y\\v_Z \end{bmatrix}_{k}+w_k  
  ```
					
The Covariance Matrix of predict state:
```math
\check P_k  = AP_{k-1}A^T +R = P+R
```
The Measurement function
```math
  \begin{bmatrix} X  \\   Y\\ Z \end{bmatrix}_{obs} = \begin{bmatrix} X \\Y\\Z \end{bmatrix}_{k}
```
It is a simple mode but shows how Kalman Filter works. Simply choose a wrong init Position, or add a random error in velocity-- the result express that as long as you set the proper Covariance Matrix (means the noise of sensor meets normal distribution and you know the covariance of all parameter,or simply you know the quality of the measurement data and state prediction )

Nothing else needs to mention now, the code and its details can be easily understood by the reference of Theory: Kalman Filter
