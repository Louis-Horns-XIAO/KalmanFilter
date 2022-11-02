#include <iostream>
#include <thread>
#include <chrono>
#include <string>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <random>

using namespace std;

struct Point
{
public:
    double x, y, z; // m
};

struct LabelPoint : public Point
{
public:
    int label;
};
struct Velocity : public Point
{
public:
    double period; // s
};

class KalmanFilter
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    // x is a array
    void dataImport(vector<Point> measurements_, vector<Velocity> motions_, vector<Point> Positions_);
    KalmanFilter();
    void CalOneStep();
    void CalAllStep();
    //重构思路：不存储非中间值的矩阵，简化private成员
private:
    int num_measure, num_motion;
    double periodTime;
    vector<Point> measurements;
    vector<Point> Position;
    vector<Velocity> motions;

    Eigen::MatrixXd Q, R;                             // Cov Mat
    Eigen::MatrixXd P_previous, P_prior, P_posterior; // Previous known
    Eigen::MatrixXd K;                                // Kalamn Param
    Eigen::MatrixXd A_pred, C_obs;
    // Input
    Eigen::MatrixXd x_previous, x_prior, x_posterior;
    Eigen::MatrixXd z_measurements, u_motions; // points velocity
    // Eigen::MatrixXd init_state;//Points //没有必要，直接将位置存在Position里面，生成一个estimation就存进去，之后访问最新位置即可。

    void Prediction(Point position_now, Velocity v_now);
    void Correction(Point position_measured);
};

KalmanFilter::KalmanFilter() : periodTime(1)
{
    P_previous.resize(3, 3);
    P_previous = Eigen::MatrixXd::Identity(3, 3);

    A_pred.resize(3, 3);
    R.resize(3, 3);
    R = Eigen::MatrixXd::Identity(3, 3);

    C_obs.resize(3, 3);
    Q.resize(3, 3);
    Q = Eigen::MatrixXd::Identity(3, 3);
}

void KalmanFilter::dataImport(vector<Point> measurements_, vector<Velocity> motions_, vector<Point> Positions_)
{
    measurements = measurements_;
    motions = motions_;
    Position = Positions_;
}
void KalmanFilter::CalOneStep()
{
    Prediction(Position[0], motions[0]);
    Correction(measurements[0]);
}

void KalmanFilter::CalAllStep()
{
    if (measurements.size() != motions.size())
    {
        cout << "Wrong Match: measurement velocity" << endl;
        return;
    }
    if(Position.size()> 1){
        cout<<"Position has been calculated at least once!"<<endl;
        return;
    }

    for (int i = 0; i < measurements.size(); i++)
    {
        if (i < Position.size())
        {
            cout<<"Step "<<i<<endl;
            Prediction(Position[i], motions[i]);
            Correction(measurements[i]);
            Point resultP;
            resultP.x = x_posterior(0,0);
            resultP.y = x_posterior(1,0);
            resultP.z = x_posterior(2,0);
            Position.push_back(resultP);
        }
    }
}

void KalmanFilter::Prediction(Point position_now, Velocity v_now_data)
{
    // to Predict
    Eigen::MatrixXd x_predicts, P_predicts_;
    Eigen::MatrixXd x_present, v_now;
    x_present.resize(3, 1);
    v_now.resize(3, 1);
    // data input
    x_present << position_now.x, position_now.y, position_now.z;
    v_now << v_now_data.x, v_now_data.y, v_now_data.z;
    // State Predict
    A_pred = Eigen::MatrixXd::Identity(3, 3);

    x_predicts = A_pred * x_present + v_now * periodTime;
    // Covariance Predict
    P_predicts_ = A_pred * P_previous * A_pred.transpose() + R;
    // output
    this->x_prior = x_predicts;
    this->P_prior = P_predicts_;
    return;
}

void KalmanFilter::Correction(Point position_measured)
{
    Eigen::MatrixXd K, x_estimate, P_estimate;
    Eigen::MatrixXd x_measure(3, 1);
    Eigen::MatrixXd I(3, 3);
    I = Eigen::MatrixXd::Identity(3, 3);

    x_measure << position_measured.x, position_measured.y, position_measured.z;
    C_obs = Eigen::MatrixXd::Identity(3, 3);

    // Cal K
    K = C_obs * P_prior * C_obs.transpose() + Q;
    K = P_prior * C_obs.transpose() * K.inverse();
    // cout<<K<<endl;
    // Correct
    x_estimate = x_prior + K * (x_measure - C_obs * x_prior);
    // cout<<x_prior<<endl;
    // cout<<(x_measure-C_obs*x_prior)<<endl;
    P_estimate = (I - K * C_obs) * P_prior;
    // output
    this->x_posterior = x_estimate;
    this->P_posterior = P_estimate;
    cout << "Position"<<"\n" << x_posterior.transpose() << endl;
    cout << "K:"<<"\n"<< K.transpose()<<"\n" <<endl;
    return;
}

int main(int argc, char **argv)
{
    default_random_engine rand_engine;
    normal_distribution<double> random_noise_v(0,0.5);

    vector<Point> meas, Pos;
    vector<Velocity> v_meas;
    for (int i = 0; i < 30; i++)
    {
        Point temp;
        Velocity v_temp;
        temp.x = i;
        temp.y = i + 1;
        temp.z = 2 * i + 2;
        v_temp.x = 1+random_noise_v(rand_engine);
        v_temp.y = 1+random_noise_v(rand_engine);
        v_temp.z = 2+random_noise_v(rand_engine), v_temp.period = 1;
        meas.push_back(temp);
        v_meas.push_back(v_temp);
    }
    Point init_state;
    init_state.x = -100;
    init_state.y = 0;
    init_state.z = 0;
    Pos.push_back(init_state);

    KalmanFilter testKF;
    testKF.dataImport(meas, v_meas, Pos);
    //testKF.CalOneStep();
    testKF.CalAllStep();
    
    return 0;
}
//写一个赋值
