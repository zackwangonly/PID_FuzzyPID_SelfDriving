#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

/*
* Initialize PID.
*/

void PID::Init(double Kp, double Ki, double Kd, double Max_Output) {
    //Set PID parameters to assigned values
    Kp_ = Kp;
    Ki_ = Ki;
    Kd_ = Kd;

    /*
    //Set errors to zeros
    p_error_ = 0.0;
    i_error_ = 0.0;
    d_error_ = 0.0;
     */

    //rest i_output_ to zero
    //the reason that we choose to use i_output_ not the i_error_ is:
    //it is hard for us to choose a threshold for i_error_
    //but i_output_ value can be easily limited within 0.8*max_control
    //e.g. the maximum value of steer is 1, we can set maximum value for i_output_ to 0.8
    i_output_ = 0.0;

    max_i_output_ = 0.8 * Max_Output;

    // initially set to false, set to true in first call of ProcessMeasurement
    is_initialized_ = false;
}

/*
* Update the PID error variables given cross track error.
*/
void PID::UpdateError(double cte) {

    /*****************************************************************************
    *  Initialization
     *  The reason to utilize initialization here is to avoid a first jump in d_error_,
     *  the d_error_ should be zero at the first measurement.
    ****************************************************************************/
    if (!is_initialized_) {

        // first measurement
        d_error_ = 0.0;
        p_error_ = -cte;

        i_output_ = -cte * Ki_;

        // done initializing,
        is_initialized_ = true;
        return;
    }

    /*****************************************************************************
    *  Update Error
    ****************************************************************************/
    //when p_error_ is not updated, it keeps the value of the previous error
    d_error_ = -cte - p_error_;
    p_error_ = -cte;

    i_output_ += -cte * Ki_;

    if(i_output_ > max_i_output_){
        i_output_ = max_i_output_;
    }else if(i_output_ < -max_i_output_){
        i_output_ = -max_i_output_;
    }
}

/*
 * Calculate the total PID error.
 */
double PID::TotalError() {
    return Kp_ * p_error_ + i_output_ + Kd_ * d_error_;
}
void PID::RenewParameter(double new_kp, double new_ki, double new_kd){
    Kp_ = new_kp;
    Ki_ = new_ki;
    Kd_ = new_kd;

}

