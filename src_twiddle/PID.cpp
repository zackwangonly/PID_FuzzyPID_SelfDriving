#include "PID.h"
#include <cmath>
#include <iostream>
#include <limits>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

/*
* Initialize PID.
*/
void PID::Init(double Kp, double Ki, double Kd, double Max_Output, Tunings Tuning_type) {
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
    i_error_ = 0.0;
    i_output_ = 0.0;

    max_i_output_ = 0.8 * Max_Output;

    // initially set to false, set to true in first call of ProcessMeasurement
    is_initialized_ = false;

    // Twiddling parameters
    flag_tuning = Tuning_type;
    dp = {0.5*Kp,0.5*Kd,0.5*Ki};
    step = 1;
    param_index = 2;  // this will wrao back to 0 after the first twiddle loop
    n_settle_steps = 100;
    n_eval_steps = 200;
    total_error = 0;
    best_error = std::numeric_limits<double>::max();
    tried_adding = false;
    tried_subtracting = false;

    // Adaptive PID
    flag_adaptive = true;
    adap_p_gain = 1.0e-6;
    adap_i_gain = 1.0e-8;
    adap_d_gain = 1.0e-3;
    adap_sliding_gain = 30;
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
        i_error_ = -cte;

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
    i_error_ = i_error_ + (-cte);

    i_output_ += -cte * Ki_;

    if(i_output_ > max_i_output_){
        i_output_ = max_i_output_;
    }else if(i_output_ < -max_i_output_){
        i_output_ = -max_i_output_;
    }

    /*****************************************************************************
    *  Twiddle Tuning
    ****************************************************************************/
    // update total error only if we're past number of settle steps
    if (step % (n_settle_steps + n_eval_steps) > n_settle_steps){
        total_error += pow(cte,2);
    }

    // last step in twiddle loop... twiddle it?
    if (flag_tuning == Twiddle && step % (n_settle_steps + n_eval_steps) == 0){
        cout << "step: " << step << endl;
        cout << "total error: " << total_error << endl;
        cout << "best error: " << best_error << endl;
        if (total_error < best_error) {
            cout << "improvement!" << endl;
            best_error = total_error;
            if (step !=  n_settle_steps + n_eval_steps) {
                // don't do this if it's the first time through
                dp[param_index] *= 1.05;
            }
            // next parameter
            param_index = (param_index + 1) % 3;
            tried_adding = tried_subtracting = false;
        }
        if (!tried_adding && !tried_subtracting) {
            // try adding dp[i] to params[i]
            AddToParameterAtIndex(param_index, dp[param_index]);
            tried_adding = true;
        }
        else if (tried_adding && !tried_subtracting) {
            // try subtracting dp[i] from params[i]
            AddToParameterAtIndex(param_index, -2 * dp[param_index]);
            tried_subtracting = true;
        }
        else {
            // set it back, reduce dp[i], move on to next parameter
            AddToParameterAtIndex(param_index, dp[param_index]);
            dp[param_index] *= 0.95;
            // next parameter
            param_index = (param_index + 1) % 3;
            tried_adding = tried_subtracting = false;
        }
        total_error = 0;
        cout << "new parameters" << endl;
        cout << "P: " << Kp_ << ", I: " << Ki_ << ", D: " << Kd_ << endl;
    }
    step++;

    /*****************************************************************************
    *  Adaptive Tuning
    ****************************************************************************/
    if(flag_tuning == Adaptive){
        double z = -(p_error_ * adap_sliding_gain + d_error_);
        Kp_ = Kp_ - adap_p_gain * p_error_ * z;
        Ki_ = Ki_ - adap_i_gain * i_error_ * z;
        Kd_ = Kd_ - adap_d_gain * d_error_ * z;
        cout << "new parameters for adaptive PID" << endl;
        cout << "P: " << Kp_ << ", I: " << Ki_ << ", D: " << Kd_ << endl;
    }
}

/*
 * Calculate the total PID error.
 */
double PID::TotalError() {
    //return Kp_ * p_error_ + i_output_ + Kd_ * d_error_;
    return Kp_ * p_error_ + Ki_ * i_error_ + Kd_ * d_error_;
}

void PID::AddToParameterAtIndex(int index, double amount) {
    if (index == 0) {
        Kp_ += amount;
    }
    else if (index == 1) {
        Kd_ += amount;
    }
    else if (index == 2) {
        Ki_ += amount;
    }
    else {
        std::cout << "AddToParameterAtIndex: index out of bounds";
    }
}

