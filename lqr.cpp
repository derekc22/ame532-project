#include <algorithm>
#include "simulation/control/control.hpp"
#include "simulation/control/lqr.hpp"
#include "simulation/control/care.hpp"
#include "simulation/trim/trim.hpp"

namespace control {

    LinearQuadraticController::LinearQuadraticController(const ControlLawParameters& params) :
        Q(params.gains.Q),
        R(params.gains.R)
    {}

    LinearQuadraticRegulator::LinearQuadraticRegulator(const ControlLawParameters& params) : LinearQuadraticController(params) {};

    LinearQuadraticTracker::LinearQuadraticTracker(const ControlLawParameters& params) : LinearQuadraticController(params) {};


    Eigen::VectorXd LinearQuadraticController::_step(const FullStateControlLawInput& ctrl_law_input) {

        if (!K.has_value()){
            const control::CareSolution care_sol = control::solve_care(ctrl_law_input.A, ctrl_law_input.B, Q, R);
            K = control::lqr_gain(ctrl_law_input.B, R, care_sol.P);
        }

        trim::TrimStateVector_T<double> meas_deviation = ctrl_law_input.meas - ctrl_law_input.meas_des;
        Eigen::VectorXd u_deviation = -K.value() * meas_deviation;

        return u_deviation;
    }

}
