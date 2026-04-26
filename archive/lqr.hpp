#pragma once
#include <Eigen/Dense>
#include <optional>

namespace control { struct ControlLawParameters; struct FullStateControlLawInput;} // forward declare

namespace control {

    struct LinearQuadraticController {
        Eigen::MatrixXd Q;
        Eigen::MatrixXd R;
        std::optional<Eigen::MatrixXd> K;

        LinearQuadraticController(const ControlLawParameters& params);

        Eigen::VectorXd _step(const FullStateControlLawInput& ctrl_law_input);
    };

    struct LinearQuadraticRegulator : LinearQuadraticController {
        LinearQuadraticRegulator(const ControlLawParameters& params);
    };

    struct LinearQuadraticTracker : LinearQuadraticController {
        LinearQuadraticTracker(const ControlLawParameters& params);
    };

}
