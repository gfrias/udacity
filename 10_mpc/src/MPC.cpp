#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
#include <ctime>

using CppAD::AD;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

size_t x_start;
size_t y_start;
size_t psi_start;
size_t v_start;
size_t cte_start;
size_t epsi_start;
size_t delta_start;
size_t a_start;

AD<double> polyeval(Eigen::VectorXd coeffs, AD<double> x) {
    AD<double> result = 0.0;
    for (int i = 0; i < coeffs.size(); i++) {
        result += coeffs[i] * CppAD::pow(x, i);
    }
    return result;
}

AD<double> polyder(Eigen::VectorXd coeffs, AD<double> x) {
    AD<double> result = 0.0;
    for (int i = 1; i < coeffs.size(); i++) {
        result += i * coeffs[i] * CppAD::pow(x, i-1);
    }
    return result;
}

class FG_eval {
public:
    // Fitted polynomial coefficients
    Eigen::VectorXd coeffs;
    Config config;
  
    FG_eval(Eigen::VectorXd coeffs, Config config) {
      this->coeffs = coeffs;
      this->config = config;
    }
  
    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
    void operator()(ADvector& fg, const ADvector& vars) {
        // The cost is stored is the first element of `fg`.
        // Any additions to the cost should be added to `fg[0]`.
        fg[0] = 0;
        
        // The part of the cost based on the reference state.
        for (int i = 0; i < config.solver_N; i++) {
          fg[0] += config.w_cte*CppAD::pow(vars[cte_start + i] - config.ref_cte, 2);
          fg[0] += config.w_epsi*CppAD::pow(vars[epsi_start + i] - config.ref_epsi, 2);
          fg[0] += config.w_v*CppAD::pow(vars[v_start + i] - config.ref_v, 2);
        }
        
        // Minimize the use of actuators.
        for (int i = 0; i < config.solver_N - 1; i++) {
            fg[0] += config.w_delta*CppAD::pow(vars[delta_start + i], 2);
            fg[0] += config.w_a*CppAD::pow(vars[a_start + i], 2);
        }

        // Minimize the value gap between sequential actuations.
        for (int i = 0; i < config.solver_N - 2; i++) {
            fg[0] += config.w_deltadot*CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2);
            fg[0] += config.w_adot*CppAD::pow(vars[a_start + i + 1] - vars[a_start + i], 2);
        }
        
        //
        // Setup Constraints
        //
        // NOTE: In this section you'll setup the model constraints.
        
        // Initial constraints
        //
        // We add 1 to each of the starting indices due to cost being located at
        // index 0 of `fg`.
        // This bumps up the position of all the other values.
        fg[1 + x_start] = vars[x_start];
        fg[1 + y_start] = vars[y_start];
        fg[1 + psi_start] = vars[psi_start];
        fg[1 + v_start] = vars[v_start];
        fg[1 + cte_start] = vars[cte_start];
        fg[1 + epsi_start] = vars[epsi_start];
        
        // The rest of the constraints
        for (int i = 0; i < config.solver_N - 1; i++) {
            // The state at time t+1 .
            AD<double> x1 = vars[x_start + i + 1];
            AD<double> y1 = vars[y_start + i + 1];
            AD<double> psi1 = vars[psi_start + i + 1];
            AD<double> v1 = vars[v_start + i + 1];
            AD<double> cte1 = vars[cte_start + i + 1];
            AD<double> epsi1 = vars[epsi_start + i + 1];
            
            // The state at time t.
            AD<double> x0 = vars[x_start + i];
            AD<double> y0 = vars[y_start + i];
            AD<double> psi0 = vars[psi_start + i];
            AD<double> v0 = vars[v_start + i];
            AD<double> cte0 = vars[cte_start + i];
            AD<double> epsi0 = vars[epsi_start + i];
          
            // Only consider the actuation at time t.
            AD<double> delta0 = vars[delta_start + i];
            AD<double> a0 = vars[a_start + i];
            
            AD<double> f0 = polyeval(coeffs, x0);
            AD<double> psides0 = CppAD::atan(polyder(coeffs, x0));
            
            // Here's `x` to get you started.
            // The idea here is to constraint this value to be 0.
            //
            // Recall the equations for the model:
            // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
            // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
            // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
            // v_[t+1] = v[t] + a[t] * dt
            // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
            // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
            fg[2 + x_start + i] = x1 - (x0 + v0 * CppAD::cos(psi0) * config.solver_dt);
            fg[2 + y_start + i] = y1 - (y0 + v0 * CppAD::sin(psi0) * config.solver_dt);
            fg[2 + psi_start + i] = psi1 - (psi0 + v0 * delta0 / Lf * config.solver_dt);
            fg[2 + v_start + i] = v1 - (v0 + a0 * config.solver_dt);
            fg[2 + cte_start + i] =
            cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * config.solver_dt));
            fg[2 + epsi_start + i] =
            epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * config.solver_dt);
        }
    }
};

//
// MPC class definition implementation.
//
MPC::MPC(Config config) {
  this->config = config;

  x_start = 0;
  y_start = x_start + config.solver_N;
  psi_start = y_start + config.solver_N;
  v_start = psi_start + config.solver_N;
  cte_start = v_start + config.solver_N;
  epsi_start = cte_start + config.solver_N;
  delta_start = epsi_start + config.solver_N;
  a_start = delta_start + config.solver_N - 1;
}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd x0, Eigen::VectorXd coeffs) {
    size_t i;
    typedef CPPAD_TESTVECTOR(double) Dvector;
    
    double x = x0[0];
    double y = x0[1];
    double psi = x0[2];
    double v = x0[3];
    double cte = x0[4];
    double epsi = x0[5];
  
    double delta = x0[6];
    double a = x0[7];
  
    //apply delay
    double dt = config.delay;
  
    x = x + v*cos(psi)*dt;
    y = y + v*sin(psi)*dt;
    psi = psi + v*delta*dt/Lf;
    v = v + a*dt;
    cte = cte + (v * sin(epsi) * dt);
    epsi = epsi + v * delta * dt / Lf;
  
    // number of independent variables
    // N timesteps == N - 1 actuations
    size_t n_vars = config.solver_N * 6 + (config.solver_N - 1) * 2;
    // Number of constraints
    size_t n_constraints = config.solver_N * 6;
    
    // Initial value of the independent variables.
    // Should be 0 except for the initial values.
    Dvector vars(n_vars);
    for (i = 0; i < n_vars; i++) {
        vars[i] = 0.0;
    }
    // Set the initial variable values
    vars[x_start] = x;
    vars[y_start] = y;
    vars[psi_start] = psi;
    vars[v_start] = v;
    vars[cte_start] = cte;
    vars[epsi_start] = epsi;
    
    // Lower and upper limits for x
    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);
    
    // Set all non-actuators upper and lowerlimits
    // to the max negative and positive values.
    for (i = 0; i < delta_start; i++) {
        vars_lowerbound[i] = -1.0e19;
        vars_upperbound[i] = 1.0e19;
    }
    
    // The upper and lower limits of delta are set to -25 and 25
    // degrees (values in radians).
    // NOTE: Feel free to change this to something else.
    for (i = delta_start; i < a_start; i++) {
        vars_lowerbound[i] = -0.436332;
        vars_upperbound[i] = 0.436332;
    }
    
    // Acceleration/decceleration upper and lower limits.
    // NOTE: Feel free to change this to something else.
    for (i = a_start; i < n_vars; i++) {
        vars_lowerbound[i] = -1.0;
        vars_upperbound[i] = 1.0;
    }

    // Lower and upper limits for constraints
    // All of these should be 0 except the initial
    // state indices.
    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);
    for (i = 0; i < n_constraints; i++) {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
    }
  
    constraints_lowerbound[x_start] = x;
    constraints_lowerbound[y_start] = y;
    constraints_lowerbound[psi_start] = psi;
    constraints_lowerbound[v_start] = v;
    constraints_lowerbound[cte_start] = cte;
    constraints_lowerbound[epsi_start] = epsi;
    
    constraints_upperbound[x_start] = x;
    constraints_upperbound[y_start] = y;
    constraints_upperbound[psi_start] = psi;
    constraints_upperbound[v_start] = v;
    constraints_upperbound[cte_start] = cte;
    constraints_upperbound[epsi_start] = epsi;
  
    // Object that computes objective and constraints
    FG_eval fg_eval(coeffs, config);
    
    // options
    std::string options;
    options += "Integer print_level  0\n";
    options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";
    
    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;
    
    // solve the problem
    CppAD::ipopt::solve<Dvector, FG_eval>(
                                          options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
                                          constraints_upperbound, fg_eval, solution);
    
    //
    // Check some of the solution values
    //
    bool ok = true;
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;
  
    vector<double> ret =
    
    {solution.x[x_start + 1],   solution.x[y_start + 1],
        solution.x[psi_start + 1], solution.x[v_start + 1],
        solution.x[cte_start + 1], solution.x[epsi_start + 1],
        solution.x[delta_start],   solution.x[a_start]};
    
    for (int i = x_start+1; i < y_start; i++) {
        ret.push_back(solution.x[i]);
    }
    for (int i = y_start+1; i < psi_start; i++) {
        ret.push_back(solution.x[i]);
    }
    auto cost = solution.obj_value;
    std::cout << "Cost " << cost << std::endl;
    
    return ret;
}
