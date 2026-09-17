convergence.csv        : RK4 order verification on the damped harmonic oscillator.
                         Column 'integrator' is 0 (rk4). 'observed_order' is
                         log(e_{k-1}/e_k)/log(h_{k-1}/h_k).
dopri_tolerance.csv    : Dormand-Prince 5(4) error/cost vs requested rel_tol on
                         the same problem.
aircraft_comparison.csv: nonlinear 6-DOF errors against a rel_tol = 1e-13
                         reference. Rows 1-6 are RK4 (dt_or_rtol = step size),
                         rows 7-10 are DOPRI 5(4) (dt_or_rtol = rel_tol).
