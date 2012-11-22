#include <acado_toolkit.hpp>
#include <include/acado_gnuplot/gnuplot_window.hpp>

int main(int argc, char **argv)
{

  USING_NAMESPACE_ACADO;

  double KKT_tol = 1e-3;
  
  // load data
  VariablesGrid data("../demonstrations/data.txt"); 

  // N=2 sets of demonstrations
  DifferentialState q1, dq1;
  DifferentialState q2, dq2;

  Parameter         a, b;

  Grid time_grid = data.getTimePoints(); 

  double x1_init[2] = {data.getFirstVector()(0), data.getFirstVector()(1)};
  double x2_init[2] = {data.getFirstVector()(2), data.getFirstVector()(3)};

  // ---------------------------------------------------
  // optimize
  // ---------------------------------------------------

  // define ODE
  DifferentialEquation f_opt;
  f_opt << dot(q1)  == dq1;
  f_opt << dot(dq1) == -a*q1-b*dq1;  
  f_opt << dot(q2)  == dq2;
  f_opt << dot(dq2) == -a*q2-b*dq2;
 
  Function h;
  h << q1;
  h << dq1;
  h << q2;
  h << dq2;
 
  // define OCP
  OCP ocp(time_grid);
  ocp.minimizeLSQ(h,data);
  
  // constraints
  ocp.subjectTo(f_opt);

  //ocp.subjectTo( AT_START, s   == 0);  
  ocp.subjectTo( AT_START, q1  == x1_init[0] );  
  ocp.subjectTo( AT_START, dq1 == x1_init[1] );  
  ocp.subjectTo( AT_START, q2  == x2_init[0] );  
  ocp.subjectTo( AT_START, dq2 == x2_init[1] );  
  ocp.subjectTo( AT_START, dq2 == x2_init[1] );  

  // define optimization algorithm
  OptimizationAlgorithm algorithm(ocp);

  // define some useful options
  algorithm.set( INTEGRATOR_TYPE, INT_RK78 );
  algorithm.set( KKT_TOLERANCE, KKT_tol);  
  algorithm.set( PRINTLEVEL, HIGH );

  algorithm.set( MAX_NUM_ITERATIONS, 200 );
  algorithm.set (PRINT_COPYRIGHT, NO);
  
  algorithm.set( GLOBALIZATION_STRATEGY, GS_FULLSTEP ); // <-- important
  algorithm.set( LEVENBERG_MARQUARDT, 1e-6 );

  // solve the problem
  algorithm.solve();

  // output solution
  VariablesGrid parameters, states_opt;
  algorithm.getParameters(parameters);
  algorithm.getDifferentialStates(states_opt);
  states_opt.printToFile( "../demonstrations/states_opt.txt","",PS_PLAIN );
  
  Vector p_sol = parameters.getLastVector();
  p_sol.print("parameters = \n");

  q1.clearStaticCounters(); 

  // ---------------------------------------------------
  // simulate
  // ---------------------------------------------------
  double a_sol = p_sol(0);
  double b_sol = p_sol(1);

  DifferentialEquation f_sim;
  DifferentialState q, dq;
  f_sim << dot(q)  == dq;
  f_sim << dot(dq) == -a_sol*q - b_sol*dq;

  IntegratorRK78 integrator( f_sim );  
  integrator.set( INTEGRATOR_PRINTLEVEL, NONE );
  integrator.set( INTEGRATOR_TOLERANCE, 1.0e-6 );

  // initial condition
  Vector x0(2);
  x0(0) = states_opt(0,0);
  x0(1) = states_opt(0,1);

  VariablesGrid states_sim;
  
  integrator.integrate( time_grid, x0 );
  integrator.getX( states_sim );
  states_sim.printToFile( "../demonstrations/states_sim.txt","",PS_PLAIN );

  return 0;
}
//EOF

