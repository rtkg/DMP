/* the following command line arguments should be provided
  
   argv[1] -  NumberOfImagePoints (int)
   argv[2] -  InitialGuess        (int)
   argv[3] -  OPTION              (int)
   argv[4] -  KKT_tol             (double)

   Example (assuming 30 points are available in "measurements.txt"):
   -----------------------------------------------------------------
   ./exercise2 30 0 1 1e-06
*/


#include <include/acado_gnuplot/gnuplot_window.hpp>
#include <acado_toolkit.hpp>
#include <string>
#include <assert.h>
#include <math.h>

#define DATA_PATH "/home/rkg/Data/Coding/dynamical_systems/data/"


int main(int argc, char **argv)
{
  USING_NAMESPACE_ACADO

  const int nBF = atoi(argv[1]);
  const double KKT_tol = atof(argv[2]);


  //READ DATA
  std::string path=DATA_PATH;
  Matrix d = readFromFile((path+"demos.dat").c_str()); //d(:,0)=time, d(:,1)=x, d(:,2)=dx, d(:,3)=ddx;
  Vector pI = readFromFile((path+"initial_guess.dat").c_str());
  Matrix A = readFromFile((path+"inequality_constraint_matrix.dat").c_str());
  Vector b = readFromFile((path+"inequality_constraint_vector.dat").c_str());
  Matrix S = readFromFile((path+"scale_matrix.dat").c_str());
  const int nS = d.getNumRows();

  //PARAMETER VECTOR
  //p(0)=a, p(1)=b, p(2:1+nBF)=wi, p(2+nBF:1+3*nBF)=[ri,ci]
  Parameter p(2+3*nBF,1); 

  //OBJECTIVE FUNCTION
  Expression B;
  B.appendCols(d.getCol(1));
  B.appendCols(d.getCol(2));
  Matrix dummy(nS,3*nBF); dummy.setZero();  //try to do this with just initializing the expression
  B.appendCols(dummy);

  for(int i=0;i<nS;i++)
    for(int j=0;j<nBF;j++)
      B(i,j+2)= (-(d(i,0)-S(3+nBF+2*j,3+nBF+2*j)*p(3+nBF+2*j)).getPowInt(2)/(2*S(2+nBF+2*j,2+nBF+2*j)*S(2+nBF+2*j,2+nBF+2*j)*p(2+nBF+2*j)*p(2+nBF+2*j))).getExp();

  Expression f;
  f<<B*S*p-d.getCol(3);






  NLP nlp;
  nlp.minimize(0.5*f.transpose()*f);
  nlp.subjectTo(A*S*p <= b);
  


  //ALGORITHM 
  ParameterEstimationAlgorithm algorithm(nlp);
  VariablesGrid initial_guess( 2+3*nBF,0.0,0.0,1 );
  initial_guess.setVector( 0,S.getInverse()*pI );
  algorithm.initializeParameters(initial_guess);
  
  // OPTIONS
  algorithm.set( KKT_TOLERANCE, KKT_tol);
  algorithm.set( ABSOLUTE_TOLERANCE, 1e-4);
  algorithm.set( PRINTLEVEL,HIGH);
  algorithm.set( MAX_NUM_ITERATIONS, 2000 );
  algorithm.set (PRINT_COPYRIGHT, NO);
  // algorithm.set (PRINT_SCP_METHOD_PROFILE, YES);
  algorithm.set( HESSIAN_APPROXIMATION, EXACT_HESSIAN ); 
  algorithm.set(GLOBALIZATION_STRATEGY, GS_LINESEARCH ); 
  algorithm.set(LINESEARCH_TOLERANCE, 1e-2 ); 
  algorithm.set(INFEASIBLE_QP_HANDLING,IQH_RELAX_L2);
  algorithm.set(FEASIBILITY_CHECK,BT_TRUE);


  // LOGGING
  LogRecord logRecord( LOG_AT_EACH_ITERATION,(path+"log.dat").c_str(),PS_PLAIN);
  logRecord << LOG_OBJECTIVE_VALUE;
  algorithm << logRecord;

  //SOLVING
  double clock1 = clock();
  algorithm.solve();
  double clock2 = clock();
  Vector solution;
  algorithm.getParameters(solution);
  // solution.print("optimal solution \n");
  solution.printToFile((path+"solution.dat").c_str(),"",PS_PLAIN);
  printf("\n computation time (ACADO) = %.16e \n", (clock2-clock1)/CLOCKS_PER_SEC);

  return 0;
}
//EOF

