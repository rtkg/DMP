
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
  const double w_max   = 1e6;


  //READ DEMO & GAUSSIAN BASIS FUNCTIONS PARAMETERS
  std::string path=DATA_PATH;
  Matrix d = readFromFile((path+"demo.dat").c_str()); //d(:,0)=time, d(:,1)=x, d(:,2)=dx, d(:,3)=ddx;
  Vector pBF = readFromFile((path+"pBF.dat").c_str());
  assert(pBF.getDim() == (unsigned int)(2*nBF));
  const int nS = d.getNumRows();

  //PARAMETER VECTOR 
  Parameter p(2+nBF,1); 

  //OBJECTIVE FUNCTION
Expression f;

  Matrix B(nS,2+nBF); B.setZero();
  for(int i=0;i<nS;i++)
    for(int j=2;j<nBF+2;j++)
	B(i,j)= exp(-pow(d(i,0)-pBF(2*(j-2)+1),2)/(2*pow(pBF(2*(j-2)),2)));


  f<<d.getCol(1)*p(0)+d.getCol(2)*p(1)-d.getCol(3)+ B*p;

  Matrix uA(2+nBF,2+nBF); uA.setIdentity();
  Matrix lA(2+nBF,2+nBF); lA.setIdentity();
  Vector u(2+nBF); u(0)=-2; u(1)=-2; //upper bounds on a & b
  Vector l(2+nBF); l(0)= -1e3; l(1)=-1e3; //lower bounds on a & b

  for(int i=0;i<nBF;i++)
    {
      u(i+2)=w_max;
      l(i+2)=-w_max;
    }

  Matrix H(2+nBF,2+nBF); H.setIdentity();
  //H(1+nBF,1+nBF)=25;

  Expression E;
   E<<f.transpose()*f*0.5;

  // DEFINE OPTIMIZATION ALGORITHM AND SOLVE THE NLP:
  NLP nlp;
  nlp.minimize(E);
  nlp.subjectTo(uA*p <= u);
  nlp.subjectTo(lA*p >= l);

  ParameterEstimationAlgorithm algorithm(nlp);

   // algorithm.set( INTEGRATOR_TYPE, INT_RK45 ); 
  algorithm.set( KKT_TOLERANCE, KKT_tol);
  algorithm.set( ABSOLUTE_TOLERANCE, 1e-4);
  algorithm.set( PRINTLEVEL,LOW );
  algorithm.set( MAX_NUM_ITERATIONS, 200 );
  algorithm.set (PRINT_COPYRIGHT, NO);
  algorithm.set( HESSIAN_APPROXIMATION, EXACT_HESSIAN ); 
  algorithm.set(OBJECTIVE_SENSITIVITY , FORWARD_SENSITIVITY ); 
  algorithm.set(CONSTRAINT_SENSITIVITY, FORWARD_SENSITIVITY ); 
  algorithm.set(DYNAMIC_SENSITIVITY   , FORWARD_SENSITIVITY ); 

  
  double clock1 = clock();
  algorithm.solve();
  double clock2 = clock();

  printf("\n computation time (ACADO) = %.16e \n", (clock2-clock1)/CLOCKS_PER_SEC);

  Vector w;
  algorithm.getParameters(w);
  w.print("optimal solution \n");
  w.printToFile((path+"weights.dat").c_str(),"",PS_PLAIN);

	// MPCexport mpc( nlp );

	// mpc.set( HESSIAN_APPROXIMATION, GAUSS_NEWTON    );
	// mpc.set( DISCRETIZATION_TYPE,   SINGLE_SHOOTING );
	// mpc.set( INTEGRATOR_TYPE,       INT_RK4         );
	// mpc.set( NUM_INTEGRATOR_STEPS,  30              );
	// mpc.set( QP_SOLVER,             QP_QPOASES      );
	// mpc.set( HOTSTART_QP,           NO              );
	// mpc.set( GENERATE_TEST_FILE,    YES             );
	// mpc.set( GENERATE_MAKE_FILE,    YES             );

	// mpc.exportCode( "fwL2_export" );




  return 0;
}
//EOF

