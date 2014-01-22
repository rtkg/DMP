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
#include <deque>
#include <assert.h>
#include <math.h>

#define DATA_PATH "/home/rkg/Data/Coding/DMP/data/"
//---------------------------------------------------------------------
std::vector<std::vector<int> > getDemoInd(std::deque<std::string> const & args)
{
  int nD=(int)args.size();
  std::vector<std::vector<int> > d_ind(nD);
  std::vector<int> ind;
  int nS=0;
  for(int i=0; i<nD;i++)
    {
      ind.resize(atoi(args[i].c_str()));
      for (int j=0;j<atoi(args[i].c_str());j++)
	ind[j]=nS+j;

      d_ind[i]=ind;
      nS+=atoi(args[i].c_str());
    }
  return d_ind;
}
//---------------------------------------------------------------------
std::vector<int>  getAInd(int const nBF, int const nD)
{
  std::vector<int> a_ind(nD);
  for(int i=0; i<nD;i++)
    a_ind[i]=(nBF+2)*i;

  return a_ind;
}
//---------------------------------------------------------------------
std::vector<int>  getBInd(int const nBF, int const nD)
{
  std::vector<int> b_ind(nD);
  for(int i=0; i<nD;i++)
    b_ind[i]=1+(nBF+2)*i;

  return b_ind;
}
//---------------------------------------------------------------------
std::vector<std::vector<int> > getWInd(int const nBF, int const nD)
{

  std::vector<std::vector<int> > w_ind(nD);
  std::vector<int> ind(nBF);
  for(int i=0; i<nD;i++)
    {
      for (int j=0;j<nBF;j++)   
	ind[j]=2+(nBF+2)*i+j;

      w_ind[i]=ind;
    }
  return w_ind;
}
//---------------------------------------------------------------------
std::vector<int>  getRInd(int const nBF, int const nD)
{
  std::vector<int> r_ind(nBF);
  for(int i=0; i<nBF;i++)
    r_ind[i]=nD*(2+nBF)+i*2;

  return r_ind;
}
//---------------------------------------------------------------------
std::vector<int>  getCInd(int const nBF, int const nD)
{
  std::vector<int> c_ind(nBF);
  for(int i=0; i<nBF;i++)
    c_ind[i]=nD*(2+nBF)+i*2+1;

  return c_ind;
}
//---------------------------------------------------------------------

int main(int argc, char **argv)
{
  USING_NAMESPACE_ACADO
  const double KKT_tol = 1e-4;

  //READ THE DEMO LENGTHS & nBF FROM THE COMMAND LINE
  std::deque<std::string> args(argv + 1, argv + argc + !argc);
  const int nBF=atoi(args[0].c_str());
  args.pop_front();
  const int nD=(int)args.size();
  int nS=0;
  for(int i=0; i<nD;i++)
    nS+=atoi(args[i].c_str());


  //READ DATA
  std::string path=DATA_PATH;
  Matrix D = readFromFile((path+"demos.dat").c_str()); //d(:,0)=time, d(:,1)=x, d(:,2)=dx, d(:,3)=ddx;
  Vector pI = readFromFile((path+"initial_guess.dat").c_str());
  Matrix A = readFromFile((path+"inequality_constraint_matrix.dat").c_str());
  Vector b = readFromFile((path+"inequality_constraint_vector.dat").c_str());
  Matrix Aeq = readFromFile((path+"equality_constraint_matrix.dat").c_str());
  Vector beq = readFromFile((path+"equality_constraint_vector.dat").c_str());
  Vector ub = readFromFile((path+"upper_bounds.dat").c_str());
  Vector lb = readFromFile((path+"lower_bounds.dat").c_str());
  Matrix S = readFromFile((path+"scale_matrix.dat").c_str());

  //RELEVANT INDEX SETS
  std::vector<std::vector<int> > d_ind=getDemoInd(args);
  std::vector<int> a_ind=getAInd(nBF,nD);
  std::vector<int> b_ind=getBInd(nBF,nD);
  std::vector<std::vector<int> > w_ind=getWInd(nBF,nD);
  std::vector<int> r_ind=getRInd(nBF,nD);
  std::vector<int> c_ind=getCInd(nBF,nD);

  //PARAMETER & OBJECTIVE FUNCTION
 Parameter p(2*nD+nBF*(2+nD),1);
 Expression B(nS,2*nD+nBF*(2+nD));  
 double t,x,dx;

 for (int d=0; d<nD; d++)
   for(int s=0;s<(int)d_ind[d].size();s++)
     {
       t=D(d_ind[d][s],0);
       x=D(d_ind[d][s],1);
       dx=D(d_ind[d][s],2);

       B(d_ind[d][s],a_ind[d])=x;
       B(d_ind[d][s],b_ind[d])=dx;

       for(int n=0;n<nBF;n++)
	  B(d_ind[d][s],w_ind[d][n])=(-0.5*(t-p(c_ind[n])*S(c_ind[n],c_ind[n])).getPowInt(2)/(p(r_ind[n])*p(r_ind[n])*S(r_ind[n],r_ind[n])*S(r_ind[n],r_ind[n]))).getExp();

     }           

 Expression f;
 f<<B*S*p-D.getCol(3);

  NLP nlp;
  nlp.minimize(0.5*f.transpose()*f);
  nlp.subjectTo(A*S*p <= b);
  nlp.subjectTo(lb <= S*p <= ub);

  if(beq.getDim() > 0)
    nlp.subjectTo(Aeq*S*p == beq);
  
  //ALGORITHM 
  ParameterEstimationAlgorithm algorithm(nlp);
  VariablesGrid initial_guess(2*nD+nBF*(2+nD),0.0,0.0,1 );
  initial_guess.setVector( 0,S.getInverse()*pI );
  algorithm.initializeParameters(initial_guess);
  
  // OPTIONS
  algorithm.set( KKT_TOLERANCE, KKT_tol);
  algorithm.set( ABSOLUTE_TOLERANCE, 1e-4);
  algorithm.set( PRINTLEVEL,HIGH);
  algorithm.set( MAX_NUM_ITERATIONS, 4000 );
  algorithm.set (PRINT_COPYRIGHT, NO);
  // algorithm.set (PRINT_SCP_METHOD_PROFILE, YES);
  algorithm.set( HESSIAN_APPROXIMATION, BLOCK_BFGS_UPDATE); 
  algorithm.set(GLOBALIZATION_STRATEGY, GS_LINESEARCH ); 
  algorithm.set(LINESEARCH_TOLERANCE, 1e-2 ); 
  algorithm.set(INFEASIBLE_QP_HANDLING,IQH_RELAX_L2);
  algorithm.set(FEASIBILITY_CHECK,BT_TRUE);

  // // LOGGING
  // LogRecord logRecord( LOG_AT_EACH_ITERATION,(path+"log.dat").c_str(),PS_PLAIN);
  // logRecord << LOG_OBJECTIVE_VALUE;
  // algorithm << logRecord;

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

