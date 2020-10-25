package algo;

/**
 * Asymetric VRP with Resources Constraints (Time Windows and Capacity)
 * Branch and Price algorithm (Branch and Bound + Column generation)
 * For educational purpose only!  No code optimization.  Just to understand the main basic steps of the B&P algorithm.
 * Pricing through Dynamic Programming of the Short Path Problem with Resources Constraints (SPPRC)
 * Algorithm inspired by the book
 * Desrosiers, Desaulniers, Solomon, "Column Generation", Springer, 2005 (GERAD, 25th anniversary)
 * => Branch and bound (class BPACVRPTW)
 * => Column generation (class columngen) : chapter 3
 * => Pricing SPPRC (class SPPRC): chapter 2
 * CPLEX code for the column generation inspired by the example "CutStock.java" provided in the examples directory of the IBM ILOG CPLEX distribution
 *
 * @author mschyns
 * M.Schyns@ulg.ac.be
 */

import java.io.IOException;
import java.text.DecimalFormat;
import java.util.ArrayList;

import ilog.concert.*;
import ilog.cplex.*;

public class columngen {

	static class IloNumVarArray {
		// Creation of a new class similar to an ArrayList for CPLEX unknowns
		// taken from "cutsotck.java"
		int _num = 0;
		IloNumVar[] _array = new IloNumVar[32];

		void add(IloNumVar ivar) {
			if (_num >= _array.length) {
				IloNumVar[] array = new IloNumVar[2 * _array.length];
				System.arraycopy(_array, 0, array, 0, _num);
				_array = array;
			}
			_array[_num++] = ivar;
		}

		IloNumVar getElement(int i) {
			return _array[i];
		}

		int getSize() {
			return _num;
		}
	}

	public double computeColGen(paramsVRP userParam, ArrayList<route> routes)
			throws IOException {
		int i, j, prevcity, city;
		double cost, obj;
		double[] pi;
		boolean oncemore;

		try {

			// ---------------------------------------------------------
			// construct the model for the Restricted Master Problem
			// ---------------------------------------------------------
			// warning: for clarity, we create a new cplex env each time we start a
			// Column Generation
			// this class contains (nearly) everything about CG and could be used
			// independently
			// However, since the final goal is to encompass it inside 锟� Branch and
			// Bound (BB),
			// it would (probably) be better to create only once the CPlex env when we
			// initiate the BB and to work with the same (but adjusted) lp matrix each
			// time
			IloCplex cplex = new IloCplex();

			IloObjective objfunc = cplex.addMinimize();

			// for each vertex/client, one constraint (chapter 3, 3.23 )
			IloRange[] lpmatrix = new IloRange[userParam.nbclients];
			for (i = 0; i < userParam.nbclients; i++)
				lpmatrix[i] = cplex.addRange(1.0, Double.MAX_VALUE);
			// for each constraint, right member >=1
			// lpmatrix[i] = cplex.addRange(1.0, 1.0);
			// or for each constraint, right member=1 ... what is the best?

			// Declaration of the variables
			IloNumVarArray y = new IloNumVarArray(); // y_p to define whether a path p
			// is used

			// Populate the lp matrix and the objective function
			// first with the routes provided by the argument 'routes' of the function
			// (in the context of the Branch and Bound, it would be a pity to start
			// again the CG from scratch at each node of the BB!)
			// (we should reuse parts of the previous solution(s))
			for (route r : routes) {
				int v;
				cost = 0.0;
				prevcity = 0;
				for (i = 1; i < r.getpath().size(); i++) {
					city = r.getpath().get(i);
					cost += userParam.dist[prevcity][city];
					prevcity = city;
				}

				r.setcost(cost);
				IloColumn column = cplex.column(objfunc, r.getcost());
				// obj coefficient
				for (i = 1; i < r.getpath().size() - 1; i++) {
					v = r.getpath().get(i) - 1;
					column = column.and(cplex.column(lpmatrix[v], 1.0));
					// coefficient of y_i in (3.23) => 0 for the other y_p
				}
				y.add(cplex.numVar(column, 0.0, Double.MAX_VALUE));
				// creation of the variable y_i
			}
			// complete the lp with basic route to ensure feasibility
			if (routes.size() < userParam.nbclients) { // a priori true only the first time
				for (i = 0; i < userParam.nbclients; i++) {
					cost = userParam.dist[0][i + 1]
							+ userParam.dist[i + 1][userParam.nbclients + 1];
					IloColumn column = cplex.column(objfunc, cost); // obj coefficient
					column = column.and(cplex.column(lpmatrix[i], 1.0)); // coefficient of
					// y_i in (3.23)
					// => 0 for the
					// other y_p
					y.add(cplex.numVar(column, 0.0, Double.MAX_VALUE)); // creation of the
					// variable y_i
					route newroute = new route();
					newroute.addcity(0);
					newroute.addcity(i + 1);
					newroute.addcity(userParam.nbclients + 1);
					newroute.setcost(cost);
					routes.add(newroute);
				}
			}

			// cplex.exportModel("model.lp");

			// CPlex params
			cplex.setParam(IloCplex.IntParam.RootAlgorithm, IloCplex.Algorithm.Primal);
			cplex.setOut(null);
			// cplex.setParam(IloCplex.DoubleParam.TiLim,30); // max number of
			// seconds: 2h=7200 24h=86400

			// ---------------------------------------------------------
			// column generation process
			// ---------------------------------------------------------
			DecimalFormat df = new DecimalFormat("#0000.00");
			oncemore = true;
			double[] prevobj = new double[100];
			int nbroute;
			int previ = -1;
			while (oncemore) {

				oncemore = false;
				// ---------------------------------------s------------------
				// solve the current RMP
				// ---------------------------------------------------------
				if (!cplex.solve()) {
					System.out.println("CG: relaxation infeasible!");
					return 1E10;
				}
				prevobj[(++previ) % 100] = cplex.getObjValue();
				// store the 30 last obj values to check stability afterwards

				// System.out.println(cplex.getStatus());
				// cplex.exportModel("model.lp");

				// ---------------------------------------------------------
				// solve the subproblem to find new columns (if any)
				// ---------------------------------------------------------
				// first define the new costs for the subproblem objective function
				// (SPPRC)
				pi = cplex.getDuals(lpmatrix);
				for (i = 1; i < userParam.nbclients + 1; i++)
					for (j = 0; j < userParam.nbclients + 2; j++)
						userParam.cost[i][j] = userParam.dist[i][j] - pi[i - 1];

				// start dynamic programming
				SPPRC sp = new SPPRC();
				ArrayList<route> routesSPPRC = new ArrayList<route>();

				nbroute = userParam.nbclients; // arbitrarily limit to the 5 first
				// shortest paths with negative cost
				// if ((previ>100) &&
				// (prevobj[(previ-3)%100]-prevobj[previ%100]<0.0003*Math.abs((prevobj[(previ-99)%100]-prevobj[previ%100]))))
				// {
				// System.out.print("/");
				// complete=true; // it the convergence is too slow, start a "complete"
				// shortestpast
				// }
				sp.shortestPath(userParam, routesSPPRC, nbroute);
				sp = null;

				// /////////////////////////////
				// parameter here
				if (routesSPPRC.size() > 0) {
					for (route r : routesSPPRC) {
//             if (userParam.debug) {
//             System.out.println(" "+r.getcost());
//             }
						ArrayList<Integer> rout = r.getpath();
						prevcity = rout.get(1);
						cost = userParam.dist[0][prevcity];
						IloColumn column = cplex.column(lpmatrix[rout.get(1) - 1], 1.0);
						for (i = 2; i < rout.size() - 1; i++) {
							city = rout.get(i);
							cost += userParam.dist[prevcity][city];
							prevcity = city;
							column = column.and(cplex.column(lpmatrix[rout.get(i) - 1], 1.0));
							// coefficient of y_i in (3.23) => 0 for the other y_p
						}
						cost += userParam.dist[prevcity][userParam.nbclients + 1];
						column = column.and(cplex.column(objfunc, cost));
						y.add(cplex.numVar(column, 0.0, Double.MAX_VALUE,
								"P" + routes.size())); // creation of the variable y_i
						r.setcost(cost);
						routes.add(r);

						oncemore = true;
					}
					System.out.print("\nCG Iter " + previ + " Current cost: "
							+ df.format(prevobj[previ % 100]) + " " + routes.size()
							+ " routes");
					System.out.flush();
				}
				//if (previ % 50 == 0)
				routesSPPRC = null;
			}

			System.out.println();

			for (i = 0; i < y.getSize(); i++)
				routes.get(i).setQ(cplex.getValue(y.getElement(i)));
			obj = cplex.getObjValue(); // mmmmhhh: to check. To be entirely safe, we
			// should recompute the obj using the distBase
			// matrix instead of the dist matrix

			cplex.end();
			return obj;
		} catch (IloException e) {
			System.err.println("Concert exception caught '" + e + "' caught");
		}
		return 1E10;
	}
}
