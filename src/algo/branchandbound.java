package algo;

import java.io.IOException;
import java.util.ArrayList;

public class branchandbound {
	double lowerbound;
	double upperbound;

	public branchandbound() {
		lowerbound = -1E10;
		upperbound = 1E10;
	}

	class treeBB {
		// this is a linked tree list recording all the branching during Branch and Bound
		treeBB father; // link to the node processed before branching
		treeBB son0; // link to the son on the left of the tree (edge=0; first processed) => need it to compute the global lowerbound
		int branchFrom; // we branch on edges between cities => city origin of the edge
		int branchTo; // we branch on edges between cities => city destination of the edge
		int branchValue; // we branch on edges between cities => value of the branching (remove edge=0; set edge=1)
		double lowestValue; // lower bound on the solution if we start from this node (i.e. looking only down for this tree)
		boolean toplevel; // to compute the global lowerBound, need to know if everything above has been considered
	}

	public void EdgesBasedOnBranching(paramsVRP userParam, treeBB branching,
	                                  boolean recur) {
		int i;
		if (branching.father != null) { // stop before root node
			if (branching.branchValue == 0) { // forbid this edge (in this direction)
				// associate a very large distance to this edge to make it unattractive
				userParam.dist[branching.branchFrom][branching.branchTo] = userParam.verybig;
			} else {
			    // impose this edge (in this direction)
				// associate a very large and unattractive distance to all edges
				// starting from "branchFrom" excepted the one leading to "branchTo"
				// and excepted when we start from depot (several vehicles)
				if (branching.branchFrom != 0) {
					for (i = 0; i < branching.branchTo; i++)
						userParam.dist[branching.branchFrom][i] = userParam.verybig;
					for (i++; i < userParam.nbclients + 2; i++)
						userParam.dist[branching.branchFrom][i] = userParam.verybig;
				}
				// associate a very large and unattractive distance to all edges ending
				// at "branchTo" excepted the one starting from "branchFrom"
				// and excepted when the destination is the depot (several vehicles)
				if (branching.branchTo != userParam.nbclients + 1) {
					for (i = 0; i < branching.branchFrom; i++)
						userParam.dist[i][branching.branchTo] = userParam.verybig;
					for (i++; i < userParam.nbclients + 2; i++)
						userParam.dist[i][branching.branchTo] = userParam.verybig;
				}
				// forbid the edge in the opposite direction
				userParam.dist[branching.branchTo][branching.branchFrom] = userParam.verybig;
			}
			if (recur) EdgesBasedOnBranching(userParam, branching.father, true);
		}
	}

	public boolean BBNode(paramsVRP userParam, ArrayList<route> routes,
                          treeBB branching, ArrayList<route> bestRoutes, int depth)
			throws IOException {
		// userParam (input) : all the parameters provided by the users (cities,
		// roads...)
		// routes (input) : all (but we could decide to keep only a subset) the
		// routes considered up to now (to initialize the Column generation process)
		// branching (input): BB branching context information for the current node
		// to process (branching edge var, branching value, branching from...)
		// bestRoutes (output): best solution encountered
		int i, j, bestEdge1, bestEdge2, prevcity, city, bestVal;
		double coef, bestObj, change, CGobj;
		boolean feasible;

		try {

			// check first that we need to solve this node. Not the case if we have
			// already found a solution within the gap precision
			if ((upperbound - lowerbound) / upperbound < userParam.gap)
				return true;

			// init
			if (branching == null) { // root node - first call
				// first call - root node
				treeBB newNode = new treeBB();
				newNode.father = null;
				newNode.toplevel = true;
				newNode.branchFrom = -1;
				newNode.branchTo = -1;
				newNode.branchValue = -1;
				newNode.son0 = null;
				branching = newNode;
			}

			// display some local info
			if (branching.branchValue < 1) {
				System.out.println("\nEdge from " + branching.branchFrom + " to "
						+ branching.branchTo + ": forbid");
			} else {
				System.out.println("\nEdge from " + branching.branchFrom + " to "
						+ branching.branchTo + ": set");
			}
			int MB = 1024 * 1024;
			Runtime runtime = Runtime.getRuntime();
			System.out.print("Java Memory=> Total:" + (runtime.totalMemory() / MB)
					+ " Max:" + (runtime.maxMemory() / MB) + " Used:"
					+ ((runtime.totalMemory() - runtime.freeMemory()) / MB) + " Free: "
					+ runtime.freeMemory() / MB);

			// Compute a solution for this node using Column generation
			columngen CG = new columngen();

			CGobj = CG.computeColGen(userParam, routes);
			// feasible ? Does a solution exist?
			if ((CGobj > 2 * userParam.maxlength) || (CGobj < -1e-6)) {
				// can only be true when the routes in the solution include forbidden edges (can happen when the BB set branching values)
				System.out.println("RELAX INFEASIBLE | Lower bound: " + lowerbound
						+ " | Upper bound: " + upperbound + " | Gap: "
						+ ((upperbound - lowerbound) / upperbound) + " | BB Depth: "
						+ depth + " | " + routes.size() + " routes");
				return true; // stop this branch
			}
			branching.lowestValue = CGobj;

			// update the global lowerBound when required
			if ((branching.father != null) && (branching.father.son0 != null)
					&& branching.father.toplevel) {
				// all nodes above and on the left have been processed=> we can compute
				// a new lowerBound
				lowerbound = Math.min(branching.lowestValue, branching.father.son0.lowestValue);
				branching.toplevel = true;
			} else if (branching.father == null) {
				// root node
				lowerbound = CGobj;
			}

			if (branching.lowestValue > upperbound) {
				CG = null;
				System.out.println("CUT | Lower bound: " + lowerbound
						+ " | Upper bound: " + upperbound + " | Gap: "
						+ ((upperbound - lowerbound) / upperbound) + " | BB Depth: "
						+ depth + " | Local CG cost: " + CGobj + " | " + routes.size()
						+ " routes");
				return true; // cut this useless branch
			} else {
				// ///////////////////////////////////////////////////////////////////////////
				// check the (integer) feasibility. Otherwise search for a branching
				// variable
				feasible = true;
				bestEdge1 = -1;
				bestEdge2 = -1;
				bestObj = -1.0;
				bestVal = 0;

				// transform the path variable (of the CG model) into edges variables
				for (i = 0; i < userParam.nbclients + 2; i++) {
					java.util.Arrays.fill(userParam.edges[i], 0.0);
				}
				for (route r : routes) {
					if (r.getQ() > 1e-6) {
						// we consider only the routes in the current local solution
						ArrayList<Integer> path = r.getpath(); // get back the sequence of
						// cities (path for this route)
						prevcity = 0;
						for (i = 1; i < path.size(); i++) {
							city = path.get(i);
							userParam.edges[prevcity][city] += r.getQ(); // convert into edges
							prevcity = city;
						}
					}
				}

				// find a fractional edge
				for (i = 0; i < userParam.nbclients + 2; i++) {
					for (j = 0; j < userParam.nbclients + 2; j++) {
						coef = userParam.edges[i][j];
						if ((coef > 1e-6) && ((coef < 0.9999999999) || (coef > 1.0000000001))) {
							// this route has a fractional coefficient in the solution =>
							// should we branch on this one?
							feasible = false;
							// what if we impose this route in the solution? Q=1
							// keep the ref of the edge which should lead to the largest change
							change = Math.min(coef, Math.abs(1.0 - coef));
							change *= routes.get(i).getcost();
							if (change > bestObj) {
								bestEdge1 = i;
								bestEdge2 = j;
								bestObj = change;
								bestVal = (Math.abs(1.0 - coef) > coef) ? 0 : 1;
							}
						}
					}
				}

				if (feasible) {
					if (branching.lowestValue < upperbound) { // new incumbant feasible solution!
						upperbound = branching.lowestValue;
						bestRoutes.clear();
						for (route r : routes) {
							if (r.getQ() > 1e-6) {
								route optim = new route();
								optim.setcost(r.getcost());
								optim.path = r.getpath();
								optim.setQ(r.getQ());
								bestRoutes.add(optim);
							}
						}
						System.out.println("OPT | Lower bound: " + lowerbound
								+ " | Upper bound: " + upperbound + " | Gap: "
								+ ((upperbound - lowerbound) / upperbound) + " | BB Depth: "
								+ depth + " | Local CG cost: " + CGobj + " | " + routes.size()
								+ " routes");
						System.out.flush();
					} else {
                        System.out.println("FEAS | Lower bound: " + lowerbound
                                + " | Upper bound: " + upperbound + " | Gap: "
                                + ((upperbound - lowerbound) / upperbound) + " | BB Depth: "
                                + depth + " | Local CG cost: " + CGobj + " | " + routes.size()
                                + " routes");
                    }
					return true;
				} else {
					System.out.println("INTEG INFEAS | Lower bound: " + lowerbound
							+ " | Upper bound: " + upperbound + " | Gap: "
							+ ((upperbound - lowerbound) / upperbound) + " | BB Depth: "
							+ depth + " | Local CG cost: " + CGobj + " | " + routes.size()
							+ " routes");
					System.out.flush();
					// ///////////////////////////////////////////////////////////
					// branching (diving strategy)

					// first branch -> set edges[bestEdge1][bestEdge2]=0
					// record the branching information in a tree list
					treeBB newNode1 = new treeBB();
					newNode1.father = branching;
					newNode1.branchFrom = bestEdge1;
					newNode1.branchTo = bestEdge2;
					newNode1.branchValue = bestVal; // first version was not with bestVal
					// but with 0
					newNode1.lowestValue = -1E10;
					newNode1.son0 = null;

					// branching on edges[bestEdge1][bestEdge2]=0
					EdgesBasedOnBranching(userParam, newNode1, false);

					// the initial lp for the CG contains all the routes of the previous
					// solution less(去掉分支的边) the routes containing this arc
					ArrayList<route> nodeRoutes = new ArrayList<route>();
					for (route r : routes) {
						ArrayList<Integer> path = r.getpath();
						boolean accept = true;
						if (path.size() > 3) { // we must keep trivial routes
							// Depot-City-Depot in the set to ensure
							// feasibility of the CG
							prevcity = 0;
							for (j = 1; accept && (j < path.size()); j++) {
								city = path.get(j);
								if ((prevcity == bestEdge1) && (city == bestEdge2))
									accept = false;
								prevcity = city;
							}
						}
						if (accept) nodeRoutes.add(r);
					}

					boolean ok;
					ok = BBNode(userParam, nodeRoutes, newNode1, bestRoutes, depth + 1);
					nodeRoutes = null; // free memory
					if (!ok) {
						return false;
					}

					branching.son0 = newNode1;

					// second branch -> set edges[bestEdge1][bestEdge2]=1
					// record the branching information in a tree list
					treeBB newNode2 = new treeBB();
					newNode2.father = branching;
					newNode2.branchFrom = bestEdge1;
					newNode2.branchTo = bestEdge2;
					newNode2.branchValue = 1 - bestVal; // first version: always 1
					newNode2.lowestValue = -1E10;
					newNode2.son0 = null;

					// branching on edges[bestEdge1][bestEdge2]=1
					// second branching=>need to reinitialize the dist matrix
					for (i = 0; i < userParam.nbclients + 2; i++) {
                        System.arraycopy(userParam.distBase[i], 0, userParam.dist[i], 0,
                              userParam.nbclients + 2);
                    }
					//reinitialize了因此需要recur递归一下
					EdgesBasedOnBranching(userParam, newNode2, true);
					// the initial lp for the CG contains all the routes of the previous
					// solution less the routes incompatible with this arc
					ArrayList<route> nodeRoutes2 = new ArrayList<route>();
					for (route r : routes) {
						ArrayList<Integer> path = r.getpath();
						boolean accept = true;
						if (path.size() > 3) { // we must keep trivial routes
							// Depot-City-Depot in the set to ensure
							// feasibility of the CG
							prevcity = 0;
							for (i = 1; accept && (i < path.size()); i++) {
								city = path.get(i);
								if (userParam.dist[prevcity][city] >= userParam.verybig - 1E-6) accept = false;
								prevcity = city;
							}
						}
						if (accept) nodeRoutes2.add(r);
					}
					ok = BBNode(userParam, nodeRoutes2, newNode2, bestRoutes, depth + 1);
					nodeRoutes2 = null;

					// update lowest feasible value of this node
					branching.lowestValue = Math.min(newNode1.lowestValue, newNode2.lowestValue);

					return ok;
				}
			}

		} catch (IOException e) {
			System.err.println("Error: " + e);
		}
		return false;
	}

}
