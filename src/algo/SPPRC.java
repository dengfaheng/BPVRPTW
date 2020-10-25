package algo;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.TreeSet;

// shortest path with resource constraints
// inspired by Irnish and Desaulniers, "SHORTEST PATH PROBLEMS WITH RESOURCE CONSTRAINTS"
// for educational demonstration only - (nearly) no code optimization
//
// four main lists will be used:
// labels: array (ArrayList) => one dimensional unbounded vector
//		 list of all labels created along the feasible paths (i.e. paths satisfying the resource constraints)
//		
// U: sorted list (TreeSet) => one dimensional unbounded vector
//		sorted list containing the indices of the unprocessed labels (paths that can be extended to obtain a longer feasible path)
//
// P: sorted list (TreeSet) => one dimensional unbounded vector
//		sorted list containing the indices of the processed labels ending at the depot with a negative cost
//
// city2labels: matrix (array of ArrayList) => nbClients x unbounded
//		for each city, the list of (indices of the) labels attached to this city/vertex
//		before processing a label at vertex i, we compare pairwise all labels at the same vertex to remove the dominated ones

public class SPPRC {
	paramsVRP userParam;
	ArrayList<label> labels;

	class label {
		// we use a labelling algorithm.
		// labels are attached to each vertex to specify the state of the resources
		// when we follow a corresponding feasible path ending at this vertex
		public int city;                // current vertex
		public int indexPrevLabel;    // previous label in the same path (i.e. previous vertex in the same path with the state of the resources)
		public double cost;                // first resource: cost (e.g. distance or strict travel time)
		public float tTime;                // second resource: travel time along the path (including wait time and service time)
		public double demand;                // third resource: demand,i.e. total quantity delivered to the clients encountered on this path
		public boolean dominated;            // is this label dominated by another one? i.e. if dominated, forget this path.
		public boolean[] vertexVisited;

		label(int a1, int a2, double a3, float a4, double a5, boolean a6, boolean[] a7) {
			city = a1;
			indexPrevLabel = a2;
			cost = a3;
			tTime = a4;
			demand = a5;
			dominated = a6;
			vertexVisited = a7;
		}
	}

	class MyLabelComparator implements Comparator<Integer> {
		// the U treeSet is an ordered list
		// to maintain the order, we need to define a comparator: cost is the main criterium
		public int compare(Integer a, Integer b) {
			label A = labels.get(a);
			label B = labels.get(b);

			// Be careful!  When the comparator returns 0, it means that the two labels are considered EXACTLY the same ones!
			// This comparator is not only used to sort the lists!  When adding to the list, a value of 0 => not added!!!!!
			if (A.cost - B.cost < -1e-7)
				return -1;
			else if (A.cost - B.cost > 1e-7)
				return 1;
			else {
				if (A.city == B.city) {
					if (A.tTime - B.tTime < -1e-7)
						return -1;
					else if (A.tTime - B.tTime > 1e-7)
						return 1;
					else if (A.demand - B.demand < -1e-7)
						return -1;
					else if (A.demand - B.demand > 1e-7)
						return 1;
					else {
						int i = 0;
						while (i < userParam.nbclients + 2) {
							if (A.vertexVisited[i] != B.vertexVisited[i]) {
								if (A.vertexVisited[i])
									return -1;
								else
									return 1;
							}
							i++;
						}
						return 0;
					}
				} else if (A.city > B.city)
					return 1;
				else
					return -1;
			}
		}
	}


	public void shortestPath(paramsVRP userParamArg, ArrayList<route> routes, int nbRoute) {
		label current;
		int i, j, idx, nbsol, maxSol;
		double d, d2;
		int[] checkDom;
		float tt, tt2;
		Integer currentidx;

		this.userParam = userParamArg;
		// unprocessed labels list => ordered TreeSet List (?optimal:  need to be sorted like this?)
		TreeSet<Integer> U = new TreeSet<Integer>(new MyLabelComparator());   // unprocessed labels list

		// processed labels list => ordered TreeSet List 
		TreeSet<Integer> P = new TreeSet<Integer>(new MyLabelComparator());   // processed labels list

		// array of labels
		labels = new ArrayList<label>(2 * userParam.nbclients); // initial size at least larger than nb clients
		boolean[] cust = new boolean[userParam.nbclients + 2];
		cust[0] = true;
		for (i = 1; i < userParam.nbclients + 2; i++) cust[i] = false;
		labels.add(new label(0, -1, 0.0, 0, 0, false, cust));    // first label: start from depot (client 0)
		U.add(0);

		// for each city, an array with the index of the corresponding labels (for dominance)
		checkDom = new int[userParam.nbclients + 2];
		ArrayList<Integer>[] city2labels = new ArrayList[userParam.nbclients + 2];
		for (i = 0; i < userParam.nbclients + 2; i++) {
			city2labels[i] = new ArrayList<Integer>();
			checkDom[i] = 0;  // index of the first label in city2labels that needs to be checked for dominance (last labels added)
		}
		city2labels[0].add(0);

		nbsol = 0;
		maxSol = 2 * nbRoute;
		while ((U.size() > 0) && (nbsol < maxSol)) {
			// second term if we want to limit to the first solutions encountered to speed up the SPPRC (perhaps not the BP)
			// remark: we'll keep only nbroute, but we compute 2xnbroute!  It makes a huge difference=>we'll keep the most negative ones
			// this is something to analyze further!  how many solutions to keep and which ones?
			// process one label => get the index AND remove it from U
			currentidx = U.pollFirst();
			current = labels.get(currentidx);

			// check for dominance
			// code not fully optimized: 
			int l1, l2;
			boolean pathdom;
			label la1, la2;
			ArrayList<Integer> cleaning = new ArrayList<Integer>();
			for (i = checkDom[current.city]; i < city2labels[current.city].size(); i++) {  // check for dominance between the labels added since the last time we came here with this city and all the other ones
				for (j = 0; j < i; j++) {
					l1 = city2labels[current.city].get(i);
					l2 = city2labels[current.city].get(j);
					la1 = labels.get(l1);
					la2 = labels.get(l2);
					if (!(la1.dominated || la2.dominated)) {  // could happen since we clean 'city2labels' thanks to 'cleaning' only after the double loop
						pathdom = true;
						for (int k = 1; pathdom && (k < userParam.nbclients + 2); k++)
							pathdom = (!la1.vertexVisited[k] || la2.vertexVisited[k]);
						if (pathdom && (la1.cost <= la2.cost) && (la1.tTime <= la2.tTime) && (la1.demand <= la2.demand)) {
							labels.get(l2).dominated = true;
							U.remove((Integer) l2);
							cleaning.add(l2);
							pathdom = false;
							//System.out.print(" ###Remove"+l2);
						}
						pathdom = true;
						for (int k = 1; pathdom && (k < userParam.nbclients + 2); k++)
							pathdom = (!la2.vertexVisited[k] || la1.vertexVisited[k]);

						if (pathdom && (la2.cost <= la1.cost) && (la2.tTime <= la1.tTime) && (la2.demand <= la1.demand)) {
							labels.get(l1).dominated = true;
							U.remove(l1);
							cleaning.add(l1);
							//System.out.print(" ###Remove"+l1);
							j = city2labels[current.city].size();
						}
					}
				}
			}

			for (Integer c : cleaning)
				city2labels[current.city].remove((Integer) c);   // a little bit confusing but ok since c is an Integer and not an int!

			cleaning = null;
			checkDom[current.city] = city2labels[current.city].size();  // update checkDom: all labels currently in city2labels were checked for dom.

			// expand REF
			if (!current.dominated) {
				//System.out.println("Label "+current.city+" "+current.indexPrevLabel+" "+current.cost+" "+current.ttime+" "+current.dominated);
				if (current.city == userParam.nbclients + 1) { // shortest path candidate to the depot!
					if (current.cost < -1e-7) {                // SP candidate for the column generation
						P.add(currentidx);
						nbsol = 0;
						for (Integer labi : P) {
							label s = labels.get(labi);
							if (!s.dominated)
								nbsol++;
						}
					}
				} else {  // if not the depot, we can consider extensions of the path
					for (i = 0; i < userParam.nbclients + 2; i++) {
						if ((!current.vertexVisited[i]) && (userParam.dist[current.city][i] < userParam.verybig - 1e-6)) {  // don't go back to a vertex already visited or along a forbidden edge
							// ttime
							tt = (float) (current.tTime + userParam.ttime[current.city][i] + userParam.s[current.city]);
							if (tt < userParam.a[i])
								tt = userParam.a[i];
							// demand
							d = current.demand + userParam.d[i];
							//System.out.println("  -- "+i+" d:"+d+" t:"+tt);

							// is feasible?
							if ((tt <= userParam.b[i]) && (d <= userParam.capacity)) {
								idx = labels.size();
								boolean[] newcust = new boolean[userParam.nbclients + 2];
								System.arraycopy(current.vertexVisited, 0, newcust, 0, userParam.nbclients + 2);
								newcust[i] = true;
								//speedup: third technique - Feillet 2004 as mentioned in Laporte's paper
								for (j = 1; j <= userParam.nbclients; j++)
									if (!newcust[j]) {
										tt2 = (float) (tt + userParam.ttime[i][j] + userParam.s[i]);
										d2 = d + userParam.d[j];
										if ((tt2 > userParam.b[j]) || (d2 > userParam.capacity))
											newcust[j] = true;  // useless to visit this client
									}

								labels.add(new label(i, currentidx, current.cost + userParam.cost[current.city][i], tt, d, false, newcust));    // first label: start from depot (client 0)
								if (!U.add((Integer) idx)) {
									// only happens if there exists already a label at this vertex with the same cost, time and demand and visiting the same cities before
									// It can happen with some paths where the order of the cities is permuted
									labels.get(idx).dominated = true; // => we can forget this label and keep only the other one
								} else
									city2labels[i].add(idx);

							}
						}
					}
				}
			}
		}
		// clean
		checkDom = null;

		// filtering: find the path from depot to the destination
		Integer lab;
		i = 0;
		while ((i < nbRoute) && ((lab = P.pollFirst()) != null)) {
			label s = labels.get(lab);
			if (!s.dominated) {
				if (/*(i < nbroute / 2) ||*/ (s.cost < -1e-4)) {
					// System.out.println(s.cost);
					//        	if(s.cost > 0) {
					//        		System.out.println("warning >>>>>>>>>>>>>>>>>>>>");
					//        	}
					route newroute = new route();
					newroute.setcost(s.cost);
					newroute.addcity(s.city);
					int path = s.indexPrevLabel;
					while (path >= 0) {
						newroute.addcity(labels.get(path).city);
						path = labels.get(path).indexPrevLabel;
					}
					newroute.switchpath();
					routes.add(newroute);
					i++;
				}
			}

		}
	}
}
