package algo;

import java.io.IOException;
import java.util.ArrayList;

public class Main {

	public static void main(String[] args) throws IOException {
		branchandbound bp = new branchandbound();
		paramsVRP instance = new paramsVRP();
		instance.initParams("dataset/C101.TXT");
		ArrayList<route> initRoutes = new ArrayList<route>();
		ArrayList<route> bestRoutes = new ArrayList<route>();

		bp.BBNode(instance, initRoutes, null, bestRoutes, 0);
		double optCost = 0;
		System.out.println();
		System.out.println("solution >>>");
		for (route bestRoute : bestRoutes) {
			System.out.println(bestRoute.path);
			optCost += bestRoute.cost;
		}

		System.out.println("\nbest Cost = " + optCost);
	}

}
