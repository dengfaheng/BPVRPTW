package algo;

import java.io.IOException;
import java.util.ArrayList;

public class Main {

	public static void main(String[] args) throws IOException {
		branchandbound bp = new branchandbound();
		paramsVRP instance = new paramsVRP();
		instance.initParams("dataset/C104.TXT");
		ArrayList<route> initRoutes = new ArrayList<route>();
		ArrayList<route> bestRoutes = new ArrayList<route>();
		
		bp.BBnode(instance, initRoutes, null, bestRoutes, 0);
		double optCost = 0;
		System.out.println();
		System.out.println("solution >>>");
		for(int i = 0; i < bestRoutes.size(); ++i) {
			System.out.println(bestRoutes.get(i).path);
			optCost+=bestRoutes.get(i).cost;
		}

		System.out.println("\nbest Cost = "+optCost);
	}

}
