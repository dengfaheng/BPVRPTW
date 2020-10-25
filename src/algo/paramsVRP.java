package algo;

// this class contains the inputs and methods to read the inputs
// for the Branch and Price CVRP with TW 
// ...I'm afraid that it is not pure OO code
// ...but it is not so bad

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;


public class paramsVRP {
	public int mvehic;
	public int nbclients;
	public int capacity;
	public double[][] cost; // for the SPPRC subProblem
	public double[][] distBase; // original distances for the Branch and Bound
	public double[][] dist; // distances that will be updated during the B&B before being used in the CG & SPPRC
	public double[][] ttime;
	public double[][] edges; // weight of each edge during branch and bound
	public double[] posx, posy, d, wval;
	public int[] a; // time windows: a=early, b=late, s=service
	public int[] b;
	public int[] s;
	public double verybig;
	public double speed;
	public double gap;
	public double maxlength;
	public boolean serviceInTW;
	String[] citieslab;

	public paramsVRP() {
		gap = 0.00000000001;
		serviceInTW = false;
		nbclients = 100;
		speed = 1;
		mvehic = 0;
		verybig = 1E10;
	}

	public void initParams(String inputPath) throws IOException {
		int i, j;

		try {
			/**
			 * @update 2013. 6. 12
			 * @modify Geunho Kim
			 *
			 *  for Hadoop distributed file system
			 */

			BufferedReader br = new BufferedReader(new FileReader(inputPath));

			String line = new String();

			// //////////////////////////
			// for local file system
			// BufferedReader br = new BufferedReader(new FileReader(inputPath));

			for (i = 0; i < 5; i++)
				line = br.readLine();

			String[] tokens = line.split("\\s+");
			mvehic = Integer.parseInt(tokens[1]);
			capacity = Integer.parseInt(tokens[2]);

			citieslab = new String[nbclients + 2];
			d = new double[nbclients + 2];
			a = new int[nbclients + 2];
			b = new int[nbclients + 2];
			s = new int[nbclients + 2];
			posx = new double[nbclients + 2];
			posy = new double[nbclients + 2];
			distBase = new double[nbclients + 2][nbclients + 2];
			cost = new double[nbclients + 2][nbclients + 2];
			dist = new double[nbclients + 2][nbclients + 2];
			ttime = new double[nbclients + 2][nbclients + 2];

			for (i = 0; i < 4; i++)
				line = br.readLine();

			for (i = 0; i < nbclients + 1; i++) {
				line = br.readLine();
				//System.out.println(line);
				tokens = line.split("\\s+");
				citieslab[i] = tokens[1]; // customer number
				posx[i] = Double.parseDouble(tokens[2]); // x coordinate
				posy[i] = Double.parseDouble(tokens[3]); // y coordinate
				d[i] = Double.parseDouble(tokens[4]); // demand
				a[i] = Integer.parseInt(tokens[5]); // ready time
				b[i] = Integer.parseInt(tokens[6]); // due time
				s[i] = Integer.parseInt(tokens[7]); // service
				// check if the service should be done before due time
				if (serviceInTW)
					b[i] -= s[i];
			}
			br.close();

			// second depot : copy of the first one for arrival
			citieslab[nbclients + 1] = citieslab[0];
			d[nbclients + 1] = 0.0;
			a[nbclients + 1] = a[0];
			b[nbclients + 1] = b[0];
			s[nbclients + 1] = 0;
			posx[nbclients + 1] = posx[0];
			posy[nbclients + 1] = posy[0];

			// ---- distances
			double max;
			maxlength = 0.0;
			for (i = 0; i < nbclients + 2; i++) {
				max = 0.0;
				for (j = 0; j < nbclients + 2; j++) {
					// dist[i][j]=Math.round(10*Math.sqrt((posx[i]-posx[j])*(posx[i]-posx[j])+(posy[i]-posy[j])*(posy[i]-posy[j])))/10.0;
					distBase[i][j] = ((int) (10 * Math
							.sqrt((posx[i] - posx[j]) * (posx[i] - posx[j])
									+ (posy[i] - posy[j]) * (posy[i] - posy[j])))) / 10.0;
					// truncate to get the same results as in Solomon
					if (max < distBase[i][j]) max = distBase[i][j];
				}
				maxlength += max; // a route with a length longer than this is not
				// possible (we need it to check the feasibility of
				// the Column Gen sol.
			}
			for (i = 0; i < nbclients + 2; i++) {
				distBase[i][0] = verybig;
				distBase[nbclients + 1][i] = verybig;
				distBase[i][i] = verybig;
			}
			/*
			 * for(i = 0; i < 20; i++)
			 *   distBase[10][i] = verybig;
			 * for(i = 21; i < nbclients+2; i++)
			 *   distBase[10][i] = verybig;
			 * for(i = 0; i < 10; i++)
			 *   distBase[i][20] = verybig;
			 * for(i = 11; i < nbclients+2; i++)
			 *   distBase[i][20] = verybig;
			 * distBase[20][10] = verybig;
			 */
			for (i = 0; i < nbclients + 2; i++)
				for (j = 0; j < nbclients + 2; j++) {
					dist[i][j] = distBase[i][j];
				}


			// ---- time
			for (i = 0; i < nbclients + 2; i++)
				for (j = 0; j < nbclients + 2; j++)
					ttime[i][j] = distBase[i][j] / speed;

			for (j = 0; j < nbclients + 2; j++) {
				cost[0][j] = dist[0][j];
				cost[j][nbclients + 1] = dist[j][nbclients + 1];
			}
			// cost for the other edges are defined during column generation

		} catch (IOException e) {
			System.err.println("Error: " + e);
		}

		wval = new double[nbclients + 2];
		for (i = 1; i < nbclients + 2; i++)
			wval[i] = 0.0;

		edges = new double[nbclients + 2][nbclients + 2];

	}
}