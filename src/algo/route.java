package algo;

import java.util.ArrayList;

public class route implements Cloneable {
	public double cost, Q;
	// first resource: cost (e.g. distance or strict travel time)

	public ArrayList<Integer> path;

	public route() {
		this.path = new ArrayList<Integer>();
		this.cost = 0.0;
	}

	public route(int pathSize) {
		this.path = new ArrayList<Integer>(pathSize);
		this.cost = 0.0;
	}

	/*
	 * @update 2013. 6. 8
	 * @modify Geunho Kim
	 */
	// method for deep cloning
	public route clone() throws CloneNotSupportedException {
		route route = (route) super.clone();
		route.path = (ArrayList<Integer>) path.clone();
		return route;
	}

	public void removeCity(int city) {
		this.path.remove(Integer.valueOf(city));

	}

	public void addcity(int f_city, int city) {
		int index = this.path.indexOf(f_city);
		this.path.add(index + 1, city);
	}

	public void addcity(int city) {
		this.path.add(city);
	}

	public void setcost(double c) {
		this.cost = c;
	}

	public double getcost() {
		return this.cost;
	}

	public void setQ(double a) {
		this.Q = a;
	}

	public double getQ() {
		return this.Q;
	}

	public ArrayList<Integer> getpath() {
		return this.path;
	}

	public void switchpath() {
		Integer swap;
		int nb = path.size() / 2;
		for (int i = 0; i < nb; i++) {
			swap = path.get(i);
			path.set(i, path.get(path.size() - 1 - i));
			path.set(path.size() - 1 - i, swap);
		}
	}
}