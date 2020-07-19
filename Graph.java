package graph;

import java.util.*;
public class Graph {
	private int V;
	private LinkedList<Integer> adj[];
	Graph(int v){
		V = v;
		adj = new LinkedList[v];
		for(int i=0;i<v;i++) {
			adj[i] = new LinkedList<Integer>();
		}
	}
	 void addEdge(int v,int u) {
		adj[v].add(u);
	}
	void BFS(int s) {
		Queue<Integer> queue = new LinkedList<>();
		boolean isVisited[] = new boolean[V];
		queue.add(s);
		isVisited[s] = true;
		while(!queue.isEmpty()) {
			s = queue.poll();
			System.out.print(s + " ");
			Iterator<Integer> i = adj[s].listIterator();
			while(i.hasNext()) {
				int n = i.next();
				if(!isVisited[n]) {
					isVisited[n] = true;
					queue.add(n);
				}
			}
		}
	}
	void DFSUtil(int s, boolean[] visited) {
		
		if(visited[s]) {
			return;
		}
			System.out.print(s + " ");
			visited[s] = true;
			Iterator<Integer> i = adj[s].listIterator();
			while(i.hasNext()) {
				int n = i.next();
				DFSUtil(n,visited);
			}
	}
	void DFS(int s) {
		boolean visited[] = new boolean[V];
		DFSUtil(s,visited);
	}
	
	// Shortest Path (Dijkstra Algorithm)
	boolean shortestPathBFS(int[] dist, int pred[], int s, int d) {
		Queue<Integer> queue = new LinkedList<>();
		boolean visited[] = new boolean[V];
		for(int i=0;i<V;i++) {
			dist[i] = Integer.MAX_VALUE;
			pred[i] = -1;
		}
		dist[s] = 0;
		visited[s] = true;
		queue.add(s);
		while(!queue.isEmpty()) {
			int u = queue.remove();
			Iterator<Integer> i = adj[u].listIterator();
			while(i.hasNext()) {
				int n = i.next();
				if(!visited[n]) {
					visited[n] = true;
					dist[n] = dist[u]+1;
					pred[n] = u;
					queue.add(n);
				}
				if(n == d) {
					return true;
				}
			}
		}
		return false;
	}
	
	void shortestPath(int s, int d) {
		int pred[] = new int[V];
		int dist[] = new int[V];
		if(shortestPathBFS(dist,pred,s,d) == false) {
			System.out.println("No Path Available");
			return;
		}
		System.out.println("Shortest Distance = "+dist[d]);
		LinkedList<Integer> path = new LinkedList<>();
		int crawl = d;
		path.add(crawl);
		while(pred[crawl] != -1) {
			crawl = pred[crawl];
			path.add(crawl);
		}
		for(int i=path.size()-1;i>=0;i--) {
			System.out.print(path.get(i)+" ");
		}
	}
	
	//Kosaraju's Algorithm
	Graph getTranspose() {
		Graph gr = new Graph(V);
		for(int j=0;j<V;j++) {
			Iterator<Integer> i = adj[j].listIterator();
			while(i.hasNext()) {
				int n = i.next();
				gr.adj[n].add(j);
			}
		}
		return gr;
	}
	
	void fillOrder(boolean[] visited, int s, Stack<Integer> stack) {
		visited[s] = true;
		Iterator<Integer> i = adj[s].listIterator();
		while(i.hasNext()) {
			int n = i.next();
			if(!visited[n]) {
				fillOrder(visited,n,stack);
			}
		}
		stack.push(s);
	}
	
	void printSCC() {
		boolean visited1[] = new boolean[V];
		Stack<Integer> stack = new Stack<>();
		for(int j=0;j<V;j++) {
			if(!visited1[j]) {
				fillOrder(visited1,j,stack);
			}
		}
		Graph gr = getTranspose();
		boolean visited2[] = new boolean[V];
		while(!stack.isEmpty()) {
			int v = stack.pop();
			if(!visited2[v]) {
				gr.DFSUtil(v, visited2);
				System.out.println(" ");
			}
		}
	}
	
	// Finding Mother vertex
	int findMother() {
		boolean visited[] = new boolean[V];
		int v = 0;
		for(int j=0;j<V;j++) {
			if(!visited[j]) {
				DFSUtil(j,visited);
				v = j;
			}
		}
		for(int i=0;i<V;i++) {
			visited[i] = false;
		}
		DFSUtil(v,visited);
		for(int i=0;i<V;i++) {
			if(visited[i] == false) {
				return -1;
			}
		}
		return v;
	}
	
	public static void main(String args[]) {
		Graph g = new Graph(5);
		g.addEdge(1, 0); 
        g.addEdge(0, 2); 
        g.addEdge(2, 1); 
        g.addEdge(0, 3); 
        g.addEdge(3, 4);
        //g.BFS(2);
        //g.DFS(2);
        //g.shortestPath(0, 2);
        //g.printSCC();
        System.out.println(g.findMother());
	}
}
