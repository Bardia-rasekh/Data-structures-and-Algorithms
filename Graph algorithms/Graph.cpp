#include "Header.h"

int main()
{
	graph obj;


	obj.add_vertex("ariya",9823541);
	obj.add_vertex("bardiya",2194847);
	obj.add_vertex("ali", 1948573);
	obj.add_vertex("mohsen", 3719046);
	obj.add_vertex("bruce", 3905765);
	obj.add_vertex("alex", 3923765);
	obj.add_vertex("edward", 1095765);
	obj.add_vertex("ehsan", 1092734);
	obj.add_vertex("masoud", 1395680);
	obj.add_vertex("jack", 9435163);
	obj.add_vertex("kevin", 1465912);

	obj.add_edge(0, 1, 4);
	obj.add_edge(0, 2, 2);
	obj.add_edge(0, 3, 3);
	obj.add_edge(0, 4, 6);
	obj.add_edge(1, 2, 6);
	obj.add_edge(3, 4, 4);
	obj.add_edge(1, 5, 3);
	obj.add_edge(2, 3, 3);
	obj.add_edge(2, 4, 6);
	obj.add_edge(2, 6, 6);
	obj.add_edge(3, 4, 4);
	obj.add_edge(3, 6, 3);
	
	//obj.display_vertesies();
	//obj.display_edges();
	

	//obj.bfs_travers(3);
	//obj.dfs_travers(3);
	

	//obj.sssp_bellman_ford(0);
	//obj.sssp_dijkstra(0);


	//obj.topological_sort_kahn();
	//obj.topological_sort_dfs();
	

	//obj.articulation_points_Tarjan();


	//obj.mst_kruskal();
	//obj.mst_prime(0);
	
	
	//obj.apsp_floyd_warshall();


	//obj.maxFlow_fordFulkerson(4, 5);


	//obj.outer_coloring_decision(2);
	//obj.outer_coloring_permutation(2);
	//obj.coloring_optimization();


	//obj.outer_hamiltonian_cycle();


	//if (obj.outer_is_eulerian()) obj.outer_euler_cycle();


	return 0;
}
