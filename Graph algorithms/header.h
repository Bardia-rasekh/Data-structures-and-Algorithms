#include <bits/stdc++.h>

using namespace std;


class graph
{
private:
	vector<pair<string, int>> vertesies;
	int vertesies_count = 0;
	vector<vector<pair<int, int>>> edges;
	vector<tuple<int, int, int>> all_edges;
	map<pair<int, int>, int> bellman_ford_edges;
	vector<vector<int>> ford_fulkerson_edges;
	vector<vector<int>> eular_edges;
	void inner_topological_sort_dfs(vector<bool>& visited, int i, list<int>& result);
	void inner_articulation_points_Tarjan(vector<int>& disc, vector<int>& low, vector<bool>& visited, vector<int>& parent, set<int>& result, int v, int& time);
	void make_set(vector<int>& root);
	int find_root(vector<int>& root, int x);
	void union_set(vector<int>& root, vector<int>& rank, int x, int y);
	static bool sort_compare(tuple<int, int, int>& a, tuple<int, int, int>& b);
	int bfs_minCap(vector<int>& parent, vector<vector<int>>& residual_graph, int source = -1, int sink = -1);
	bool is_valid(vector<int>& color, int c = -1, int v = -1);
	bool inner_coloring_decision(vector<int>& color, int color_count = -1, int v = -1);
	void inner_coloring_permutation(vector<int>& color, int v = -1, int color_count = -1);
	bool inner_hamiltonian_cycle(vector<bool>& visited, vector<int>& path, int v = -1);
	void remove_edge(int v = -1, int av = -1);
	int dfs_count(vector<bool>& visited, int v = -1);
	bool isValid_nextEdge(int v = -1, int av = -1);
	int odd_degree_vertex();
	void inner_euler_cycle(int v = -1);
	void dfs(vector<bool>& visited, int v = -1);
	bool is_connected();
	int inner_is_eulerian();
public:
	void add_vertex(string name = "", int id = -1);
	void add_edge(int v_1 = -1, int v_2 = -1, int weight = -1);
	void display_vertesies();
	void display_edges();
	void bfs_travers(int start = -1);
	void dfs_travers(int start = -1);
	void sssp_dijkstra(int start = -1);
	void sssp_bellman_ford(int start = -1);
	void topological_sort_kahn();
	void topological_sort_dfs();
	void articulation_points_Tarjan();
	void mst_kruskal();
	void mst_prime(int start = -1);
	void apsp_floyd_warshall();
	void maxFlow_fordFulkerson(int source = -1, int sink = -1);
	void outer_coloring_decision(int color_count = -1);
	void outer_coloring_permutation(int color_count = -1);
	void coloring_optimization();
	void outer_hamiltonian_cycle();
	void outer_euler_cycle();
	bool outer_is_eulerian();
};

void graph::add_vertex(string name,int id)
{
	vertesies.emplace_back(make_pair(name,id));

	vector <pair<int, int>> temp_1;
	edges.emplace_back(temp_1);

	vector<int> temp_2;
	ford_fulkerson_edges.emplace_back(temp_2);
	
	for (int i = 0; i < vertesies_count; i++) ford_fulkerson_edges[i].emplace_back(0);
	for (int i = 0; i <= vertesies_count; i++) ford_fulkerson_edges[vertesies_count].emplace_back(0);

	eular_edges.emplace_back(temp_2);

	vertesies_count++;
}

void graph::add_edge(int v_1, int v_2, int weight)
{
	edges[v_1].emplace_back(make_pair(v_2, weight));
	edges[v_2].emplace_back(make_pair(v_1, weight));
	all_edges.emplace_back(make_tuple(v_1, v_2, weight));
	all_edges.emplace_back(make_tuple(v_2, v_1, weight));
	bellman_ford_edges[make_pair(v_1, v_2)] = weight;
	bellman_ford_edges[make_pair(v_2, v_1)] = weight;
	ford_fulkerson_edges[v_1][v_2] = weight;
	ford_fulkerson_edges[v_2][v_1] = weight;
	eular_edges[v_1].emplace_back(v_2);
	eular_edges[v_2].emplace_back(v_1);
}

void graph::display_vertesies()
{
	cout << "display_vertesies : " << '\n';
	for (auto i = vertesies.begin(); i != vertesies.end(); i++) cout << i->first << " : " << i->second << endl;
	cout << '\n' << endl;
}

void graph::display_edges()
{
	cout << "display_edges : " << '\n';
	for (auto i = edges.begin(); i != edges.end(); i++)
	{
		for (auto j = i->begin(); j != i->end(); j++) cout << "->(" << j->first << " , " << j->second << ") ";
		cout << '\n';
	}
	cout << '\n' << endl;
}

void graph::bfs_travers(int start)
{
	queue<int> q;
	vector<bool> visited(vertesies_count, false);
	q.push(start);
	visited[start] = true;

	cout << "bfs_travers : " << '\n';

	while (!q.empty())
	{
		int temp = q.front();
		q.pop();
		cout << temp << " : " << vertesies[temp].first << " , " << vertesies[temp].second << endl;

		for (auto i = edges[temp].begin(); i != edges[temp].end(); i++)
		{
			if (!visited[i->first])
			{
				q.push(i->first);
				visited[i->first] = true;
			}
		}
	}
	cout << '\n' << endl;
}

void graph::dfs_travers(int start)
{
	stack<int> st;
	vector<bool> visited(vertesies_count, false);
	st.push(start);
	visited[start] = true;

	cout << "dfs_travers : " << '\n';

	while (!st.empty())
	{
		int temp = st.top();
		st.pop();
		cout << temp << " : " << vertesies[temp].first << " , " << vertesies[temp].second << endl;

		for (auto i = edges[temp].begin(); i != edges[temp].end(); i++)
		{
			if (!visited[i->first])
			{
				st.push(i->first);
				visited[i->first] = true;
			}
		}
	}
	cout << '\n' << endl;
}

void graph::sssp_dijkstra(int start)
{
	vector<int> dist(vertesies_count, INT_MAX);
	priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;

	pq.push(make_pair(0, start));
	dist[start] = 0;

	while (!pq.empty())
	{
		int u = pq.top().second;
		pq.pop();

		for (int i = 0; i < edges[u].size(); i++)
		{
			int v = edges[u][i].first;
			int weight = edges[u][i].second;

			if (dist[v] > dist[u] + weight)
			{
				dist[v] = dist[u] + weight;
				pq.push(make_pair(dist[v], v));
			}
		}
	}

	cout << "sssp_dijkstra : " << '\n';
	for (int i = 0; i < dist.size(); i++) cout << i << " : " << dist[i] << '\n';
	cout << '\n' << endl;
}

void graph::sssp_bellman_ford(int start)
{
	vector<int> dist(vertesies_count, INT_MAX);
	dist[start] = 0;
	bool same = true;

	for (int i = 1; i < vertesies_count; i++)
	{
		same = true;
		for (const auto& j : bellman_ford_edges)
		{
			int u = j.first.first;
			int v = j.first.second;
			int weight = j.second;
			if (dist[u] != INT_MAX && dist[v] > dist[u] + weight)
			{
				dist[v] = dist[u] + weight;
				same = false;
			}
		}
		if (same) break;
	}

	if (!same)
	{
		for (const auto& j : bellman_ford_edges)
		{
			int u = j.first.first;
			int v = j.first.second;
			int weight = j.second;
			if (dist[u] != INT_MAX && dist[v] > dist[u] + weight)
			{
				cout << "Negative weight cycle detected." << '\n' << '\n';
				return;
			}
		}
	}

	cout << "sssp_bellman_ford : " << '\n';
	for (int i = 0; i < vertesies_count; i++) cout << i << " : " << dist[i] << endl;
	cout << '\n' << '\n';
}

void graph::topological_sort_kahn()
{
	queue<int> q;
	vector<int> indegree(vertesies_count, 0);
	vector<int> result;

	for (int i = 0; i < edges.size(); i++) for (int j = 0; j < edges[i].size(); j++) indegree[edges[i][j].first]++;

	for (int i = 0; i < indegree.size(); i++) if (indegree[i] == 0) q.push(i);

	while (!q.empty())
	{
		int temp = q.front();
		q.pop();
		result.emplace_back(temp);

		for (auto i = edges[temp].begin(); i != edges[temp].end(); i++)
		{
			indegree[i->first]--;
			if (!indegree[i->first]) q.push(i->first);
		}
	}

	cout << "topological_sort_kahn : " << '\n';
	for (auto i = result.begin(); i != result.end(); i++) cout << *i << " ";
	cout << '\n' << endl;
}

void graph::topological_sort_dfs()
{
	vector<bool> visited(vertesies_count, false);
	list<int> result;

	for (int i = 0; i < vertesies_count; i++) if (!visited[i]) inner_topological_sort_dfs(visited, i, result);

	cout << "topological_sort_dfs : " << '\n';
	for (auto i = result.begin(); i != result.end(); i++) cout << *i << " ";
	cout << '\n' << endl;
}

void graph::inner_topological_sort_dfs(vector<bool>& visited, int i, list<int>& result)
{
	visited[i] = true;
	for (auto j = edges[i].begin(); j != edges[i].end(); j++) if (!visited[j->first]) inner_topological_sort_dfs(visited, j->first, result);
	result.push_front(i);
}

void graph::articulation_points_Tarjan()
{
	vector<bool> visited(vertesies_count, false);
	vector<int> low(vertesies_count, 0);
	vector<int> disc(vertesies_count, 0);
	set<int> result;
	vector<int> parent(vertesies_count, -1);
	int time = 0;
	inner_articulation_points_Tarjan(disc, low, visited, parent, result, 0, time);

	cout << "articulation_points_Tarjan : " << '\n';
	for (auto i = result.begin(); i != result.end(); i++) cout << *i << " -> (" << vertesies[*i].first << " , " << vertesies[*i].second << ")" << endl;
	cout << '\n' << endl;
}

void graph::inner_articulation_points_Tarjan(vector<int>& disc, vector<int>& low, vector<bool>& visited, vector<int>& parent, set<int>& result, int v, int& time)
{
	visited[v] = true;
	disc[v] = low[v] = ++time;
	int child = 0;
	for (auto i = edges[v].begin(); i != edges[v].end(); i++)
	{
		int av = i->first;
		if (!visited[av])
		{
			child++;
			parent[av] = v;
			inner_articulation_points_Tarjan(disc, low, visited, parent, result, av, time);
			low[v] = min(low[v], low[av]);
			if (parent[v] == -1 && child >= 2) result.emplace(v);
			else if (parent[v] != -1 && disc[v] <= low[av]) result.emplace(v);
		}
		else if (parent[v] != av) low[v] = min(disc[av], low[v]);
	}

}

void graph::make_set(vector<int>& root)
{
	for (int i = 0; i < vertesies_count; i++) root[i] = i;
}

int graph::find_root(vector<int>& root,int x)
{
	if (x != root[x]) root[x] = find_root(root, root[x]);
	return root[x];
}

void graph::union_set(vector<int>& root,vector<int>& rank,int x,int y)
{
	int rootX = find_root(root, x);
	int rootY = find_root(root, y);
	if (rootX == rootY) return;
	if (rank[rootX] > rank[rootY]) root[rootY] = rootX;
	else
	{
		root[rootX] = rootY;
		if (rank[rootX] == rank[rootY]) rank[rootY]++;
	}
}

bool graph::sort_compare(tuple<int, int, int>& a, tuple<int, int, int>& b)
{
	return (get<2>(a) < get<2>(b));
}

void graph::mst_kruskal()
{
	vector<int> root(vertesies_count);
	vector<int> rank(vertesies_count, 0);
	vector<tuple<int, int, int>> result;
	int rootX, rootY;

	make_set(root);
	sort(all_edges.begin(), all_edges.end(),sort_compare);
	
	for (auto i = all_edges.begin(); i != all_edges.end(); i++)
	{
		rootX = get<0>(*i);
		rootY = get<1>(*i);
		if (find_root(root, rootX) != find_root(root, rootY))
		{
			result.emplace_back(*i);
			union_set(root, rank, rootX, rootY);
		}
	}

	cout << "mst_kruskal : " << '\n';
	for (auto i = result.begin(); i != result.end(); i++) cout << "(" << get<0>(*i) << " , " << get<1>(*i) << " , " << get<2>(*i) << ")  ";
	cout << '\n' << endl;
}

void graph::mst_prime(int start)
{
	vector<bool> in_mst(vertesies_count, false);
	vector<int> parent(vertesies_count, -1);
	vector<int> min_weight(vertesies_count, INT_MAX);
	priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;
	vector<tuple<int, int, int>> result;
	int v, av, weight;

	min_weight[start] = 0;
	pq.push(make_pair(0, start));

	while (!pq.empty())
	{
		v = pq.top().second;
		pq.pop();

		if (!in_mst[v])
		{
			if (parent[v] != -1) result.emplace_back(make_tuple(parent[v], v, min_weight[v]));
			in_mst[v] = true;

			for (auto i : edges[v])
			{
				av = i.first;
				weight = i.second;
				if (!in_mst[av] && min_weight[av] > weight)
				{
					parent[av] = v;
					min_weight[av] = weight;
					pq.push(make_pair(weight, av));
				}
			}
		}
	}

	cout << "mst_prime : " << '\n';
	for (auto i : result) cout << "(" << get<0>(i) << " , " << get<1>(i) << " , " << get<2>(i) << ")  ";
	cout << '\n' << endl;
}

void graph::apsp_floyd_warshall()
{
	vector<vector<int>> dist(vertesies_count, vector<int>(vertesies_count, INT_MAX));

	for (int i = 0; i < vertesies_count; i++) dist[i][i] = 0;
	for (auto i : all_edges) dist[get<0>(i)][get<1>(i)] = get<2>(i);
	for (int k = 0; k < vertesies_count; k++) for (int i = 0; i < vertesies_count; i++) for (int j = 0; j < vertesies_count; j++)
	{
		if (dist[i][k] != INT_MAX && dist[k][j] != INT_MAX) dist[i][j] = min(dist[i][j], dist[i][k] + dist[k][j]);
	}

	cout << "apsp_floyd_warshall : " << '\n';
	for (int i = 0; i < vertesies_count; i++) for (int j = 0; j < vertesies_count; j++) cout << i << "," << j << " : " << dist[i][j] << '\n';
	cout << '\n' << endl;
}

int graph::bfs_minCap(vector<int>& parent, vector<vector<int>>& residual_graph, int source , int sink)
{
	fill(parent.begin(), parent.end(), -1);
	parent[source] = -2;
	queue<pair<int, int>> q;
	q.push({ source,INT_MAX });

	while (!q.empty())
	{
		int u = q.front().first;
		int capacity = q.front().second;
		q.pop();

		for (int av = 0; av < vertesies_count; av++)
		{
			if (u != av && parent[av] == -1 && residual_graph[u][av] != 0)
			{
				parent[av] = u;
				int min_cap = min(capacity, residual_graph[u][av]);
				if (av == sink) return min_cap;
				q.push({ av,min_cap });
			}
		}
	}
	return 0;
}

void graph::maxFlow_fordFulkerson(int source, int sink)
{
	vector<int> parent(vertesies_count, -1);
	vector<vector<int>> residual_graph = ford_fulkerson_edges;
	int max_flow = 0, min_cap = 0;
	while (min_cap = bfs_minCap(parent, residual_graph, source, sink))
	{
		max_flow += min_cap;
		int u = sink;
		while (u != source)
		{
			int v = parent[u];
			residual_graph[u][v] += min_cap;
			residual_graph[v][u] -= min_cap;
			u = v;
		}
	}

	cout << "ford fulkerson maximum flow result : " << max_flow << '\n' << '\n' << endl;
}

bool graph::is_valid(vector<int>& color,int c,int v)
{
	for (auto u : edges[v]) if (color[u.first] == c) return false;
	return true;
}

bool graph::inner_coloring_decision(vector<int>& color,int color_count,int v)
{
	if (v == vertesies_count) return true;
	for (int c = 0; c < color_count; c++)
	{
		if (is_valid(color,c, v))
		{
			color[v] = c;
			if (inner_coloring_decision(color, color_count, v + 1)) return true;
			color[v] = 0;
		}
	}
	return false;
}

void graph::outer_coloring_decision(int color_count)
{
	vector<int> color(vertesies_count, -1);
	cout << "Coloring decision : " << '\n';
	if (inner_coloring_decision(color, color_count, 0)) cout << "Possible.";
	else cout << "Not Possible.";
	cout << '\n' << '\n' << endl;
}

void graph::inner_coloring_permutation(vector<int>& color,int v,int color_count)
{
	if (v == vertesies_count)
	{
		for (auto i : color) cout << i << " ";
		cout << '\n';
		return;
	}
	for (int c = 0; c < color_count; c++)
	{
		if (is_valid(color, c, v))
		{
			color[v] = c;
			inner_coloring_permutation(color, v + 1, color_count);
			color[v] = 0;
		}
	}
}

void graph::outer_coloring_permutation(int color_count)
{
	vector<int> color(vertesies_count, -1);
	cout << "Coloring permutation : " << '\n';
	inner_coloring_permutation(color, 0, color_count);
	cout << '\n' << endl;
}

void graph::coloring_optimization()
{
	vector<int> color(vertesies_count, -1);
	color[0] = 0;

	vector<bool> already_assigned(vertesies_count);

	for (int v = 1; v < vertesies_count; v++)
	{
		fill(already_assigned.begin(), already_assigned.end(), false);

		for (auto av : edges[v]) if (color[av.first] != -1) already_assigned[color[av.first]] = true;

		int c;
		for (c = 0; c < vertesies_count; c++) if (!already_assigned[c]) break;

		color[v] = c;
	}

	cout << "Coloring optimization :" << '\n';
	for (auto i : color) cout << i << " ";
	cout << '\n' << "Minimum color required : " << *max_element(color.begin(), color.end()) + 1;
	cout << '\n' << '\n' << endl;
}

bool graph::inner_hamiltonian_cycle(vector<bool>& visited, vector<int>& path, int v)
{
	if (path.size() == vertesies_count) return true;
	for (auto av : edges[v])
	{
		if (!visited[av.first])
		{
			path.emplace_back(av.first);
			visited[av.first] = true;
			if (inner_hamiltonian_cycle(visited, path, av.first)) return true;
			path.pop_back();
			visited[av.first] = false;
		}
	}
	return false;
}

void graph::outer_hamiltonian_cycle()
{
	vector<bool> visited(vertesies_count, false);
	vector<int> path;

	path.emplace_back(0);
	visited[0] = true;

	if (inner_hamiltonian_cycle(visited, path, 0))
	{
		cout << "Hamiltonian cycle exists : " << '\n';
		for (auto i : path) cout << i << " ";
		cout << path[0] << '\n' << endl;
	}
	else cout << "Hamiltonian cycle does not exists." << '\n' << '\n' << endl;
}

void graph::remove_edge(int v, int av)
{
	auto it = find_if(eular_edges[v].begin(), eular_edges[v].end(), [&](int const& ref) {return ref == av; });
	*it = -1;
	it = find_if(eular_edges[av].begin(), eular_edges[av].end(), [&](int const& ref) {return ref == v; });
	*it = -1;
}

int graph::dfs_count(vector<bool>& visited, int v)
{
	visited[v] = true;
	int count = 1;

	for (auto av : eular_edges[v]) if (av != -1 && !visited[av]) count += dfs_count(visited, av);

	return count;
}

bool graph::isValid_nextEdge(int v, int av)
{
	int count = 0;
	for (auto i : eular_edges[v]) if (i != -1) count++;
	if (!count) return false;
	if (count == 1) return true;

	vector<bool> visited(vertesies_count, false);
	int count1 = dfs_count(visited, v);

	remove_edge(v, av);

	fill(visited.begin(), visited.end(), false);

	int count2 = dfs_count(visited, v);

	eular_edges[v].emplace_back(av);
	eular_edges[av].emplace_back(v);
	
	return (count1 <= count2);
}

int graph::odd_degree_vertex()
{
	for (int i = 0; i < vertesies_count; i++) if (eular_edges[i].size() % 2) return i;
	for (int i = 0; i < vertesies_count; i++) if (eular_edges[i].size()) return i;
}

void graph::inner_euler_cycle(int v)
{
	for (auto av : eular_edges[v])
	{
		if (av != -1 && isValid_nextEdge(v, av))
		{
			cout << v << "-" << av << " ";
			remove_edge(v, av);
			inner_euler_cycle(av);
		}
	}
}

void graph::outer_euler_cycle()
{
	inner_euler_cycle(odd_degree_vertex());
	cout << '\n' << '\n' << endl;
}

void graph::dfs(vector<bool>& visited, int v)
{
	visited[v] = true;
	for (auto av : eular_edges[v])
	{
		if (!visited[av]) dfs(visited, av);
	}
}

bool graph::is_connected()
{
	vector<bool> visited(vertesies_count, false);

	int i;
	for (i = 0; i < vertesies_count; i++) if (eular_edges[i].size()) break;
	
	if (i == vertesies_count) return true;

	dfs(visited, i);

	for (i = 0; i < vertesies_count; i++) if (!visited[i] && eular_edges[i].size() > 0) return false;

	return true;
}

int graph::inner_is_eulerian()
{
	if (!is_connected()) return 0;

	int odd = 0;
	for (int i = 0; i < vertesies_count; i++)
	{
		if (eular_edges[i].size() % 2) odd++;
	}

	if (odd > 2) return 0;
	
	return (!odd) ? 2 : 1;
}

bool graph::outer_is_eulerian()
{
	int temp = inner_is_eulerian();
	if (!temp) cout << "Graph is not Euerlian" << endl;
	else if(temp == 1) cout << "Graph has an Euler path" << endl;
	else cout << "Graph has an Euler circuit" << endl;

	return (temp);
}

