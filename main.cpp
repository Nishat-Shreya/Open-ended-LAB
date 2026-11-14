#include<bits/stdc++.h>
using namespace std;
#define ll long long
const ll INF = LLONG_MAX/2;

// false = every time step is an independent traffic scenario
// true  = load of previous time step is carried to the next step (cumulative)
const bool CARRY_LOAD = false;

// ---------- Graph (global) ----------
int n, m;
vector<int> eu, ev;
vector<ll> w;
vector<vector<pair<int,int> > > adj; // adj[u] = (v, edgeId)
set<pair<int,int> > usedEdge;

void addEdge(int u, int v, ll wt){
    int id = eu.size();
    eu.push_back(u);
    ev.push_back(v);
    w.push_back(wt);
    adj[u].push_back(make_pair(v, id));
    adj[v].push_back(make_pair(u, id));
    usedEdge.insert(make_pair(min(u, v), max(u, v)));
}

// ---------- 1) Graph generation (connected, undirected, weighted) ----------
// Random spanning tree guarantees connectivity, then extra random edges are added.
void generateGraph(int nodes, int edges, int maxW, mt19937 &rng){
    n = nodes;
    eu.clear(); ev.clear(); w.clear(); usedEdge.clear();
    adj.assign(n + 1, vector<pair<int,int> >());
    for(int i = 2; i <= n; i++){
        int j = rng() % (i - 1) + 1;
        addEdge(i, j, rng() % maxW + 1);
    }
    int tries = 0;
    while((int)eu.size() < edges && tries < 20 * edges){
        tries++;
        int u = rng() % n + 1;
        int v = rng() % n + 1;
        if(u == v) continue;
        if(usedEdge.count(make_pair(min(u, v), max(u, v)))) continue;
        addEdge(u, v, rng() % maxW + 1);
    }
    m = eu.size();
}

// ---------- Print generated road network ----------
void printGraph(){
    printf("\n--- Generated Road Network ---\n");
    printf("Vertices (intersections): %d, Edges (roads): %d\n", n, m);

    printf("Edge list (id : u -- v , weight):\n");
    for(int e = 0; e < m; e++){
        printf("   %d : %d -- %d , w=%lld\n", e, eu[e], ev[e], w[e]);
    }

    printf("Adjacency list (node : neighbor(weight) ...):\n");
    for(int u = 1; u <= n; u++){
        printf("   %d :", u);
        for(int i = 0; i < (int)adj[u].size(); i++){
            int v = adj[u][i].first;
            int id = adj[u][i].second;
            printf(" %d(%lld)", v, w[id]);
        }
        printf("\n");
    }
}

// ---------- Request generation (one time step) ----------
// request i = (rc[i] vehicles, source rs[i], destination rd[i])
void generateRequests(int R, mt19937 &rng, vector<int> &rc, vector<int> &rs, vector<int> &rd){
    rc.assign(R, 0); rs.assign(R, 0); rd.assign(R, 0);
    for(int i = 0; i < R; i++){
        rc[i] = rng() % 50 + 1;
        rs[i] = rng() % n + 1;
        do{ rd[i] = rng() % n + 1; }while(rd[i] == rs[i]);
    }
}

// ---------- 2) Dijkstra (returns edge-path from src to dst using cost[]) ----------
vector<int> dijkstraPath(vector<ll> &cost, int src, int dst){
    vector<ll> dist(n + 1, INF);
    vector<int> parEdge(n + 1, -1), parNode(n + 1, -1);
    priority_queue<pair<ll,int>, vector<pair<ll,int> >, greater<pair<ll,int> > > pq;
    dist[src] = 0;
    pq.push(make_pair(0LL, src));
    while(!pq.empty()){
        ll d = pq.top().first;
        int u = pq.top().second;
        pq.pop();
        if(d > dist[u]) continue;
        for(int i = 0; i < (int)adj[u].size(); i++){
            int v = adj[u][i].first;
            int id = adj[u][i].second;
            if(dist[u] + cost[id] < dist[v]){
                dist[v] = dist[u] + cost[id];
                parEdge[v] = id;
                parNode[v] = u;
                pq.push(make_pair(dist[v], v));
            }
        }
    }
    vector<int> path;
    int cur = dst;
    while(cur != src){
        path.push_back(parEdge[cur]);
        cur = parNode[cur];
    }
    reverse(path.begin(), path.end());
    return path;
}

// ---------- 3) Penalty-based candidate path generation ----------
// After each Dijkstra run, the cost of the edges of that path is doubled and Dijkstra
// is run again. This gives up to K alternative valid paths.
// LIMITATION: not guaranteed to be the exact K-shortest simple paths (like Yen's algorithm),
// and fewer than K distinct paths may be found.
// pathsOut[p] = edge list, lensOut[p] = real length. Path 0 = true shortest path.
void getCandidatePaths(int s, int d, int K,
                       vector<vector<int> > &pathsOut, vector<ll> &lensOut){
    pathsOut.clear();
    lensOut.clear();
    set<vector<int> > seen;
    vector<ll> cost = w;
    for(int k = 0; k < K; k++){
        vector<int> p = dijkstraPath(cost, s, d);
        if(!seen.count(p)){
            seen.insert(p);
            ll len = 0;
            for(int i = 0; i < (int)p.size(); i++) len += w[p[i]];
            pathsOut.push_back(p);
            lensOut.push_back(len);
        }
        for(int i = 0; i < (int)p.size(); i++) cost[p[i]] *= 2; // penalty
    }
}

// ---------- 4) Vehicle assignment ----------
// mode 0 = shortest path only
// mode 1 = greedy load-balancing (min max-load only, distance only breaks ties)
// mode 2 = proposed GMLR heuristic:
//          score = len(path)/shortestLen + beta * (max load on path after adding the vehicle)
//          Balances distance and congestion, but it is a HEURISTIC (not guaranteed optimal).
// startLoad = load already present on the roads before this time step
vector<ll> assignVehicles(vector<int> &rc, int R,
                          vector<vector<vector<int> > > &candEdges,
                          vector<vector<ll> > &candLen,
                          int mode, double beta,
                          vector<ll> startLoad,
                          vector<vector<int> > &distribution){
    vector<ll> load = startLoad;
    distribution.assign(R, vector<int>());
    for(int r = 0; r < R; r++){
        int P = candEdges[r].size();
        distribution[r].assign(P, 0);
        ll shortLen = candLen[r][0];
        for(int v = 0; v < rc[r]; v++){
            int best = 0;
            double bestScore = 1e18;
            for(int p = 0; p < P; p++){
                ll mx = 0;
                for(int i = 0; i < (int)candEdges[r][p].size(); i++){
                    mx = max(mx, load[candEdges[r][p][i]] + 1);
                }
                double score;
                if(mode == 0) score = p;
                else if(mode == 1) score = mx * 1000000.0 + candLen[r][p];
                else{
                    // 'volatile' stops the compiler from fusing (a + b*c) into one FMA instruction,
                    // so the result is identical on every machine (x86, ARM/Mac, ...)
                    double distPart = (double)candLen[r][p] / shortLen;
                    volatile double loadPart = beta * mx;
                    score = distPart + loadPart;
                }
                if(score < bestScore){
                    bestScore = score;
                    best = p;
                }
            }
            distribution[r][best]++;
            for(int i = 0; i < (int)candEdges[r][best].size(); i++){
                load[candEdges[r][best][i]]++;
            }
        }
    }
    return load;
}

// ---------- 5) Evaluation ----------
// totalVehicleDistance = distance of the CURRENT time step only:
// sum over roads of ((load - startLoad) * road length)
void evaluate(vector<ll> &load, vector<ll> &startLoad, ll &maxLoad, double &avgLoad,
              double &variance, ll &totalVehicleDistance){
    maxLoad = 0;
    totalVehicleDistance = 0;
    double sum = 0;
    for(int e = 0; e < m; e++){
        maxLoad = max(maxLoad, load[e]);
        totalVehicleDistance += (load[e] - startLoad[e]) * w[e];
        sum += load[e];
    }
    avgLoad = sum / m;
    double var = 0;
    for(int e = 0; e < m; e++){
        var += (load[e] - avgLoad) * (load[e] - avgLoad);
    }
    variance = var / m;
}

// ---------- 6) Main experiment (3 methods compared) ----------
void runExperiment(int nodes, int edges, int T, int R, bool detailed, int seed){
    mt19937 rng(seed);
    generateGraph(nodes, edges, 20, rng);
    int K = 4;
    double beta = 0.1;
    const char* names[3] = {"Shortest-Path", "Greedy-LB", "Proposed(GMLR)"};

    double totStat[3][4];
    memset(totStat, 0, sizeof(totStat));
    double timeMs[3] = {0, 0, 0};
    double candTimeMs = 0;
    vector<vector<ll> > carry(3, vector<ll>(m, 0));

    printf("\n===== Graph: V=%d, E=%d, Time steps=%d, Requests/step=%d =====\n", n, m, T, R);
    if(detailed) printGraph();

    for(int t = 1; t <= T; t++){
        vector<int> rc, rs, rd;
        generateRequests(R, rng, rc, rs, rd);

        vector<vector<vector<int> > > candEdges(R);
        vector<vector<ll> > candLen(R);
        auto cs = chrono::high_resolution_clock::now();
        for(int i = 0; i < R; i++){
            getCandidatePaths(rs[i], rd[i], K, candEdges[i], candLen[i]);
        }
        auto ce = chrono::high_resolution_clock::now();
        candTimeMs += chrono::duration<double, milli>(ce - cs).count();

        if(detailed) printf("\n--- Time step %d ---\n", t);

        for(int mode = 0; mode < 3; mode++){
            vector<vector<int> > dist;
            vector<ll> startLoad(m, 0);
            if(CARRY_LOAD) startLoad = carry[mode];

            auto st = chrono::high_resolution_clock::now();
            vector<ll> load = assignVehicles(rc, R, candEdges, candLen, mode, beta, startLoad, dist);
            auto en = chrono::high_resolution_clock::now();
            timeMs[mode] += chrono::duration<double, milli>(en - st).count();
            if(CARRY_LOAD) carry[mode] = load;

            ll maxLoad, totalVehicleDistance;
            double avgLoad, variance;
            evaluate(load, startLoad, maxLoad, avgLoad, variance, totalVehicleDistance);
            totStat[mode][0] += maxLoad;
            totStat[mode][1] += avgLoad;
            totStat[mode][2] += variance;
            totStat[mode][3] += totalVehicleDistance;

            if(detailed && mode == 2){
                for(int i = 0; i < R; i++){
                    printf("Request %d: (n=%d, s=%d, d=%d)\n", i + 1, rc[i], rs[i], rd[i]);
                    for(int p = 0; p < (int)candEdges[i].size(); p++){
                        if(dist[i][p] == 0) continue;
                        printf("   %d vehicles, len=%lld, path: %d", dist[i][p], candLen[i][p], rs[i]);
                        int cur = rs[i];
                        for(int k = 0; k < (int)candEdges[i][p].size(); k++){
                            int e = candEdges[i][p][k];
                            cur = (eu[e] == cur) ? ev[e] : eu[e];
                            printf(" -> %d", cur);
                        }
                        printf("\n");
                    }
                }
                printf("Road-wise load (edge u-v : load):\n");
                for(int e = 0; e < m; e++){
                    if(load[e] > 0) printf("   %d-%d : %lld\n", eu[e], ev[e], load[e]);
                }
            }
            if(detailed){
                printf("[%s] MaxLoad=%lld AvgLoad=%.2f Variance=%.2f TotalVehicleDistance=%lld\n",
                       names[mode], maxLoad, avgLoad, variance, totalVehicleDistance);
            }
        }
    }

    printf("\n>>> Average over %d time steps <<<\n", T);
    printf("%-16s %9s %9s %11s %14s %12s\n", "Method", "MaxLoad", "AvgLoad", "Variance", "TotalVehDist", "Assign(ms)");
    for(int mode = 0; mode < 3; mode++){
        printf("%-16s %9.2f %9.2f %11.2f %14.2f %12.3f\n", names[mode],
               totStat[mode][0] / T, totStat[mode][1] / T,
               totStat[mode][2] / T, totStat[mode][3] / T, timeMs[mode]);
    }
    printf("Candidate path generation time (all steps): %.3f ms\n", candTimeMs);
}

// ---------- 7) Trade-off analysis: vary beta ----------
// beta = 0 -> same as shortest path (only distance matters)
// larger beta -> more importance to load balancing
void tradeoffAnalysis(int nodes, int edges, int T, int R, int seed){
    mt19937 rng(seed);
    generateGraph(nodes, edges, 20, rng);
    int K = 4;
    double betas[7] = {0.0, 0.05, 0.1, 0.3, 0.5, 1.0, 2.0};

    printf("\n===== Trade-off Analysis (V=%d, E=%d, T=%d, R=%d) =====\n", n, m, T, R);
    printf("%-8s %9s %11s %14s\n", "Beta", "MaxLoad", "Variance", "TotalVehDist");

    for(int b = 0; b < 7; b++){
        mt19937 rng2(seed + 100); // same requests for every beta
        double sMax = 0, sVar = 0, sDist = 0;
        for(int t = 1; t <= T; t++){
            vector<int> rc, rs, rd;
            generateRequests(R, rng2, rc, rs, rd);
            vector<vector<vector<int> > > candEdges(R);
            vector<vector<ll> > candLen(R);
            for(int i = 0; i < R; i++){
                getCandidatePaths(rs[i], rd[i], K, candEdges[i], candLen[i]);
            }
            vector<vector<int> > dist;
            vector<ll> startLoad(m, 0);
            vector<ll> load = assignVehicles(rc, R, candEdges, candLen, 2, betas[b], startLoad, dist);
            ll maxLoad, tvd;
            double avgLoad, variance;
            evaluate(load, startLoad, maxLoad, avgLoad, variance, tvd);
            sMax += maxLoad;
            sVar += variance;
            sDist += tvd;
        }
        printf("%-8.2f %9.2f %11.2f %14.2f\n", betas[b], sMax / T, sVar / T, sDist / T);
    }
    printf("Observation: Beta controls the balance between congestion and distance.\n");
    printf("A higher beta gives more importance to road load balancing.\n");
    printf("It may reduce MaxLoad and Variance, but can increase travel distance.\n");
    printf("The exact result depends on the generated graph and requests.\n");
}

// ---------- 8) Complexity analysis ----------
void printComplexityAnalysis(){
    printf("\n===== Computational Complexity Analysis =====\n");
    printf("V = vertices, E = edges, R = requests, N = total vehicles,\n");
    printf("K = candidate paths per request, L = average edges in a path\n\n");
    printf("Graph generation              : O(V + E) expected\n");
    printf("Dijkstra (one run)            : O((V + E) log V)\n");
    printf("Candidate path generation     : O(R * K * (V + E) log V)\n");
    printf("Assignment: Shortest-Path     : O(N * L)\n");
    printf("Assignment: Greedy-LB         : O(N * K * L)\n");
    printf("Assignment: Proposed (GMLR)   : O(N * K * L)\n");
    printf("Evaluation                    : O(E)\n");
    printf("Overall per time step         : O(R*K*(V+E) log V + N*K*L + E)\n");
    printf("Memory                        : O(V + E + R*K*L)\n\n");
    printf("Scalability: running time grows almost linearly with N, R and K,\n");
    printf("and the Dijkstra part grows with (V+E) log V for bigger graphs.\n");
}

// ---------- 9) Limitations (heuristic notes) ----------
void printLimitations(){
    printf("\n===== Limitations =====\n");
    printf("1) Proposed algorithm (GMLR) is a greedy HEURISTIC, not guaranteed optimal.\n");
    printf("   It does not directly minimize global variance or maximum load.\n");
    printf("2) Up to K alternative paths are generated using an edge-penalty technique.\n");
    printf("   They are NOT guaranteed to be the exact K-shortest simple paths,\n");
    printf("   and fewer than K distinct paths may be found.\n");
    printf("3) Greedy-LB looks only at the current maximum load, not average or variance.\n");
    printf("4) Each time step is treated as an independent scenario (CARRY_LOAD = false).\n");
}

int main(){
    // small graph, full details
    runExperiment(10, 18, 3, 3, true, 1);
    // varying sizes, summary only (scalability)
    runExperiment(50, 150, 5, 20, false, 2);
    runExperiment(200, 800, 5, 50, false, 3);
    runExperiment(1000, 4000, 5, 100, false, 4);
    // trade-off between load balancing and distance
    tradeoffAnalysis(200, 800, 5, 50, 3);
    printComplexityAnalysis();
    printLimitations();
    return 0;
}