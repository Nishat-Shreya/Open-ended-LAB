# Traffic-Aware Vehicle Routing and Load Balancing in a Road Network

**Khulna University of Engineering & Technology (KUET)**
Department of Computer Science and Engineering
**CSE 2202**: Algorithm Analysis and Design Laboratory

| Name | Roll |
|---|---|
| Nishat Tasin Shreya | 2307035 |
| Tasmin Rubaiat Rimve | 2307043 |

---

## Overview

A city road network is modeled as a connected, undirected, weighted graph `G = (V, E, w)`.
At every discrete time step, a set of transportation requests arrives. Each request
`rᵢ = (nᵢ, sᵢ, dᵢ)` asks for `nᵢ` vehicles to travel from intersection `sᵢ` to intersection `dᵢ`.

The goal is to split the vehicles of each request over one or more valid paths so that

- traffic is spread evenly over the roads (low maximum load, low variance), and
- the total travel distance stays as small as possible.

This project generates random road networks of different sizes, implements three routing
strategies, evaluates them with four metrics, and compares them on exactly the same instances.

## Methods compared

| Mode | Method | Idea |
|---|---|---|
| 0 | **Shortest-Path** | Every vehicle takes the shortest path. Load is ignored. |
| 1 | **Greedy-LB** | Every vehicle takes the path whose busiest road has the lowest load. Distance only breaks ties. |
| 2 | **Proposed GMLR** (Greedy Multi-Path Load-Aware Routing) | Every vehicle takes the path with the lowest combined score of distance and load. |

## How it works

1. **Generate the graph.** A random spanning tree guarantees connectivity, then random extra edges are added (weights 1 to 20).
2. **Generate requests.** At each time step, `R` requests with random vehicle count, source and destination (source ≠ destination).
3. **Candidate paths (penalty method).** Run Dijkstra to get the shortest path `P₀`, double the cost of its edges, run Dijkstra again for the next path, and repeat up to `K` times. Duplicate paths are dropped.
4. **Assign vehicles.** Vehicles are placed one at a time on the best-scoring path, and the road loads are updated after every vehicle.
5. **Evaluate** each method with the four metrics, repeat for every time step, and report the average.

## Formulas

Path length:

$$\mathrm{len}(p)=\sum_{e\in p} w(e)$$

Highest road load on a path if one more vehicle is added:

$$\mathrm{mx}(p)=\max_{e\in p}\bigl(\mathrm{load}(e)+1\bigr)$$

Score used to pick a path for the next vehicle (smaller is better):

$$\mathrm{score}(p)=\frac{\mathrm{len}(p)}{\mathrm{len}(P_0)}+\beta\cdot \mathrm{mx}(p)$$

Evaluation metrics (`m = |E|`):

$$\mathrm{MaxLoad}=\max_{e\in E}\ \mathrm{load}(e)$$

$$\mathrm{AvgLoad}=\frac{1}{m}\sum_{e\in E}\mathrm{load}(e)$$

$$\mathrm{Variance}=\frac{1}{m}\sum_{e\in E}\bigl(\mathrm{load}(e)-\mathrm{AvgLoad}\bigr)^{2}$$

$$\mathrm{TotalVehicleDistance}=\sum_{e\in E}\bigl(\mathrm{load}(e)-\mathrm{startLoad}(e)\bigr)\cdot w(e)$$

`β ≥ 0` controls the trade-off. At `β = 0`, GMLR behaves exactly like Shortest-Path.

## Build and run

```bash
g++ -std=c++17 -O2 traffic_routing.cpp -o traffic_routing
./traffic_routing
```

No external libraries are needed.

## Input

The program reads nothing from the keyboard or from a file. All data is generated randomly
with fixed seeds, so every run gives the same result. The parameters are set in `main()`:

| Parameter | Meaning | Values used |
|---|---|---|
| `nodes`, `edges` | Intersections and roads (V, E) | (10, 18), (50, 150), (200, 800), (1000, 4000) |
| `T` | Time steps | 3, 5, 5, 5 |
| `R` | Requests per time step | 3, 20, 50, 100 |
| `detailed` | Print paths and road-wise loads | `true` only for the first (small) graph |
| `seed` | Random seed | 1, 2, 3, 4 |
| edge weight | Road length | 1 to 20 (random) |
| request | (vehicles `nᵢ`, source `sᵢ`, destination `dᵢ`) | `nᵢ` from 1 to 50, `sᵢ ≠ dᵢ` |
| `K` | Candidate paths per request (maximum) | 4 |
| `beta` | Load vs. distance weight | 0.1 (swept from 0 to 2 in the trade-off test) |

To change the experiment, edit the `runExperiment(...)` calls in `main()`.
Set `CARRY_LOAD = true` at the top of the file to carry road loads from one time step to the next.

## Output

For the small graph (V = 10) the program prints:

- the generated road network (edge list and adjacency list),
- for every time step, the assigned paths and how many vehicles use each path,
- the road-wise traffic load,
- the four metrics for all three methods.

For every graph size it then prints the metrics averaged over all time steps, together with
the assignment time and the candidate-path generation time. At the end it prints the
β trade-off table, the complexity analysis, and the limitations.

Example (V = 10, time step 2, GMLR):

```text
Request 1: (n=14, s=4, d=2)
   12 vehicles, len=7, path: 4 -> 10 -> 2
   2 vehicles, len=14, path: 4 -> 2
...
[Proposed(GMLR)] MaxLoad=24 AvgLoad=8.00 Variance=67.11 TotalVehicleDistance=710
```

The complete output of one run is saved in [`output.txt`](output.txt).
All numbers except the timings are identical on every machine.

## Sample results

Large instance: V = 1000, E = 4000, 100 requests per step, averaged over 5 time steps.

| Method | MaxLoad | AvgLoad | Variance | TotalVehDist |
|---|---|---|---|---|
| Shortest-Path | 132.80 | 3.57 | 159.63 | **52131.80** |
| Greedy-LB | **52.00** | 3.55 | **68.21** | 62666.80 |
| Proposed (GMLR) | 57.00 | 3.48 | 69.13 | 60829.00 |

Effect of β (V = 200, E = 800, 50 requests per step, 5 time steps):

| β | MaxLoad | Variance | TotalVehDist |
|---|---|---|---|
| 0.00 | 164.80 | 335.65 | 20721.40 |
| 0.05 | 70.60 | 140.04 | 24347.40 |
| 0.10 | 60.00 | 131.71 | 25288.40 |
| 0.30 | 54.00 | 129.09 | 26126.00 |
| 0.50 | 53.60 | 129.07 | 26292.20 |
| 1.00 | 53.40 | 129.65 | 26437.00 |
| 2.00 | 53.60 | 129.74 | 26458.00 |

### Observations

- Shortest-Path ignores load, so its maximum load and variance are the highest, but its total distance is the lowest.
- Greedy-LB gives the lowest maximum load, but it takes long detours and has the highest total distance.
- GMLR keeps the load almost as balanced as Greedy-LB while driving less distance.
- Raising β lowers MaxLoad and Variance at first, then the gain levels off after about β = 0.3 to 0.5, while the distance keeps growing. A β around 0.1 to 0.3 is a good balance for these instances.
- Most of the running time goes into candidate-path generation (repeated Dijkstra), not into vehicle assignment.

## Complexity

`V` = intersections, `E` = roads, `R` = requests, `N` = total vehicles, `K` = candidate paths per request, `L` = average edges per path.

| Part | Complexity |
|---|---|
| Graph generation | O(V + E) expected |
| Dijkstra (one run) | O((V + E) log V) |
| Candidate path generation | O(R · K · (V + E) log V) |
| Assignment: Shortest-Path | O(N · L) |
| Assignment: Greedy-LB and GMLR | O(N · K · L) |
| Evaluation | O(E) |
| Memory | O(V + E + R · K · L) |

## Limitations

1. GMLR is a greedy heuristic, not an optimal algorithm. It does not directly minimize the global variance or the maximum load.
2. Candidate paths come from an edge-penalty technique, not from an exact K-shortest-simple-paths method such as Yen's algorithm. Fewer than `K` distinct paths may be found.
3. Greedy-LB looks only at the current maximum load, not at the average load or the variance.
4. Each time step is an independent scenario (`CARRY_LOAD = false`), so congestion that builds up over several steps is not modeled.

## Files

| File | Description |
|---|---|
| `traffic_routing.cpp` | Full C++ source code |
| `output.txt` | Output of one complete run |
| `README.md` | This file |

## Reproducibility note

The score of a path is computed in two separate steps (distance part and load part) so the
compiler cannot fuse them into one floating-point instruction. This keeps the results
identical on x86 and on ARM machines.
