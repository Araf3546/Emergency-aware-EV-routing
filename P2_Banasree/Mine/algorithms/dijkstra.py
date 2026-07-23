import traci
import heapq

MAX_HOPS = 5   # 🔑 critical: local horizon


def get_outgoing_edges(edge_id):
    outgoing = set()
    try:
        lane_id = f"{edge_id}_0"
        for link in traci.lane.getLinks(lane_id):
            next_lane = link[0]
            next_edge = next_lane.split("_")[0]
            if not next_edge.startswith(":"):
                outgoing.add(next_edge)
    except traci.TraCIException:
        pass
    return list(outgoing)


def dijkstra(start_edge, target_edge):
    dist = {start_edge: 0.0}
    prev = {}
    pq = [(0.0, start_edge)]
    visited = set()

    while pq:
        cur_dist, cur_edge = heapq.heappop(pq)

        if cur_edge in visited:
            continue
        visited.add(cur_edge)

        if len(prev) > MAX_HOPS:
            break

        if cur_edge == target_edge:
            break

        for nxt in get_outgoing_edges(cur_edge):
            if nxt == cur_edge:
                continue

            try:
                w = traci.edge.getTraveltime(nxt)
            except traci.TraCIException:
                continue

            nd = cur_dist + w
            if nd < dist.get(nxt, float("inf")):
                dist[nxt] = nd
                prev[nxt] = cur_edge
                heapq.heappush(pq, (nd, nxt))

    # reconstruct
    path = []
    e = target_edge
    while e in prev:
        path.insert(0, e)
        e = prev[e]

    if path:
        path.insert(0, start_edge)

    return path[:MAX_HOPS + 1]
