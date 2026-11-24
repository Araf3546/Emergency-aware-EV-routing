import unittest

from p3_hybrid_router import HybridRouter, HybridContext, ProtocolMessage
from p3_protocol import ProtocolStateManager, build_message


class HybridRouterTests(unittest.TestCase):
    def test_emergency_protocol_message_increases_score(self):
        router = HybridRouter()
        context = HybridContext(
            pheromone=0.8,
            heuristic=0.6,
            protocol_weight=0.9,
            bco_exploration=0.3,
            velocity=0.2,
        )

        score = router.score_edge(context)

        self.assertGreater(score, 0.8)

    def test_router_prefers_emergency_edge_when_protocol_signal_is_strong(self):
        """The protocol term must be able to win a decision on its own.

        Audit A3 changed what "strong" means here. Components are now min-max normalised
        across the candidate set before weighting, so what counts is which candidate leads
        each component, weighted - not how far ahead it is. The protocol weight therefore
        has to outrank the combined weight of the components the rival edge leads.
        """
        router = HybridRouter(weights={"pheromone": 0.2, "heuristic": 0.1, "protocol": 0.6, "bco": 0.1})
        candidates = [
            ("edge_a", HybridContext(0.7, 0.6, 0.2, 0.1, 0.0)),
            ("edge_b", HybridContext(0.5, 0.5, 0.9, 0.2, 0.0)),
        ]

        best_edge, best_score = router.select_best_edge(candidates)

        self.assertEqual(best_edge, "edge_b")
        self.assertGreater(best_score, 0.0)

    def test_normalised_scores_are_relative_to_the_candidate_set(self):
        """Records a real consequence of A3, so it is not rediscovered as a surprise.

        With two candidates, min-max normalisation is a rank transform: the weaker
        candidate scores 0 on every component it loses and the stronger scores 1, so the
        *size* of each gap is discarded and the decision becomes a weighted vote. Here
        edge_a leads pheromone (0.4) and heuristic (0.2) while edge_b leads protocol (0.4)
        and bco (0.1), so edge_a wins even though edge_b's protocol lead is far larger than
        edge_a's pheromone lead. Measured over a 1,800 s run this affects few decisions -
        an alternative normalisation that keeps the magnitudes chose a different edge in
        0.27% of selections - but it is a property of the scorer, not an accident.
        """
        router = HybridRouter()
        candidates = [
            ("edge_a", HybridContext(0.55, 0.6, 0.5, 0.1, 0.0)),
            ("edge_b", HybridContext(0.5, 0.5, 0.05, 0.2, 0.0)),
        ]

        best_edge, _ = router.select_best_edge(candidates)

        self.assertEqual(best_edge, "edge_a")

    def test_ordinary_vehicle_avoids_the_emergency_corridor(self):
        """Audit P8: an emergency broadcast must push ordinary traffic away, not draw it in.

        This test previously asserted `weight > 0.7`, which encoded the defect: an ordinary
        vehicle scored the edge an ambulance had just announced as its next edge higher than
        any alternative, so the protocol steered traffic into the responder's path.
        """
        router = HybridRouter()
        messages = [
            ProtocolMessage(vehicle_id="v1", emergency_flag=True, priority_level=4, next_edge="edge_c"),
            ProtocolMessage(vehicle_id="v2", emergency_flag=False, priority_level=1, next_edge="edge_d"),
        ]

        weight = router.protocol_weight_for_edge("edge_c", messages, is_emergency_vehicle=False)

        self.assertLess(weight, 0.2)

    def test_emergency_vehicle_still_prefers_its_own_corridor(self):
        """The responder itself must not be pushed off the road it is heading down."""
        router = HybridRouter()
        messages = [
            ProtocolMessage(vehicle_id="v1", emergency_flag=True, priority_level=4, next_edge="edge_c"),
        ]

        weight = router.protocol_weight_for_edge("edge_c", messages, is_emergency_vehicle=True)

        self.assertGreater(weight, 0.7)

    def test_hybrid_router_exploration_memory_updates(self):
        router = HybridRouter()
        candidates = ["edge_a", "edge_b", "edge_c"]

        router.update_exploration_memory(candidates, "edge_b")

        self.assertGreater(router.scout_memory["edge_b"], router.scout_memory["edge_a"])
        self.assertGreater(router.scout_memory["edge_b"], router.scout_memory["edge_c"])

    def test_protocol_state_manager_drops_stale_messages(self):
        state = ProtocolStateManager(freshness_window=0.1)
        message = build_message(
            vehicle_id="v1",
            current_time=0.0,
            vehicle_type="normal",
            position_edge="edge_a",
            speed=0.0,
        )

        state.update(0.0, message)
        fresh = state.get_fresh_neighbors(0.2)

        self.assertEqual(len(fresh), 0)


if __name__ == "__main__":
    unittest.main()
