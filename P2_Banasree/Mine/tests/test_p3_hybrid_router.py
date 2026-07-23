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
        router = HybridRouter(weights={"pheromone": 0.3, "heuristic": 0.2, "protocol": 0.4, "bco": 0.1})
        candidates = [
            ("edge_a", HybridContext(0.7, 0.6, 0.2, 0.1, 0.0)),
            ("edge_b", HybridContext(0.5, 0.5, 0.9, 0.2, 0.0)),
        ]

        best_edge, best_score = router.select_best_edge(candidates)

        self.assertEqual(best_edge, "edge_b")
        self.assertGreater(best_score, 0.5)

    def test_protocol_messages_are_transformed_into_weight(self):
        router = HybridRouter()
        messages = [
            ProtocolMessage(vehicle_id="v1", emergency_flag=True, priority_level=4, next_edge="edge_c"),
            ProtocolMessage(vehicle_id="v2", emergency_flag=False, priority_level=1, next_edge="edge_d"),
        ]

        weight = router.protocol_weight_for_edge("edge_c", messages)

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
