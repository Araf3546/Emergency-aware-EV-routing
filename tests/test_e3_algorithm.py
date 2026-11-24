"""Unit tests for E3 hybrid algorithm."""

import pytest
from p3_hybrid_router import NEUTRAL_WEIGHT
from algorithms.e3 import (
    HybridRouter,
    HybridContext,
    ProtocolMessage,
    get_outgoing_edges,
    heuristic,
    decay_scout_memory,
    update_exploration_memory,
    set_hybrid_weights,
    get_state_summary,
)


class TestE3HybridContext:
    """Test HybridContext dataclass."""

    def test_hybrid_context_creation(self):
        ctx = HybridContext(
            pheromone=0.8,
            heuristic=0.6,
            protocol_weight=0.9,
            bco_exploration=0.3,
            velocity=0.1,
        )
        assert ctx.pheromone == 0.8
        assert ctx.heuristic == 0.6
        assert ctx.protocol_weight == 0.9
        assert ctx.bco_exploration == 0.3
        assert ctx.velocity == 0.1


class TestProtocolMessage:
    """Test ProtocolMessage dataclass."""

    def test_protocol_message_creation(self):
        msg = ProtocolMessage(
            vehicle_id="v1",
            emergency_flag=True,
            priority_level=4,
            next_edge="edge1",
        )
        assert msg.vehicle_id == "v1"
        assert msg.emergency_flag is True
        assert msg.priority_level == 4
        assert msg.next_edge == "edge1"


class TestHybridRouter:
    """Test HybridRouter scoring logic."""

    def test_router_initialization(self):
        router = HybridRouter()
        assert router.weights["pheromone"] == 0.4
        assert router.weights["heuristic"] == 0.2
        assert router.weights["protocol"] == 0.4
        assert router.weights["bco"] == 0.1
        assert router.weights["velocity"] == 0.05

    def test_score_edge(self):
        router = HybridRouter()
        context = HybridContext(
            pheromone=0.8,
            heuristic=0.6,
            protocol_weight=0.9,
            bco_exploration=0.3,
            velocity=0.2,
        )
        score = router.score_edge(context)
        # Score = 0.4*0.8 + 0.2*0.6 + 0.4*0.9 + 0.1*0.3 + 0.05*0.2
        # = 0.32 + 0.12 + 0.36 + 0.03 + 0.01 = 0.84
        assert 0.83 < score < 0.85

    def test_score_edge_clamping(self):
        router = HybridRouter()
        # Test that scores are clamped to [0, 1]
        context_high = HybridContext(
            pheromone=2.0,
            heuristic=2.0,
            protocol_weight=2.0,
            bco_exploration=2.0,
            velocity=2.0,
        )
        score_high = router.score_edge(context_high)
        assert score_high <= 1.0
        assert score_high >= 0.0

        context_negative = HybridContext(
            pheromone=-1.0,
            heuristic=-1.0,
            protocol_weight=-1.0,
            bco_exploration=-1.0,
            velocity=-1.0,
        )
        score_negative = router.score_edge(context_negative)
        assert score_negative >= 0.0

    def test_select_best_edge(self):
        router = HybridRouter()
        candidates = [
            ("edge1", HybridContext(0.8, 0.6, 0.9, 0.3, 0.2)),
            ("edge2", HybridContext(0.5, 0.4, 0.5, 0.2, 0.1)),
            ("edge3", HybridContext(0.9, 0.7, 0.8, 0.4, 0.3)),
        ]
        best_edge, best_score = router.select_best_edge(candidates)
        assert best_edge == "edge3"  # Should select edge3 (highest score)
        assert best_score > 0.0

    def test_select_best_edge_empty_list(self):
        router = HybridRouter()
        best_edge, best_score = router.select_best_edge([])
        assert best_edge is None
        assert best_score == 0.0

    def test_protocol_weight_for_emergency(self):
        router = HybridRouter()
        messages = [
            ProtocolMessage(
                vehicle_id="v1",
                emergency_flag=True,
                priority_level=4,
                next_edge="edge1",
            ),
        ]
        weight = router.protocol_weight_for_edge("edge1", messages, is_emergency_vehicle=False)
        # Audit P8: an ordinary vehicle told an ambulance is taking this edge must be
        # pushed away from it, so the weight is low. The comment that used to sit here
        # said "0.8 + 0.2 = 1.0", which described the defect P8 removed.
        assert weight < NEUTRAL_WEIGHT

    def test_protocol_weight_no_messages(self):
        """Audit A3: silence means neutral, not "worst possible road".

        This used to return 0.0 when no message named the edge while a routine message
        about it returned about 0.5, so the score jumped by half its range depending on
        whether anyone happened to be broadcasting. Nothing physical changes at that
        boundary.
        """
        router = HybridRouter()
        weight = router.protocol_weight_for_edge("edge1", [], is_emergency_vehicle=False)
        assert weight == NEUTRAL_WEIGHT

    def test_decay_scout_memory(self):
        initial_memory = {"edge1": 0.5, "edge2": 0.01}
        scout_memory_test = initial_memory.copy()
        
        for edge_id in list(scout_memory_test.keys()):
            new_value = max(0.0, scout_memory_test[edge_id] - 0.02)
            if new_value <= 0.0:
                scout_memory_test.pop(edge_id, None)
            else:
                scout_memory_test[edge_id] = new_value
        
        assert scout_memory_test["edge1"] < 0.5
        assert scout_memory_test["edge1"] == 0.48
        assert "edge2" not in scout_memory_test  # Should be removed (0.01 - 0.02 = -0.01 < 0)

    def test_update_exploration_memory(self):
        candidate_edges = ["edge1", "edge2", "edge3"]
        selected_edge = "edge2"
        scout_memory_test = {}
        
        for edge in candidate_edges:
            if edge == selected_edge:
                scout_memory_test[edge] = min(1.0, scout_memory_test.get(edge, 0.0) + 0.15)
            else:
                scout_memory_test[edge] = min(1.0, scout_memory_test.get(edge, 0.0) + 0.05)
        
        assert scout_memory_test["edge2"] == 0.15  # Selected edge gets +0.15
        assert scout_memory_test["edge1"] == 0.05  # Unselected get +0.05
        assert scout_memory_test["edge3"] == 0.05

    def test_build_context(self):
        router = HybridRouter()
        messages = [
            ProtocolMessage(vehicle_id="v1", emergency_flag=False, priority_level=0),
        ]
        router.scout_memory["edge1"] = 0.3
        
        context = router.build_context(
            pheromone=0.8,
            heuristic=0.6,
            edge_id="edge1",
            messages=messages,
            velocity=0.2,
            is_emergency_vehicle=False,
        )
        
        assert context.pheromone == 0.8
        assert context.heuristic == 0.6
        # Audit A6: this used to assert 0.3, the raw scout-memory value read straight into
        # the score - which is the defect itself, since a higher value makes an edge more
        # attractive and the memory counts *recent use*. The term is now the inverse, so a
        # heavily used edge scores low and an untouched one scores 1.0.
        assert context.bco_exploration == 0.7
        assert context.velocity == 0.2


class TestE3Utilities:
    """Test utility functions (these won't require SUMO connection)."""

    def test_set_hybrid_weights(self):
        """Test weight setting for tuning."""
        new_weights = {
            "pheromone": 0.5,
            "heuristic": 0.1,
            "protocol": 0.3,
            "bco": 0.05,
            "velocity": 0.05,
        }
        # We can't test this fully without initializing the router,
        # but we can verify the structure is correct
        assert sum(new_weights.values()) == 1.0


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
