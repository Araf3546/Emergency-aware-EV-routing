import unittest

from p3_metrics import P3MetricsLogger


class P3MetricsLoggerTests(unittest.TestCase):
    def test_record_broadcasts_and_counts(self):
        metrics = P3MetricsLogger()
        metrics.record_broadcast("veh1", emergency_flag=False, yield_request=False)
        metrics.record_broadcast("veh2", emergency_flag=True, yield_request=True)

        self.assertEqual(metrics.vehicle_broadcasts, 2)
        self.assertEqual(metrics.emergency_messages, 1)
        self.assertEqual(metrics.yield_requests, 1)
        self.assertEqual(metrics.per_vehicle["veh2"]["emergency_broadcasts"], 1)

    def test_record_neighbor_state_and_candidate_scoring(self):
        metrics = P3MetricsLogger()
        metrics.record_neighbor_state(3, 1)
        metrics.record_candidate_scoring(5)
        metrics.record_emergency_zone_snapshot(2)

        self.assertEqual(metrics.fresh_neighbor_messages, 3)
        self.assertEqual(metrics.emergency_neighbor_messages, 1)
        self.assertEqual(metrics.candidate_edges_scored, 5)
        self.assertEqual(metrics.emergency_zones_observed, 2)

    def test_record_reroute_execution(self):
        metrics = P3MetricsLogger()
        metrics.record_reroute("veh1", True, is_emergency_vehicle=False, emergency_context_active=True)

        self.assertEqual(metrics.reroute_decisions, 1)
        self.assertEqual(metrics.reroute_executed, 1)
        self.assertEqual(metrics.emergency_reroutes, 1)
        self.assertIn("veh1", metrics.vehicles_rerouted)

    def test_record_vehicle_step_and_arrival(self):
        metrics = P3MetricsLogger()
        metrics.record_vehicle_step(
            vehicle_id="veh1",
            sim_time=0.0,
            vehicle_type="normal",
            is_emergency=False,
            speed=5.0,
            waiting_time=1.0,
            distance=10.0,
        )
        metrics.record_vehicle_arrival("veh1", 120.0)

        self.assertEqual(metrics.vehicle_logs["veh1"]["start_time"], 0.0)
        self.assertEqual(metrics.vehicle_logs["veh1"]["end_time"], 120.0)
        self.assertEqual(metrics.vehicle_logs["veh1"]["waiting_time"], 1.0)
        self.assertEqual(metrics.vehicle_logs["veh1"]["distance"], 10.0)

    def test_record_vehicle_reroute_counts(self):
        metrics = P3MetricsLogger()
        metrics.record_vehicle_reroute("veh1")
        metrics.record_vehicle_reroute("veh1")

        self.assertEqual(metrics.vehicle_logs["veh1"]["total_reroutes"], 2)

    def test_record_emergency_event_response(self):
        metrics = P3MetricsLogger()
        metrics.record_emergency_event_response("event1", 12.5)

        self.assertEqual(metrics.emergency_event_responses, 1)
        self.assertEqual(metrics.emergency_response_times, [12.5])


if __name__ == "__main__":
    unittest.main()
