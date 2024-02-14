import unittest

from swacon.algorithms.control.collision_avoidance.safety_filter import AgentState, solve_collision_neighbors, solve_new_state, safety_filter  # noqa: F401


class Test(unittest.TestCase):
    def test_bad_input(self) -> None:
        self.assertRaises(AssertionError, safety_filter, None)

    def test_one_agent(self) -> None:
        agents = {
            "a1": AgentState(0.0, 0.0, 0.0, 0.0),
        }
        self.assertEqual(safety_filter(agents), agents)

    def test_two_agents(self) -> None:
        agents = {
            "a1": AgentState(0.0, 0.0, 0.0, 0.0),
            "a2": AgentState(0.0, 0.0, 0.0, 0.0),
        }
        self.assertNotEqual(safety_filter(agents), agents)
