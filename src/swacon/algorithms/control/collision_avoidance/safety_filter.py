import traceback
from typing import Dict, List

import cvxopt
import numpy as np

# TODO: documentation
DEFAULT_SAFETY_RADIUS = 0.3
DEFAULT_SAFETY_DISTANCE = 0.5
DEFAULT_GAMMA = 7.5
DEFAULT_IS_GAMMA_POWER = False


# TODO: documentation
class AgentState:
    def __init__(self, x: float, y: float, vx: float, vy: float) -> None:
        assert isinstance(x, float)
        assert isinstance(y, float)
        assert isinstance(vx, float)
        assert isinstance(vy, float)
        self.x = x
        self.y = y
        self.vx = vx
        self.vy = vy


# TODO: documentation
def solve_collision_neighbors(agents: Dict[str, AgentState], safety_radius: float = DEFAULT_SAFETY_RADIUS) -> Dict[str, List[str]]:
    assert isinstance(agents, dict)
    neighbors: Dict[str, List[str]] = dict()
    for agent_name, agent in agents.items():
        agent_neighbors: List[str] = list()
        for neighbor_name, neighbor in agents.items():
            if agent_name != neighbor_name:
                dx = neighbor.x - agent.x
                dy = neighbor.y - agent.y
                distance = np.sqrt(np.square(dx) + np.square(dy))
                if distance <= safety_radius:
                    agent_neighbors.append(neighbor_name)
        neighbors[agent_name] = agent_neighbors
    return neighbors


# TODO: documentation
def solve_new_state(
    agent_name: str,
    agent: AgentState,
    neighbor_states: Dict[str, AgentState],
    safety_distance: float = DEFAULT_SAFETY_DISTANCE,
    gamma: float = DEFAULT_GAMMA,
    is_gamma_power: float = DEFAULT_IS_GAMMA_POWER,
) -> AgentState:
    H = np.zeros((len(neighbor_states), 2))
    b = np.zeros((len(neighbor_states), 1))
    for i, (neighbor_name, neighbor) in enumerate(neighbor_states.items()):
        hx_cal = np.square(agent.x - neighbor.x) + np.square(agent.y - neighbor.y) - np.square(safety_distance)
        if is_gamma_power:
            gamma = np.power(hx_cal, gamma)
        else:
            gamma = gamma * hx_cal
        H[i] = -2 * np.transpose(np.array([agent.x, agent.y]) - np.array([neighbor.x, neighbor.y]))
        b[i] = gamma
    # Resize the H and b into appropriate matrix for optimization
    H_mat = cvxopt.matrix(H, tc="d")
    b_mat = cvxopt.matrix(b, tc="d")
    # Calculate Q and c matrices
    Q_mat = +2 * cvxopt.matrix(np.eye(2), tc="d")
    c_mat = -2 * cvxopt.matrix(np.array([agent.vx, agent.vy]), tc="d")
    try:
        cvxopt.solvers.options["show_progress"] = False
        solution = cvxopt.solvers.qp(Q_mat, c_mat, H_mat, b_mat, verbose=False)
        solution = solution["x"]
        new_avx, new_avy = solution
        if agent.vx != 0.0 and agent.vy != 0.0:
            if new_avx == 0.0 and new_avy == 0.0:
                print(f"[{agent_name}] deadlocked")
    except Exception as e:
        # If the optimization problem does not have a solution, then stop the agent
        print(f"[{agent_name}] QP solver failed with U=0")
        traceback.print_exc()
        new_avx = 0.0
        new_avy = 0.0
    return AgentState(agent.x, agent.y, new_avx, new_avy)


# TODO: documentation
def safety_filter(
    agents: Dict[str, AgentState],
    safety_radius: float = DEFAULT_SAFETY_RADIUS,
    safety_distance: float = DEFAULT_SAFETY_DISTANCE,
    gamma: float = DEFAULT_GAMMA,
    is_gamma_power: float = DEFAULT_IS_GAMMA_POWER,
) -> Dict[str, AgentState]:
    """Filter agents' control signals to avoid collisions

    ### Arguments
    - `agents`: key is drone name, value is AgentState
    - `safety_radius`: ???
    - `safety_distance`: ???
    - `gamma`: ???
    - `is_gamma_power`: ???

    ### Result
    - key is drone name, value is filtered AgentState
    """
    assert isinstance(agents, dict)
    collision_neighbors = solve_collision_neighbors(agents, safety_radius)
    new_agents: Dict[str, AgentState] = dict()
    for agent_name, neighbors in collision_neighbors.items():
        neighbor_states: Dict[str, AgentState] = dict()
        for neighbor_name in neighbors:
            neighbor_state = agents[neighbor_name]
            neighbor_states[neighbor_name] = neighbor_state
        old_state = agents[agent_name]
        new_state = solve_new_state(agent_name, old_state, neighbor_states, safety_distance, gamma, is_gamma_power)
        new_agents[agent_name] = new_state
    return new_agents
