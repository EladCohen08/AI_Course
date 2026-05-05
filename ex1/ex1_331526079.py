import search
import math
import time
from collections import deque

# Const penalty for non reachable, 
# still finite in case it isnt really infinite, 
# so it would find a solution.
INF_PENALTY = 10**9 

class ElevatorsProblem(search.Problem):
    def __init__(self, initial):
        self.height = initial['height']
        self.elevator_specs = initial['Elevators'] # {id: (f, F, wmax)} 
        
        # {id: (fstart, w, fgoal)}, Remove people already at their goal.
        self.person_specs = {
                    pid: spec for pid, spec in initial['Persons'].items() 
                    if spec[0] != spec[2]
        }   
        
        # -- State Structure Definitions
        # elev_state: ((id, current_floor), ...) 
        # p_state:    ((id, current_floor, in_elevator_id_or_None), ...)
        # Sorted for consistency in goal checking, gemini suggested this.
        elev_state = tuple(sorted((eid, spec[0]) for eid, spec in self.elevator_specs.items()))
        p_state = tuple(sorted(
            (pid, spec[0], None) 
            for pid, spec in self.person_specs.items()
        ))
        
        # Precompute static data for the heuristic and search efficiency
        self.max_cap_per_floor = self._compute_max_capacities()
        self.floor_dist_map = self._precompute_floor_distances()
        self.elev_dist_map = self._precompute_elevator_min_hops()
        self.floor_nav_dist = self._precompute_floor_nav_dist()
        self._mst_cache = {}
        self._h_cache = {}
        self._precompute_string_actions()

        # If the current floor of a person CANT reach 
        # the goal floor, flag as impossible.
        self.is_impossible = False
        for pid, (start_f, _, _) in self.person_specs.items():
            if self.floor_dist_map[pid][start_f] == INF_PENALTY:
                self.is_impossible = True
                break
             
        # elevator state, persons state 
        search.Problem.__init__(self, (elev_state, p_state))

    def _precompute_string_actions(self):
        """Precompute all possible action strings to avoid overhead in successor."""
        self.move_action_strs = {(eid, f): f"MOVE{{{eid},{f}}}" 
                         for eid, spec in self.elevator_specs.items() 
                         for f in spec[1]}

        self.enter_action_strs = {(pid, eid): f"ENTER{{{pid},{eid}}}" 
                          for pid in self.person_specs 
                          for eid in self.elevator_specs}

        self.exit_action_strs = {(pid, eid): f"EXIT{{{pid},{eid}}}" 
                         for pid in self.person_specs 
                         for eid in self.elevator_specs}


    def _compute_max_capacities(self):
        """Maps each floor to the highest weight capacity available to reach it."""
        max_caps = {f: 0 for f in range(self.height + 1)}
        for _, (_, reachable, wmax) in self.elevator_specs.items():
            for floor in reachable:
                if wmax > max_caps[floor]:
                    max_caps[floor] = wmax
        return max_caps

    def _precompute_floor_distances(self):
        """
        Uses a Reverse BFS to find the minimum number of elevator hops 
        needed for each person(accounting for weight limits)
        to get from any floor to their destination.
        """
        floor_dist_map = {}
        
        # -- Iterate through each person to build their custom building map 
        for pid, (_, weight, goal_f) in self.person_specs.items():
            floor_dist_map[pid] = {goal_f: 0}   # Distance to goal is 0
            queue = deque([goal_f])        # Start searching backwards from the goal
            visited = {goal_f}             # Keep track of floors we've already mapped
            
            while queue:
                curr_f = queue.popleft()
                d = floor_dist_map[pid][curr_f]
                
                # Find all floors reachable in a single elevator jump 
                for _, (_, reachable, wmax) in self.elevator_specs.items():
                    # Only consider elevators that can carry this person's weight
                    if weight <= wmax and curr_f in reachable:
                        # Every floor this elevator can touch is exactly 1 hop away from curr_f
                        for prev_f in reachable:
                            if prev_f not in visited:
                                visited.add(prev_f)
                                floor_dist_map[pid][prev_f] = d + 1
                                queue.append(prev_f)

            # f is not rechable for that person, set distance to inf
            for f in range(self.height + 1):
                if f not in floor_dist_map[pid]:
                    floor_dist_map[pid][f] = INF_PENALTY

        return floor_dist_map

    def _precompute_floor_nav_dist(self):
        """Min MOVE actions between any two floors — same BFS as floor_dist_map but weight=0 (uses all elevators)."""
        all_floors = set(f for _, (_, reachable, _) in self.elevator_specs.items() for f in reachable)
        nav_dist = {}
        for start_f in all_floors:
            nav_dist[start_f] = {start_f: 0}
            queue = deque([start_f])
            visited = {start_f}
            while queue:
                curr_f = queue.popleft()
                d = nav_dist[start_f][curr_f]
                for _, (_, reachable, _) in self.elevator_specs.items():
                    if curr_f in reachable:
                        for f2 in reachable:
                            if f2 not in visited:
                                visited.add(f2)
                                nav_dist[start_f][f2] = d + 1
                                queue.append(f2)
        return nav_dist

    def _mst_active_floors(self, active_floors):
        """Prim's MST over active floors — cached so each unique floor set is computed once."""
        key = frozenset(active_floors)
        if key in self._mst_cache:
            return self._mst_cache[key]
        if len(active_floors) <= 1:
            self._mst_cache[key] = 0
            return 0
        floors = list(active_floors)
        in_mst = {floors[0]}
        remaining = set(floors[1:])
        total = 0
        while remaining:
            best, best_floor = INF_PENALTY, None
            for f in in_mst:
                nav = self.floor_nav_dist.get(f, {})
                for g in remaining:
                    d = nav.get(g, INF_PENALTY)
                    if d < best:
                        best, best_floor = d, g
            total += best
            in_mst.add(best_floor)
            remaining.remove(best_floor)
        self._mst_cache[key] = total
        return total

    def _precompute_elevator_min_hops(self):
        """
        For every person and every elevator, find the minimum hops required
        to reach the goal from ANY floor that elevator stops at.
        """
        elev_dist_map = {}
        
        for pid in self.person_specs:
            elev_dist_map[pid] = {}
            for eid, (_, reachable, _) in self.elevator_specs.items():
                # We look at every floor this elevator touches and find the 
                # one closest to the person's goal (using our existing BFS map).
                best_hop_count = min(self.floor_dist_map[pid].get(f, INF_PENALTY) for f in reachable)
                elev_dist_map[pid][eid] = best_hop_count
                
        return elev_dist_map

    def successor(self, state):
        if self.is_impossible:
            return [] # Kill all successors to terminate fast

        elevators, persons = state
        succ_states = []
        
        elev_weights = {eid: 0 for eid, _ in elevators}
        elevs_by_floor = {}
        for eid, ef in elevators:
            if ef not in elevs_by_floor:
                elevs_by_floor[ef] = []
            elevs_by_floor[ef].append(eid)

        unfinished_floors = set()
        busy_elevators = set()
        
        # -- Exit actions
        for i, (p_id, p_floor, p_in_elev) in enumerate(persons):
            if p_in_elev is not None:
                # Update metadata for people currently in elevators
                p_weight = self.person_specs[p_id][1]
                goal_f = self.person_specs[p_id][2]
                action_str = self.exit_action_strs[(p_id, p_in_elev)]

                # PRUNING: If someone can finish their journey, we return ONLY this.
                if p_floor == goal_f:
                    # Deletes the person from the tuple entirely.
                    new_persons = persons[:i] + persons[i+1:]
                    return [(action_str, (elevators, new_persons))]

                elev_weights[p_in_elev] += p_weight
                busy_elevators.add(p_in_elev)
                
                # Regular EXIT: Getting out at a non-goal floor.
                # SLICING: Updates only this person's entry to None (standing on floor).
                new_p_entry = (p_id, p_floor, None)
                new_persons = persons[:i] + (new_p_entry,) + persons[i+1:]
                succ_states.append((action_str, (elevators, new_persons)))
            
            else:
                # Not in elevator means they are waiting on a floor and unfinished.
                unfinished_floors.add(p_floor)

        # -- ENTER Actions 
        # sum current passenger weights inside each elevator
        for i, (p_id, p_floor, p_in_elev) in enumerate(persons):
            if p_in_elev is None: 
                p_weight = self.person_specs[p_id][1]
                for e_id in elevs_by_floor.get(p_floor, []):
                    # Check constraints and add to elevator
                    if elev_weights[e_id] + p_weight <= self.elevator_specs[e_id][2]:
                        new_p_entry = (p_id, p_floor, e_id)
                        new_persons = persons[:i] + (new_p_entry,) + persons[i+1:]
                        succ_states.append((self.enter_action_strs[(p_id, e_id)], (elevators, new_persons)))

        # -- MOVE Actions 
        for e_idx, (elev_id, elev_floor) in enumerate(elevators):
            is_elev_empty = elev_id not in busy_elevators

            for target_floor in self.elevator_specs[elev_id][1]:
                if target_floor == elev_floor:
                    continue
                
                # PRUNING: Never move an EMPTY elevator to a floor with NO waiting people
                if is_elev_empty and target_floor not in unfinished_floors:
                    continue
                
                new_e_entry = (elev_id, target_floor)
                new_elevs = elevators[:e_idx] + (new_e_entry,) + elevators[e_idx+1:]
                
                # Only update persons if the elevator moving actually has people in it
                if is_elev_empty:
                    new_persons = persons
                else:
                    new_persons = tuple((pid, target_floor if pin == elev_id else pf, pin) 
                                       for pid, pf, pin in persons)
                
                succ_states.append((self.move_action_strs[(elev_id, target_floor)], (new_elevs, new_persons)))

        return succ_states

    def goal_test(self, state):
        _, persons = state
        # If the tuple is empty, all the people had already reached their floor.
        return len(persons) == 0

    def h_astar(self, node):
        _, persons = node.state

        if not persons:
            return 0

        if persons in self._h_cache:
            return self._h_cache[persons]

        if len(persons) == 1:
            pid, curr_f, in_elev = persons[0]
            if in_elev is None:
                hops = self.floor_dist_map[pid].get(curr_f, INF_PENALTY)
                result = 3 * hops
            else:
                hops_from_elev = self.elev_dist_map[pid].get(in_elev, INF_PENALTY)
                result = 1 + (3 * hops_from_elev)
            self._h_cache[persons] = result
            return result


        # -- General heauristic for multiple people
        active_floors = set() # All floors with either unfinished people or unfinished destinations
        floor_weights_out = {} # weight needing to depart floor
        floor_weights_in = {}  # weight needing to arrive at floor
        h_enters_exits = 0
       
        for pid, curr_f, in_elev in persons:
            p_weight = self.person_specs[pid][1]
            goal_f = self.person_specs[pid][2]
            
            if in_elev is not None:
                # In elevator: 1 EXIT + (2 * hops(ENTER + EXIT) to goal)
                h_enters_exits += 1 + (2 * self.elev_dist_map[pid][in_elev])
                active_floors.add(goal_f)

                # Weight demand for arrival floor
                if curr_f != goal_f:
                    floor_weights_in[goal_f] = floor_weights_in.get(goal_f, 0) + p_weight
            else:
                # On floor: (2 * hops(ENTER + EXIT) to goal) for ENTER/EXIT
                h_enters_exits += 2 * self.floor_dist_map[pid].get(curr_f, INF_PENALTY)
                active_floors.add(curr_f)
                active_floors.add(goal_f)

                floor_weights_out[curr_f] = floor_weights_out.get(curr_f, 0) + p_weight
                floor_weights_in[goal_f] = floor_weights_in.get(goal_f, 0) + p_weight
        

        # Move Heauristic
        h_moves_out = 0
        for f, w in floor_weights_out.items():
            cap = self.max_cap_per_floor[f]
            if cap == 0: # Safety check to avoid division by zero
                if w > 0: # people with no elevator that can pick them up
                    return INF_PENALTY 
            else:
                h_moves_out += math.ceil(w / cap)

        h_moves_in = 0
        for f, w in floor_weights_in.items():
            cap = self.max_cap_per_floor[f]
            if cap == 0:
                if w > 0: 
                    return INF_PENALTY
            else:
                h_moves_in += math.ceil(w / cap)

        # 3. Combine using the Union/Max principle
        # Shared moves are the worst-case of trips needed or hops needed.
        shared_moves = max(h_moves_out, h_moves_in, self._mst_active_floors(active_floors))

        result = h_enters_exits + shared_moves
        self._h_cache[persons] = result
        return result

def create_elevators_problem(game):
    return ElevatorsProblem(game)
