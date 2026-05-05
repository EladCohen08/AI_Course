import time
import ex1_331526079
import search
import re

# ==========================================
# ── TEST SUITE 1: HARD & IMPOSSIBLE ──
# ==========================================

# h1 (The Bridge): Requires P10 to transfer at floor 5.
init_state_h1 = {
    "height": 10,
    "Elevators": {0: (0, (0, 1, 2, 3, 4, 5), 20), 1: (10, (5, 6, 7, 8, 9, 10), 20)},
    "Persons": {10: (0, 10, 10)}
}

# h2 (The Weigh-In): E0 can only carry one 6kg person at a time. Forces 2 trips.
init_state_h2 = {
    "height": 8,
    "Elevators": {0: (0, (0, 4, 8), 10)},
    "Persons": {10: (0, 6, 8), 11: (0, 6, 8)}
}

# h3 (The Parity Trap): Carpool logic required to cross Even -> Hub -> Odd.
init_state_h3 = {
    "height": 8,
    "Elevators": {0: (0, (0, 2, 4, 6, 8), 10), 1: (4, (1, 3, 4, 5, 7), 10)},
    "Persons": {10: (0, 5, 7), 11: (8, 5, 1)}
}

# x1 (The Goliath): IMPOSSIBLE (Person 50kg, Elevator 40kg).
init_state_x1 = {
    "height": 5,
    "Elevators": {0: (0, (0, 1, 2, 3, 4, 5), 40)},
    "Persons": {10: (0, 50, 5)}
}

# x2 (The Island): IMPOSSIBLE (No elevator stops at start floor 3).
init_state_x2 = {
    "height": 6,
    "Elevators": {0: (0, (0, 1, 2, 4, 5, 6), 20)},
    "Persons": {10: (3, 5, 0)}
}

# x3 (The Weight Hub): IMPOSSIBLE (Elevator from hub can't carry the weight).
init_state_x3 = {
    "height": 8,
    "Elevators": {0: (0, (0, 4), 20), 1: (4, (4, 8), 10)},
    "Persons": {10: (0, 15, 8)}
}

# ==========================================
# ── TEST SUITE 2: TITAN-TIER (RELAXED) ──
# ==========================================

# t1 (The Mini-Skyscraper): 8 Floors, 3 Elevators, 4 People. 
# Features Local zones + Express route + Transfer Hub at Floor 4.
init_state_t1_relaxed = {
    "height": 8,
    "Elevators": {
        0: (0, (0, 1, 2, 3, 4), 20),       # Lower Local
        1: (4, (4, 5, 6, 7, 8), 20),       # Upper Local
        2: (0, (0, 4, 8), 15)              # The Express
    },
    "Persons": {
        10: (0, 5, 7),  # Ground to Upper Local (Must transfer at 4)
        11: (8, 5, 1),  # Penthouse to Lower Local (Must transfer at 4)
        12: (0, 10, 8), # Ground to Penthouse (Can take Express)
        13: (8, 10, 0)  # Penthouse to Ground (Can take Express)
    }
}

# t2 (The Logistics Bottleneck): Everyone must cross floor 4, tight capacities.
init_state_t2_bottleneck = {
    "height": 8,
    "Elevators": {
        0: (0, (0, 1, 2, 3, 4), 15),
        1: (4, (4, 5, 6, 7, 8), 10)
    },
    "Persons": {
        10: (0, 6, 8), 11: (1, 6, 7), 
        12: (2, 6, 8), 13: (3, 6, 5)
    }
}

# ==========================================
# ── PHYSICS SIMULATOR & VALIDATOR ──
# ==========================================

def simulate_path(initial_data, actions):
    """Plays the actions step-by-step to ensure no physics are broken."""
    elevs = {eid: {"f": spec[0], "F": spec[1], "wmax": spec[2], "passengers": []} 
             for eid, spec in initial_data["Elevators"].items()}
    persons = {pid: {"f": spec[0], "w": spec[1], "goal": spec[2], "in_elev": None} 
               for pid, spec in initial_data["Persons"].items()}
    
    for step, action in enumerate(actions):
        match = re.match(r"([A-Z]+)\{([^}]+)\}", action)
        if not match: return False, f"Invalid action syntax: {action}"
        
        cmd, args = match.groups()
        args = [int(x) for x in args.split(',')]
        
        if cmd == "MOVE":
            eid, target_f = args
            if target_f not in elevs[eid]["F"]:
                return False, f"Step {step}: E{eid} cannot reach floor {target_f}"
            elevs[eid]["f"] = target_f
            for pid in elevs[eid]["passengers"]:
                persons[pid]["f"] = target_f
                
        elif cmd == "ENTER":
            pid, eid = args
            if persons[pid]["in_elev"] is not None:
                return False, f"Step {step}: P{pid} is already in an elevator"
            if persons[pid]["f"] != elevs[eid]["f"]:
                return False, f"Step {step}: P{pid} and E{eid} are not on the same floor"
            
            current_weight = sum(persons[p]["w"] for p in elevs[eid]["passengers"])
            if current_weight + persons[pid]["w"] > elevs[eid]["wmax"]:
                return False, f"Step {step}: E{eid} capacity exceeded by P{pid}"
                
            persons[pid]["in_elev"] = eid
            elevs[eid]["passengers"].append(pid)
            
        elif cmd == "EXIT":
            pid, eid = args
            if persons[pid]["in_elev"] != eid:
                return False, f"Step {step}: P{pid} is not in E{eid}"
            persons[pid]["in_elev"] = None
            elevs[eid]["passengers"].remove(pid)
            
    # Check final goals
    for pid, p in persons.items():
        if p["f"] != p["goal"] or p["in_elev"] is not None:
            return False, f"End State: P{pid} is not at goal (Floor {p['f']}, Elev {p['in_elev']})"
            
    return True, "Path is physically valid."

def verify_problem(name, data, expect_solution=True):
    print(f"\n--- Running: {name} ---")
    p = ex1_331526079.create_elevators_problem(data)
    
    # ── LIVE TELEMETRY TRACKER (Monkey Patch) ──
    original_h = p.h_astar
    eval_count = 0
    t_start = time.time()
    
    def logging_h(node):
        nonlocal eval_count
        eval_count += 1
        # Print a live update every 20,000 nodes evaluated
        if eval_count % 20000 == 0:
            elapsed = time.time() - t_start
            # Using len(node.path()) as a safe way to get current search depth
            current_depth = len(node.path()) - 1 
            print(f"  [Live Log] ⏱️ {elapsed:.1f}s | Nodes Evaluated: {eval_count:,} | Current Search Depth: {current_depth}")
        
        return original_h(node)
    
    # Attach the tracker to the problem instance
    p.h_astar = logging_h
    # ───────────────────────────────────────────
    
    result = search.astar_search(p, p.h_astar)
    duration = time.time() - t_start
    
    if result:
        if not expect_solution:
            print(f"❌ FAIL: Expected Unsolvable, but found a path in {duration:.4f}s")
            return

        final_node, _ = result
        actions = [node.action for node in final_node.path()[::-1]][1:]
        print(f"✅ Path found in {duration:.3f}s | Length: {len(actions)} steps | Total Nodes: {eval_count:,}")
        
        # Run the Simulator
        is_valid, msg = simulate_path(data, actions)
        if is_valid:
            print(f"✅ VERIFIED: {msg}")
        else:
            print(f"❌ INVALIDATED: {msg}")
    else:
        if expect_solution:
            print(f"❌ FAIL: Expected a solution, but A* returned None in {duration:.4f}s")
        else:
            print(f"✅ VERIFIED: Correctly identified as Unsolvable in {duration:.5f}s")

def main():
    print("========================================")
    print(" ULTIMATE AI VALIDATOR & PHYSICS ENGINE ")
    print("========================================\n")
    
    # Run Hard Suite
    verify_problem("h1 (The Bridge)", init_state_h1, True)
    verify_problem("h2 (The Weigh-In)", init_state_h2, True)
    verify_problem("h3 (The Parity Trap)", init_state_h3, True)
    
    # Run Impossible Suite
    verify_problem("x1 (The Goliath)", init_state_x1, False)
    verify_problem("x2 (The Island)", init_state_x2, False)
    verify_problem("x3 (The Weight Hub)", init_state_x3, False)
    
    # Run Titan Suite (Relaxed)
    verify_problem("t1 (The Mini-Skyscraper)", init_state_t1_relaxed, True)
    verify_problem("t2 (The Logistics Bottleneck)", init_state_t2_bottleneck, True)

if __name__ == '__main__':
    main()
