import time
import ex1_331526079
import search

# ── Test Cases ──

init_state_p1 = {
    "height": 6,
    "Elevators": {0: (0, (0, 1, 2, 3), 8), 1: (4, (2, 4, 5, 6), 10)},
    "Persons": {10: (0, 3, 3), 11: (2, 4, 6), 12: (4, 5, 0)}
}

init_state_e1 = {
    "height": 5,
    "Elevators": {0: (0, (0, 1, 2, 3, 4, 5), 15), 1: (5, (0, 1, 2, 3, 4, 5), 15)},
    "Persons": {10: (0, 3, 5), 11: (5, 3, 0), 12: (3, 3, 1)}
}

init_state_e2 = {
    "height": 6,
    "Elevators": {0: (0, (0, 1, 2, 3), 10), 1: (6, (3, 4, 5, 6), 10)},
    "Persons": {10: (1, 3, 3), 11: (5, 3, 4), 12: (0, 3, 2)}
}

init_state_e3 = {
    "height": 6,
    "Elevators": {0: (0, (0, 1, 2, 3, 4), 10), 1: (6, (2, 4, 5, 6), 10)},
    "Persons": {10: (0, 3, 5), 11: (6, 3, 1), 12: (3, 4, 6)}
}

init_state_e4 = {
    "height": 5,
    "Elevators": {0: (0, (0, 1, 2, 3, 4, 5), 7), 1: (5, (0, 1, 2, 3, 4, 5), 7)},
    "Persons": {10: (0, 5, 5), 11: (0, 5, 3), 12: (5, 5, 0)}
}

init_state_e5 = {
    "height": 7,
    "Elevators": {0: (0, (0, 1, 2, 3, 4), 12), 1: (7, (3, 4, 5, 6, 7), 12)},
    "Persons": {10: (1, 4, 3), 11: (6, 4, 7), 12: (0, 4, 7)}
}

init_state_m1 = {
    "height": 6,
    "Elevators": {0: (0, (0, 1, 2, 3), 12), 1: (6, (3, 4, 5, 6), 12)},
    "Persons": {10: (0, 4, 5), 11: (5, 4, 0), 12: (2, 4, 6), 13: (5, 4, 1)}
}

init_state_m2 = {
    "height": 8,
    "Elevators": {0: (0, (0, 1, 2, 3, 4), 10), 1: (4, (2, 4, 6, 8), 10)},
    "Persons": {10: (0, 3, 8), 11: (8, 3, 0), 12: (2, 3, 6), 13: (6, 3, 1)}
}

init_state_m3 = {
    "height": 8,
    "Elevators": {0: (0, (0, 1, 2, 3, 4), 10), 1: (4, (2, 4, 6, 8), 10), 2: (4, (7, 2), 10)},
    "Persons": {10: (0, 3, 8), 11: (8, 3, 0), 12: (2, 3, 7), 13: (6, 3, 1)}
}

init_state_m4 = {
    "height": 6,
    "Elevators": {0: (0, (0, 1, 2, 3, 4, 5, 6), 8), 1: (6, (0, 1, 2, 3, 4, 5, 6), 8)},
    "Persons": {10: (0, 5, 6), 11: (0, 5, 4), 12: (6, 5, 0), 13: (6, 5, 2), 14: (3, 5, 6)}
}

init_state_m5 = {
    "height": 8,
    "Elevators": {0: (0, (0, 1, 2, 3, 4), 10), 1: (4, (4, 5, 6, 7, 8), 10)},
    "Persons": {10: (0, 6, 8), 11: (0, 4, 5), 12: (8, 6, 0), 13: (8, 5, 3)}
}

# ── Expected Optimal Steps ──

EXPECTED = {
    "p1 (original)": 13, "e1 (easy)": 10, "e2 (easy)": 11, "e3 (easy)": 18,
    "e4 (easy)": 9, "e5 (easy)": 13, "m1 (medium)": 24, "m2 (medium)": 21,
    "m3 (medium)": 22, "m4 (medium)": 16, "m5 (medium)": 25
}

# ── Solver Engine ──

def solve_problems(name, problem_data):
    """Initializes the problem and runs A* search, validating the path cost."""
    p = ex1_331526079.create_elevators_problem(problem_data)
    
    # We expect a tuple back from your search library: (Node, expanded_count)
    result = search.astar_search(p, p.h_astar)
    
    if result:
        final_node, _ = result 
        
        # Reconstruct path from the root
        actions = [node.action for node in final_node.path()[::-1]][1:]
        steps = len(actions)
        target = EXPECTED.get(name)
        
        if steps == target:
            print(f"✅ Correct! | Steps: {steps:<3}", end="")
            return True, steps
        else:
            print(f"❌ Wrong!   | Steps: {steps:<3} (Target: {target})")
            print(f"   Path: {actions}", end="")
            return False, steps
            
    print(f"🚫 No solution for {name}", end="")
    return False, 0

def main():
    problems = [
        ("p1 (original)", init_state_p1), ("e1 (easy)", init_state_e1),
        ("e2 (easy)",     init_state_e2), ("e3 (easy)", init_state_e3),
        ("e4 (easy)",     init_state_e4), ("e5 (easy)", init_state_e5),
        ("m1 (medium)",   init_state_m1), ("m2 (medium)", init_state_m2),
        ("m3 (medium)",   init_state_m3), ("m4 (medium)", init_state_m4),
        ("m5 (medium)",   init_state_m5)
    ]

    print("-" * 70)
    print(f"{'Test Name':<20} | {'Status & Result':<30} | {'Time'}")
    print("-" * 70)

    stats = []
    start_wall_clock = time.time()

    for name, data in problems:
        t_start = time.time()
        print(f"{name:<20} | ", end="", flush=True)
        
        is_ok, count = solve_problems(name, data)
        
        t_end = time.time()
        stats.append((is_ok, t_end - t_start))
        print(f" | {t_end - t_start:.3f}s")

    # ── Final Report ──
    
    total_elapsed = time.time() - start_wall_clock
    passed_tests = sum(1 for s in stats if s[0])
    avg_speed = total_elapsed / len(problems) if problems else 0

    print("-" * 70)
    print("ROBUST TEST SUMMARY:")
    print(f"  Accuracy: {passed_tests}/{len(problems)} Correct")
    print(f"  Duration: {total_elapsed:.2f}s total")
    print(f"  Efficiency: {avg_speed:.3f}s per instance (Average)")
    print("-" * 70)

if __name__ == '__main__':
    main()
