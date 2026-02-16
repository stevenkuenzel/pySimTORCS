"""
Performance profiling script for pySimTORCS simulation.
Compares execution time and provides detailed performance metrics.
"""

import time
from pysimtorcs import settings
from pysimtorcs.controller import TestController
from pysimtorcs.track import Track, import_from_torcs
from pysimtorcs.race import Race


def run_simulation(num_iterations: int = 1) -> float:
    """Run simulation and return total execution time in seconds."""
    start_time = time.perf_counter()

    for iteration in range(num_iterations):
        num_of_cars = 10
        fps = 30

        track: Track = import_from_torcs("Brondehach", 1)
        race: Race = Race(track, False, False)

        for i in range(num_of_cars):
            target_speed = 10 + i * 5
            race.create_car(TestController(target_speed))

        while race.time_max_sec > race.time_now and not race.race_finished:
            race.update(1.0 / fps)

        print(f"Iteration {iteration + 1}: Time elapsed: {race.time_now:.6f} sec")

    end_time = time.perf_counter()
    total_time = end_time - start_time
    return total_time


if __name__ == "__main__":
    print("=" * 70)
    print("pySimTORCS Performance Benchmark")
    print("=" * 70)
    print()

    # Warmup run (Numba JIT compilation happens here)
    print("Warmup run (JIT compilation)...")
    warmup_time = run_simulation(1)
    print(f"Warmup completed in {warmup_time:.3f} seconds")
    print()

    # Main benchmark
    print("Running benchmark (3 iterations)...")
    benchmark_time = run_simulation(3)
    avg_time = benchmark_time / 3

    print()
    print("=" * 70)
    print(f"Total benchmark time: {benchmark_time:.3f} seconds")
    print(f"Average time per simulation: {avg_time:.3f} seconds")
    print("=" * 70)
    print()
    print("Performance notes:")
    print(f"- Each simulation runs 10 cars for 300 seconds at 30 FPS")
    print(f"- Total simulation steps per run: {300 * 30} = 9000 steps")
    print(f"- Total cars simulated: 10 × 3 = 30 car-simulations")
    print(f"- Total physics updates: {30 * 9000:,}")
