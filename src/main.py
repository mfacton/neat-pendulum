import time

from simulation import Simulation

if __name__ == "__main__":
    sim = Simulation()
    while True:
        sim.step(0.5)
        sim.draw()
        time.sleep(0.01)
