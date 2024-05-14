import pickle
import time

import neat

from simulation import Simulation

EPOCHS = 500
FORCE = 0.04


def run_simulation(genome, config):
    net = neat.nn.FeedForwardNetwork.create(genome, config)
    sim = Simulation(0.5)
    fitness = 0

    for x in range(300):
        sim.step()

        output = net.activate(sim.state())
        decision = output.index(max(output))

        if decision == 0:
            pass
        if decision == 1:
            sim.push(FORCE)
        if decision == 2:
            sim.push(-FORCE)

        fitness -= sim.get_voffset() * 300 + sim.get_offset()

    genome.fitness = fitness


def eval_genomes(genomes, config):
    for i, (genome_id, genome) in enumerate(genomes):
        genome.fitness = 0
        run_simulation(genome, config)


def run_neat(config):
    # p = neat.Checkpointer.restore_checkpoint("neat-checkpoint-199")
    p = neat.Population(config)
    p.add_reporter(neat.StdOutReporter(True))
    stats = neat.StatisticsReporter()
    p.add_reporter(stats)
    p.add_reporter(neat.Checkpointer(50))

    winner = p.run(eval_genomes, EPOCHS)
    with open("best.pickle", "wb") as f:
        pickle.dump(winner, f)


def test_ai(config):
    with open("best.pickle", "rb") as f:
        genome = pickle.load(f)

    net = neat.nn.FeedForwardNetwork.create(genome, config)
    sim = Simulation(0.5)

    while True:
        sim.step()

        output = net.activate(sim.state())
        decision = output.index(max(output))

        if decision == 0:
            pass
        if decision == 1:
            sim.push(FORCE)
        if decision == 2:
            sim.push(-FORCE)

        sim.show()

        time.sleep(0.01)


if __name__ == "__main__":
    config = neat.Config(
        neat.DefaultGenome,
        neat.DefaultReproduction,
        neat.DefaultSpeciesSet,
        neat.DefaultStagnation,
        "neat-config.txt",
    )
    # run_neat(config)
    test_ai(config)
