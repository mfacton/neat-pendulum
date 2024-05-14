import time

from pynput import keyboard

from simulation import Simulation

RUNNING = True
KEYS = []
FORCE = 0.01


def on_press(key):
    try:
        KEYS.append(key.char)
    except AttributeError:
        pass


def on_release(key):
    try:
        KEYS.remove(key.char)
    except AttributeError:
        pass

    if key == keyboard.Key.esc:
        RUNNING = False
        return False


if __name__ == "__main__":
    sim = Simulation(0.5)

    fitness = 0
    with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
        for x in range(250):
            sim.step()
            fitness -= sim.get_offset()
            print(fitness)
            sim.show()

            if KEYS.__contains__("a"):
                sim.push(-FORCE)
            if KEYS.__contains__("d"):
                sim.push(FORCE)
            time.sleep(0.02)

        listener.join()
