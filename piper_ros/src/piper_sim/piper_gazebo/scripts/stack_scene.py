"""Generate separated, reachable cube locations without changing the world template."""
import math
import random
import xml.etree.ElementTree as ET

CUBES = ('pick_cube', 'pick_cube_blue', 'pick_cube_green')


def randomize_world(source: str, destination: str, seed: int | None = None):
    tree = ET.parse(source)
    world = tree.getroot().find('world')
    if world is None:
        raise ValueError('World element missing.')
    rng = random.Random(seed)
    for _ in range(10000):
        positions = [(0.4, 0.0)] + [
            (rng.uniform(0.28, 0.38), rng.uniform(-0.23, -0.06)) for _ in range(2)]
        if all(math.hypot(x, y) <= 0.395 for x, y in positions[1:]) and all(
            math.dist(positions[i], positions[j]) >= 0.105
            for i, j in ((0, 1), (0, 2), (1, 2))
        ):
            break
    else:
        raise RuntimeError('Could not generate separated cube positions.')
    for name, (x, y) in zip(CUBES[1:], positions[1:]):
        model = world.find(f"model[@name='{name}']")
        if model is None:
            raise ValueError(f'Missing cube: {name}')
        pose = model.find('pose')
        if pose is None or pose.text is None:
            raise ValueError(f'Missing initial pose for {name}')
        values = pose.text.split()
        values[0:2] = [str(x), str(y)]
        pose.text = ' '.join(values)
    ET.indent(tree, space='  ')
    tree.write(destination, encoding='unicode', xml_declaration=True)
    return dict(zip(CUBES, positions))
