import random
import numpy as np

from matplotlib.colors import to_hex


def generate_random_points(radius: int, num_points: int):
    """
    Generates an array of random points in the vicinity of a circle with a given radius.

    Parameters:
    - radius (float): The radius of the circle.
    - num_points (int): The number of random points to generate.

    Returns:
    - np.ndarray: An array of shape (num_points, 2) containing the (x, y) coordinates of the random points.
    """
    points = []

    for _ in range(num_points):
        # Random angle in radians
        angle = random.uniform(0, 2 * np.pi)

        # Random radius, slightly more or less than the given radius
        r = random.uniform(0.9 * radius, 1.1 * radius)

        # Convert polar coordinates to Cartesian coordinates
        x = r * np.cos(angle)
        y = r * np.sin(angle)

        points.append((x, y))

    return np.array(points)


def point_generator(radius: int, num_points: int):
    # Angle between two points in radians (360 degrees divided by number of points)
    # TODO: rewrite to evenly distributed points by radius
    # angle_step = 2 * np.pi / num_points
    for n in range(num_points):
        # Calculate the angle for the i-th point
        # angle = n * angle_step
        angle = random.uniform(0, 2 * np.pi)

        # Random radius, slightly more or less than the given radius
        r = random.uniform(0.1 * radius, 1.0 * radius)

        # Convert polar coordinates to Cartesian coordinates
        x = r * np.cos(angle)
        y = r * np.sin(angle)

        yield x, y


def color_generator():
    """
    Generates a new color for a graph.

    Yields:
    - str: A color code for the graph.
    """
    # Predefined list of colors
    colors = ['b', 'g', 'r', 'c', 'm', 'y', 'k']

    # Cycle through predefined colors first
    for color in colors:
        yield color

    # After exhausting predefined colors, generate random colors
    while True:
        yield (random.random(), random.random(), random.random())


def number_generator():
    num = 0
    while True:
        yield num
        num += 1


def green_to_yellow(steps):
    colors = []
    steps += 1
    for step in range(steps):
        red_ratio = step / (steps - 1)  # Normalize to range [0, 1]
        colors.append(to_hex([red_ratio, 1.0, 0.0]))   # RGB tuple
    return colors

def normalize(arr, lim):
    result = np.copy(arr)
    min = np.min(result)
    max = np.max(result)
    for i, val in enumerate(result):
        # Calculate the average for the row
        avg = abs((min + max) / 2)
        # Replace values greater than the average with MAGIC_VALUE. For example 42
        if abs(val) > lim:
            result[i] = avg * np.sign(val)
    return result