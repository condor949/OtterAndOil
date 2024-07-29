import numpy as np
import random


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
