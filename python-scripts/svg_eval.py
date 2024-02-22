import numpy as np
from svg.path import parse_path, Path, Line
from shapely.geometry import Polygon, LineString, Point
import matplotlib.pyplot as plt
from xml.dom import minidom  # Add this import statement



# def svg_to_shapes(svg_path):
#     shapes = []

#     doc = minidom.parse(svg_path)
#     path_elements = doc.getElementsByTagName('path')
#     polygon_elements = doc.getElementsByTagName('polygon')

#     for path_element in path_elements:
#         d_attribute = path_element.getAttribute('d')
#         path = parse_path(d_attribute)
#         vertices = [(segment.start.real, segment.start.imag) for segment in path]
#         is_closed = any(isinstance(segment, Path) for segment in path)
#         shapes.append((vertices, is_closed))

#     for polygon_element in polygon_elements:
#         points_attribute = polygon_element.getAttribute('points')
#         points = [tuple(map(float, point.split(','))) for point in points_attribute.split()]
#         shape = Polygon(points)
#         is_closed = True  # Assume polygons are always closed
#         shapes.append((shape, is_closed))

#     doc.unlink()

#     return shapes
def svg_to_shapes(svg_path):
    shapes = []

    doc = minidom.parse(svg_path)

    # Parse <path> elements
    path_elements = doc.getElementsByTagName('path')
    for path_element in path_elements:
        d_attribute = path_element.getAttribute('d')
        path = parse_path(d_attribute)
        vertices = [(segment.start.real, segment.start.imag) for segment in path]
        is_closed = any(isinstance(segment, Path) for segment in path)
        shapes.append((vertices, is_closed))

    # Parse <polygon> elements
    polygon_elements = doc.getElementsByTagName('polygon')
    for polygon_element in polygon_elements:
        points_attribute = polygon_element.getAttribute('points')
        points = [tuple(map(float, point.split(','))) for point in points_attribute.split()]
        shape = Polygon(points)
        is_closed = True  # Assume polygons are always closed
        shapes.append((shape, is_closed))

    # Parse <circle> elements
    circle_elements = doc.getElementsByTagName('circle')
    for circle_element in circle_elements:
        cx = float(circle_element.getAttribute('cx'))
        cy = float(circle_element.getAttribute('cy'))
        r = float(circle_element.getAttribute('r'))
        shape = Point(cx, cy).buffer(r)  # Create a circle using a Point and buffer
        shapes.append((shape, True))

    doc.unlink()

    return shapes
# def svg_to_shapes(svg_path):
#     shapes = []

#     doc = minidom.parse(svg_path)

#     # Parse <path> elements
#     path_elements = doc.getElementsByTagName('path')
#     for path_element in path_elements:
#         d_attribute = path_element.getAttribute('d')
#         path = parse_path(d_attribute)
#         vertices = [(segment.start.real, segment.start.imag) for segment in path]
#         is_closed = any(isinstance(segment, Path) for segment in path)
#         shapes.append((vertices, is_closed))

#     # Parse <polygon> elements
#     polygon_elements = doc.getElementsByTagName('polygon')
#     for polygon_element in polygon_elements:
#         points_attribute = polygon_element.getAttribute('points')
#         points = [tuple(map(float, point.split(','))) for point in points_attribute.split()]
#         shape = Polygon(points)
#         is_closed = True  # Assume polygons are always closed
#         shapes.append((shape, is_closed))

#     # Parse <line> elements
#     line_elements = doc.getElementsByTagName('line')
#     for line_element in line_elements:
#         x1 = float(line_element.getAttribute('x1'))
#         y1 = float(line_element.getAttribute('y1'))
#         x2 = float(line_element.getAttribute('x2'))
#         y2 = float(line_element.getAttribute('y2'))
#         shape = LineString([(x1, y1), (x2, y2)])
#         shapes.append((shape, False))

#     # Parse <circle> elements
#     circle_elements = doc.getElementsByTagName('circle')
#     for circle_element in circle_elements:
#         cx = float(circle_element.getAttribute('cx'))
#         cy = float(circle_element.getAttribute('cy'))
#         r = float(circle_element.getAttribute('r'))
#         shape = Point(cx, cy).buffer(r)  # Create a circle using a Point and buffer
#         shapes.append((shape, True))

#     doc.unlink()

#     return shapes

# def visualize_shapes(shapes):
#     for shape in shapes:
#         if isinstance(shape, LineString):
#             x, y = shape.xy
#             plt.plot(x, y, color='black')
#         elif isinstance(shape, Polygon):
#             x, y = shape.exterior.xy
#             plt.fill(x, y, alpha=0.5)
#         else:
#             try:
#                 vertices, is_closed = shape
#                 if is_closed:
#                     plt.fill(*zip(*vertices), alpha=0.5)
#                 else:
#                     if len(vertices) > 0:
#                         x, y = zip(*vertices)
#                         plt.plot(x, y, color='black')
#                     else:
#                         print("Empty vertices encountered.")
#             except ValueError:
#                 print("Error handling shape:", shape)

#     plt.axis('equal')
#     plt.show()

import random
import matplotlib.colors as mcolors

def visualize_shapes(shapes):
    for shape in shapes:
        if isinstance(shape, LineString):
            x, y = shape.xy
            plt.plot(x, y, color=get_random_color())
        elif isinstance(shape, Polygon):
            x, y = shape.exterior.xy
            plt.fill(x, y, alpha=0.5, color=get_random_color())
        elif isinstance(shape, Path):  # Check if it's a Path
            vertices = list(shape.points())
            plt.plot(*zip(*vertices), color=get_random_color())
        else:
            try:
                vertices, is_closed = shape
                if is_closed:
                    plt.fill(*zip(*vertices), alpha=0.5, color=get_random_color())
                else:
                    if len(vertices) > 0:
                        x, y = zip(*vertices)
                        plt.plot(x, y, color=get_random_color())
                    else:
                        print("Empty vertices encountered.")
            except ValueError:
                print("Error handling shape:", shape)

    plt.axis('equal')
    plt.show()

def get_random_color():
    return random.choice(list(mcolors.TABLEAU_COLORS.values()))






def main(svg_path):
    shapes = svg_to_shapes(svg_path)

    # Print the shapes variable
    # print(shapes)

    # Convert shapes to Polygon or LineString objects
    converted_shapes = []
    for vertices, is_closed in shapes:
        if is_closed:
            polygon = Polygon(vertices)
            converted_shapes.append(polygon)
        else:
            if isinstance(vertices, LineString):
                converted_shapes.append(vertices)
            else:
                line_string = LineString(vertices)
                converted_shapes.append(line_string)

    # Visualize the shapes
    visualize_shapes(converted_shapes)


if __name__ == "__main__":
    svg_path = "Traced_anatomical_model-cdr.svg"  # Replace with the path to your SVG file
    main(svg_path)
