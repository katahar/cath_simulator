# import ezdxf
# import math

# def extract_points_from_closed_shape(entity):
#     points = []
    
#     if entity.dxftype() == 'LWPOLYLINE' and entity.is_closed:
#         for vertex in entity.vertices():
#             points.append((vertex[0], vertex[1]))
        
#         # Close the shape by adding the first point again
#         points.append(points[0])
    
#     return points

# def convert_dxf_to_points(file_path):
#     doc = ezdxf.readfile(file_path)
    
#     for entity in doc.modelspace():
#         points = extract_points_from_closed_shape(entity)
        
#         if points:
#             print(f"Entity Type: {entity.dxftype()}, Points: {points}")

# def generate_points_for_circle(circle_entity, num_points=100):
#     # Generate points along the circumference of the circle
#     center = (circle_entity.dxf.center.x, circle_entity.dxf.center.y)
#     radius = circle_entity.dxf.radius
#     angle_increment = 360 / num_points
    
#     points = [(center[0] + radius * math.cos(math.radians(angle)),
#                center[1] + radius * math.sin(math.radians(angle)))
#               for angle in range(0, 360, int(angle_increment))]
    
#     # Close the circle
#     points.append(points[0])
    
#     return points

# def generate_points_for_arc(arc_entity, num_points=100):
#     # Generate points along the arc
#     center = (arc_entity.dxf.center.x, arc_entity.dxf.center.y)
#     radius = arc_entity.dxf.radius
#     start_angle = arc_entity.dxf.start_angle
#     end_angle = arc_entity.dxf.end_angle
#     angle_increment = (end_angle - start_angle) / num_points
    
#     points = [(center[0] + radius * math.cos(math.radians(start_angle + angle)),
#                center[1] + radius * math.sin(math.radians(start_angle + angle)))
#               for angle in range(0, int(end_angle - start_angle), int(angle_increment))]
    
#     # Add the endpoint of the arc
#     points.append((center[0] + radius * math.cos(math.radians(end_angle)),
#                    center[1] + radius * math.sin(math.radians(end_angle))))
    
#     return points

# # Replace 'your_file.dxf' with the path to your DXF file
# convert_dxf_to_points('Traced_anatomical_model.dxf')


import ezdxf
import numpy as np

def convert_dxf_to_points(file_path):
    doc = ezdxf.readfile(file_path)
    points = []

    for entity in doc.modelspace().query('LWPOLYLINE'):
        # process LWPOLYLINE entity
        for vertex in entity.vertices():
            points.append((vertex.dxf.location.x, vertex.dxf.location.y, vertex.dxf.location.z))

    for entity in doc.modelspace().query('CIRCLE'):
        # process CIRCLE entity
        center = (entity.dxf.center.x, entity.dxf.center.y, entity.dxf.center.z)
        radius = entity.dxf.radius
        theta = np.linspace(0, 2*np.pi, 100)  # Example: divide circle into 100 points
        circle_points = [(center[0] + radius * np.cos(angle), center[1] + radius * np.sin(angle), center[2]) for angle in theta]
        points.extend(circle_points)


    for entity in doc.modelspace().query('SPLINE'):
        # process SPLINE entity
        control_points = [(ctl.x, ctl.y, ctl.z) for ctl in entity.control_points()]
        weights = entity.weights()
        # Calculate points on the spline and append to the list using control points and weights

    return points

def main():
    file_path = 'Traced_anatomical_model.dxf'
    points = convert_dxf_to_points(file_path)
    print(points)

if __name__ == "__main__":
    main()
