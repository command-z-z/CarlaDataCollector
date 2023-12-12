def draw_3d_bounding_box(array, vertices_pos2d):
    """Draws a 3D bounding box on an image.

    This function connects vertices of a 3D bounding box and draws lines between them on a 2D image array.
    It uses a vertex graph to determine which vertices are connected and then draws lines between these connected vertices.

    Args:
        array (numpy.ndarray): The image array on which the bounding box will be drawn.
        vertices_pos2d (list): A list of 2D positions (tuples) of the vertices of the bounding box.

    """
    vertex_graph = {0: [1, 2, 4],
                    1: [0, 3, 5],
                    2: [0, 3, 6],
                    3: [1, 2, 7],
                    4: [0, 5, 6],
                    5: [1, 4, 7],
                    6: [2, 4, 7]}
    # Note that this can be sped up by not drawing duplicate lines
    for vertex_idx in vertex_graph:
        neighbour_idxs = vertex_graph[vertex_idx]
        from_pos2d = vertices_pos2d[vertex_idx]
        for neighbour_idx in neighbour_idxs:
            to_pos2d = vertices_pos2d[neighbour_idx]
            if from_pos2d is None or to_pos2d is None:
                continue
            y1, x1 = from_pos2d[0], from_pos2d[1]
            y2, x2 = to_pos2d[0], to_pos2d[1]
            # Only stop drawing lines if both are outside
            if not point_in_canvas((y1, x1)) and not point_in_canvas((y2, x2)):
                continue
            for x, y in get_line(x1, y1, x2, y2):
                if point_in_canvas((y, x)):
                    array[int(y), int(x)] = (255, 0, 0)


def get_line(x1, y1, x2, y2):
    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
    # print("Calculating line from {},{} to {},{}".format(x1,y1,x2,y2))
    points = []
    issteep = abs(y2 - y1) > abs(x2 - x1)
    if issteep:
        x1, y1 = y1, x1
        x2, y2 = y2, x2
    rev = False
    if x1 > x2:
        x1, x2 = x2, x1
        y1, y2 = y2, y1
        rev = True
    deltax = x2 - x1
    deltay = abs(y2 - y1)
    error = int(deltax / 2)
    y = y1
    if y1 < y2:
        ystep = 1
    else:
        ystep = -1
    for x in range(x1, x2 + 1):
        if issteep:
            points.append((y, x))
        else:
            points.append((x, y))
        error -= deltay
        if error < 0:
            y += ystep
            error += deltax
    # Reverse the list if the coordinates were reversed
    if rev:
        points.reverse()
    return points


def point_in_canvas(pos):
    """Determines if a point is within the bounds of a canvas.

    This function checks whether the given point lies within a specific rectangular area (the canvas).
    The canvas is assumed to have dimensions 360x720.

    Args:
        pos (tuple): A tuple representing the (x, y) coordinates of the point.

    Returns:
        bool: True if the point is within the canvas, False otherwise.

    """
    if (pos[0] >= 0) and (pos[0] < 360) and (pos[1] >= 0) and (pos[1] < 720):
        return True
    return False
