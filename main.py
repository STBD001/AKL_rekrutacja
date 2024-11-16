import numpy as np

# Tworzenie macierzy projekcji perspektywicznej
def create_projection_matrix(fov, aspect_ratio, near, far):
    f = 1 / np.tan(fov / 2)
    depth = near - far

    projection_matrix = np.array([
        [f / aspect_ratio, 0, 0, 0],
        [0, f, 0, 0],
        [0, 0, (far + near) / depth, (2 * far * near) / depth],
        [0, 0, -1, 0]
    ])
    print("Macierz projekcji (Projection Matrix):\n", projection_matrix)
    return projection_matrix


# Przekształcenie współrzędnych ekranu na współrzędne światowe na płaszczyźnie Z=0
def screen_to_world(x, y, projection_matrix_inv, drone_height):
    print("\n--- Przekształcenie ekranu na świat (Screen to World) ---")
    print("Odwrotna macierz projekcji (Inverse Projection Matrix):\n", projection_matrix_inv)

    # Punkt ekranowy z Z=1, dla przekszłcenia przez macierz projekcji
    screen_point = np.array([x, y, 1, 1])
    print("Punkt ekranowy (Screen Point, homogeniczny):", screen_point)

    # Przekształcenie na współrzędne światowe
    world_point = np.dot(projection_matrix_inv, screen_point)
    print("Współrzędne światowe przed normowaniem (World Point before normalization):", world_point)

    # Normalizacja współrzędnych (dzielenie przez współczynnik homogeniczny)
    world_point /= world_point[3]
    print("Współrzędne światowe po normowaniu (World Point after normalization):", world_point)

    # Skalowanie, aby uzyskać Z = 0 w przestrzeni światowej
    scale = -world_point[2] / drone_height
    world_point = world_point * scale
    world_point[2] = 0  # Upewniamy się, że Z = 0
    print("Współrzędne światowe po przeskalowaniu (Z=0):", world_point)

    return world_point[:3]


# Tworzenie macierzy rotacji z kątów Eulera
def euler_angles_to_rotation_matrix(yaw, pitch, roll):
    print("\n--- Tworzenie macierzy rotacji (Rotation Matrix) ---")
    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])
    Ry = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])
    rotation_matrix = np.dot(Rz, np.dot(Ry, Rx))
    print("Macierz rotacji (Rotation Matrix - Yaw, Pitch, Roll):\n", rotation_matrix)
    return rotation_matrix


# Transformacja punktu światowego do globalnego systemu odniesienia
def transform_world_point(world_point, drone_position, camera_rotation):
    print("\n--- Transformacja punktu światowego (World Point Transformation) ---")
    # Konwersja kątów z stopni na radiany
    yaw, pitch, roll = np.radians(camera_rotation)
    # Obliczanie macierzy rotacji
    rotation_matrix = euler_angles_to_rotation_matrix(yaw, pitch, roll)
    print("Macierz rotacji:\n", rotation_matrix)
    # Obrót punktu
    rotated_point = np.dot(rotation_matrix, world_point)
    print("Współrzędne światowe po rotacji (Rotated World Point):", rotated_point)
    # Translacja punktu względem pozycji drona
    global_point = drone_position + rotated_point
    print("Współrzędne globalne po translacji (Global Point):", global_point)
    return global_point


# Przekształcenie współrzędnych światowych na GPS
def to_gps_coordinates(transformed_point, drone_gps_position):
    print("\n--- Przekształcenie na współrzędne GPS (World to GPS) ---")
    gps_coordinates = drone_gps_position[:2] + transformed_point[:2]
    gps_coordinates = np.round(gps_coordinates, 2)  # Zaokrąglamy do 2 miejsc po przecinku
    print("Współrzędne GPS (GPS Coordinates): {:.2f}, {:.2f}".format(gps_coordinates[0], gps_coordinates[1]))
    return gps_coordinates


# Parametry
fov = np.radians(90)  # Kąt widzenia 90 stopni w radianach
aspect_ratio = 16 / 9  # Typowy współczynnik proporcji ekranu
near = 0.1  # Bliska płaszczyzna odcięcia
far = 1000  # Daleka płaszczyzna odcięcia

# Tworzenie macierzy projekcji i jej odwrotności
projection_matrix = create_projection_matrix(fov, aspect_ratio, near, far)
projection_matrix_inv = np.linalg.inv(projection_matrix)

# Przykładowe dane wejściowe
x, y = 512, 384  # Współrzędne ekranu
drone_height = 100  # Wysokość drona nad ziemią
drone_position = np.array([100, 200, 100])  # Pozycja drona w przestrzeni 3D
camera_rotation = [30, 0, 0]  # Kąty Eulera kamery (yaw, pitch, roll)
drone_gps_position = np.array([50.0, 19.0, 100])  # Pozycja GPS drona (latitude, longitude, altitude)

# Pipeline obliczeń
print("\n--- Pipeline obliczeń ---\n")
# 1. Ekran na współrzędne światowe
world_point = screen_to_world(x, y, projection_matrix_inv, drone_height)
# 2. Transformacja punktu względem drona
transformed_point = transform_world_point(world_point, drone_position, camera_rotation)
# 3. Transformacja współrzędnych światowych na GPS
gps_coordinates = to_gps_coordinates(transformed_point, drone_gps_position)

# Wynik
print("\nWspółrzędne GPS końcowe (Final GPS Coordinates): {:.2f}, {:.2f}".format(gps_coordinates[0], gps_coordinates[1]))
