import numpy as np

def create_projection_matrix(fov, aspect_ratio, near, far):
    f = 1 / np.tan(fov / 2)
    depth = near - far

    projection_matrix = np.array([
        [f / aspect_ratio, 0, 0, 0],
        [0, f, 0, 0],
        [0, 0, (far + near) / depth, (2 * far * near) / depth],
        [0, 0, -1, 0]
    ])

    return projection_matrix

# Przykładowe parametry
fov = np.radians(90)  # Kąt widzenia 90 stopni, przekształcony na radiany
aspect_ratio = 16 / 9  # Typowy współczynnik proporcji (zależności od rozdzielczości na ekranie)
near = 0.1  # Bliska płaszczyzna odcięcia (najbliższy punkt w przestrzeni, jaki kamera będzie w stanie "zobaczyć")
far = 1000  # Daleka płaszczyzna odcięcia (maksymalna odległość od kamery, poza którą obiekty przestają być renderowane)

# Tworzenie macierzy projekcji
projection_matrix = create_projection_matrix(fov, aspect_ratio, near, far)
print("Macierz projekcji perspektywicznej:\n", projection_matrix)

#Tworzenie macierzy odwrotnej
A_projection_matrix=np.linalg.inv(projection_matrix)
print("Macierz odwrotna projkecji perspektywicznej:\n", A_projection_matrix)