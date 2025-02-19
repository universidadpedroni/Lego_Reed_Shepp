from vehicleConstants import r_turn_min
from reeds_shepp import get_all_paths, get_optimal_path, normalize_start_and_end_point, denormalize_distance_in_path



def test_reeds_shepp():
   # Variables de entrada
    start_point_original = [0, 0, 0]
    end_point_original = [120, 60, -90]

    # Normalizar el punto final
    start_point, end_point = normalize_start_and_end_point(start_point_original, end_point_original, r_turn_min)
  
    print(start_point)
    print(end_point)
    # Obtener los caminos
    paths = get_all_paths(start_point, end_point)

    # Obtener el índice del camino óptimo
    min_distance_index = get_optimal_path(paths)

    # Obtener el camino óptimo
    path = paths[min_distance_index]

    # Escalar las distancias
    #for p in path:
    #    p['distance'] = p['distance'] * r_turn_min  # Escalar la distancia real
    path = denormalize_distance_in_path(path, r_turn_min)

    # Mensaje final
    print('Terminado') 

if __name__ == '__main__':
    print('TESTING')
    test_reeds_shepp()