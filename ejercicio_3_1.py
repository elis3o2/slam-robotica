from read_file3d import leer_g2o, matriz_info


if __name__ == "__main__":
    poses, edges = leer_g2o("parking-garage.g2o")
    print("Poses:", len(poses))
    print("Aristas:", len(edges))
    print(matriz_info(edges[0, 9:]))
