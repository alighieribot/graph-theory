"""
pedro_henrique_oliveira_francisco [2022003245]
ecom02a - teoria_dos_grafos - t01

script: cria n='(fps*video_length)' frames de grafos aleatórios e os compila em um vídeo de 'video_length' segundos.
1. cria frames de grafos, onde cada vértice é um par (x, y) com posições aleatórias
e cada aresta é uma linha cinza entre vértices cuja distância é menor que 'max_edge_distance'.
utiliza o algoritmo de kruskal para encontrar a árvore geradora mínima do grafo, que é destacada por linhas vermelhas mais grossas.
2. lê os arquivos de imagem e os compila em um vídeo usando o opencv.
"""

import cv2
import numpy as np
import random
import os
from math import sqrt

# configurações do vídeo e dos grafos
video_length = 30  # duração do vídeo em segundos
fps = 30  # taxa de quadros por segundo
n_frames = int(video_length * fps)  # número total de quadros
min_vertex_distance = 100  # distância mínima entre vértices
max_edge_distance = 200  # distância máxima para criação de arestas
vertex_radius = 20  # raio dos vértices
edge_width = 2  # largura das arestas
mst_width = 4  # largura das linhas da MST
white = (255, 255, 255)  # cor branca
gray = (169, 169, 169)  # cor cinza
red = (255, 0, 0)  # cor vermelha
width = 1080  # largura do vídeo
height = 1920  # altura do vídeo
vertex_count = 100  # número de vértices do grafo

def kruskal(vertices, frame_number):
    """
    implementa o algoritmo de Kruskal, retorna uma lista de arestas pertencentes à MST parcial até o quadro atual,
    juntamente com o comprimento total da MST.
    """
    edges = [
        ((x0, y0), (x1, y1), sqrt((x0 - x1) ** 2 + (y0 - y1) ** 2))
        for i, (x0, y0) in enumerate(vertices)
        for x1, y1 in vertices[i + 1 :]
    ]
    edges.sort(key=lambda x: x[2])

    parent = list(range(len(vertices)))

    def find(i):
        if i != parent[i]:
            parent[i] = find(parent[i])
        return parent[i]

    mst = []
    for (x0, y0), (x1, y1), _ in edges:
        i = vertices.index((x0, y0))
        j = vertices.index((x1, y1))
        if find(i) != find(j):
            parent[find(i)] = find(j)
            mst.append(((x0, y0), (x1, y1)))

    mst_portion = mst[: min(frame_number + 1, len(mst))]

    return mst_portion, len(mst)
def create_frames(n_frames):
    """
    gera os quadros individuais do vídeo, desenha os vértices como círculos brancos, as arestas como linhas cinzas
    e a MST parcial até o quadro atual como linhas vermelhas.
    """
    vertices = []
    while len(vertices) < vertex_count:
        x = random.randint(0, width)
        y = random.randint(0, height)

        if not any(
            sqrt((x - x0) ** 2 + (y - y0) ** 2) < min_vertex_distance
            for x0, y0 in vertices
        ):
            vertices.append((x, y))

    mst_length = 0
    for frame_number in range(n_frames):
        img = np.zeros((height, width, 3), dtype="uint8")

        for x, y in vertices:
            cv2.circle(img, (x, y), vertex_radius, white, -1)

        for i, (x0, y0) in enumerate(vertices):
            for x1, y1 in vertices[i + 1:]:
                if sqrt((x0 - x1) ** 2 + (y0 - y1) ** 2) < max_edge_distance:
                    cv2.line(img, (x0, y0), (x1, y1), gray, edge_width)

        mst, full_mst_length = kruskal(vertices, frame_number)
        mst_length = max(mst_length, full_mst_length)
        for (x0, y0), (x1, y1) in mst:
            cv2.line(img, (x0, y0), (x1, y1), red, mst_width)

        cv2.imwrite(f"cache/frame_{frame_number}.png", img)

    return mst_length
def create_video(fps, n_frames):
    """
    cria o vídeo a partir dos quadros gerados.
    """
    frame = cv2.imread("cache/frame_0.png")
    h, w, layers = frame.shape

    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    video = cv2.VideoWriter("graph_video.mp4", fourcc, fps, (w, h))

    for i in range(n_frames):
        video.write(cv2.imread(f"cache/frame_{i}.png"))

    cv2.destroyAllWindows()
    video.release()

    for i in range(n_frames):
        os.remove(f"cache/frame_{i}.png")

mst_length = create_frames(n_frames)
n_frames = mst_length + 5 * fps

create_frames(n_frames)
create_video(fps, n_frames)
