#python 3.7.1
import heapq

class Grafo:
    def __init__(self, V):
        self.V = V
        self.adj = [[] for _ in range(V)]
        self.anterior = [-1] * V

    def addAresta(self, v1, v2, custo):
        self.adj[v1].append((v2, custo))

    def dijkstra(self, orig, dest):
        dist = [float('inf')] * self.V
        visitados = [False] * self.V
        pq = [(0, orig)]
        dist[orig] = 0

        while pq:
            d, u = heapq.heappop(pq)

            if not visitados[u]:
                visitados[u] = True

                for v, custo_aresta in self.adj[u]:
                    if dist[v] > dist[u] + custo_aresta:
                        dist[v] = dist[u] + custo_aresta
                        self.anterior[v] = u  
                        heapq.heappush(pq, (dist[v], v))

        # Rastreia o caminho mínimo
        caminho_minimo = []
        atual = dest
        while atual != -1:
            caminho_minimo.append(atual)
            atual = self.anterior[atual]
        caminho_minimo.reverse()

        return dist[dest], caminho_minimo

if __name__ == "__main__":
    g = Grafo(5)

    g.addAresta(0, 1, 4)
    g.addAresta(0, 2, 2)
    g.addAresta(0, 3, 5)
    g.addAresta(1, 4, 1)
    g.addAresta(2, 1, 1)
    g.addAresta(2, 3, 2)
    g.addAresta(2, 4, 1)
    g.addAresta(3, 4, 1)

    distancia_minima, caminho_minimo = g.dijkstra(0, 4)
    print("Distância mínima:", distancia_minima)
    print("Caminho mínimo:", caminho_minimo)
    
    