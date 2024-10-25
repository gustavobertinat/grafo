import numpy as np

class Grafo:
    def __init__(self):
        self.cantNodos = 0
        self.nodos = []
        self.matrizAdyacencia = []

    # Agrega un nuevo nodo al grafo
    def agregarNodo(self, nodo):
        self.nodos.append(nodo)
        self.cantNodos += 1
        if (self.cantNodos == 1):
          self.matrizAdyacencia = np.array([[0]])
        else: 
          nuevaFila = np.zeros(self.cantNodos-1)                           # Crea un arreglo con ceros
          # nuevaColumna = np.transpose(np.zeros(self.cantNodos))
          # nuevaColumna = np.append(nuevaFila, np.zeros(1), axis=0)                      # Crea un arreglo con ceros
          nuevaColumna = np.zeros(self.cantNodos).reshape((self.cantNodos, 1))                     # Crea un arreglo con ceros
          self.matrizAdyacencia = np.append(self.matrizAdyacencia, nuevaFila)           # Agrega una nueva fila
          self.matrizAdyacencia = np.append(self.matrizAdyacencia, nuevaColumna).reshape(self.cantNodos,self.cantNodos)         # Agrega una nueva columna

          #b = np.insert(self.matrizAdyacencia, 3, values=0, axis=1)
          #print("MA:\n", self.matrizAdyacencia)

    # Agrega una nueva arista al grafo
    def agregarArista(self, nodoOrigen, nodoDestino, valor):
        indiceOrigen = self.nodos.index(nodoOrigen)
        indiceDestino = self.nodos.index(nodoDestino)
        self.matrizAdyacencia[indiceOrigen][indiceDestino] = valor

    # Muestra la matriz de adyascencia del grafo
    def mostrarMatrizAdyacencia(self):
        print("Matriz de Adyacencia\n")
        for f in self.matrizAdyacencia:
            print(f)

    # Calcula el camino más corto desde un nodo origen a los demás nodos
    def dijkstra(self, nodoOrigen):
        indiceOrigen = self.nodos.index(nodoOrigen)
        distancias = [0] * len(self.nodos)
        distancias[indiceOrigen] = 0
        visitados = [False] * len(self.nodos)

        for _ in range(len(self.nodos)):
            minDistancia = 99999999
            minIndice = -1
            for i in range(len(self.nodos)):
                if not visitados[i] and distancias[i] < minDistancia:
                    minDistancia = distancias[i]
                    minIndice = i

            visitados[minIndice] = True

            for i in range(len(self.nodos)):
                if (not visitados[i] and
                        self.matrizAdyacencia[minIndice][i] != 0 and
                        distancias[minIndice] + self.matrizAdyacencia[minIndice][i] < distancias[i]):
                    distancias[i] = distancias[minIndice] + self.matrizAdyacencia[minIndice][i]

        print("El camino más corto desde ", nodoOrigen)
        for i in range(len(self.nodos)):
            if i != nodoOrigen:
                print(" hasta ", self.nodos[i], ":", distancias[i])

    # Se aplica el algoritmo de Dikstra para hallar e imprimir la distancia de los caminos más cortos desde el origen hasta los otros nodos
    def dijkstra(self, nodoOrigen):
        indiceOrigen = self.nodos.index(nodoOrigen)
        distancias = [999999999] * len(self.nodos)
        distancias[indiceOrigen] = 0
        visitados = [False] * len(self.nodos)

        for _ in range(len(self.nodos)):
            min_distancia = 999999999     # este número se indica cuando el nodo es inalcanzable
            min_indice = -1
            for i in range(len(self.nodos)):
                if not visitados[i] and distancias[i] < min_distancia:
                    min_distancia = distancias[i]
                    min_indice = i

            visitados[min_indice] = True

            for i in range(len(self.nodos)):
                if (not visitados[i] and
                        self.matrizAdyacencia[min_indice][i] != 0 and
                        distancias[min_indice] + self.matrizAdyacencia[min_indice][i] < distancias[i]):
                    distancias[i] = distancias[min_indice] + self.matrizAdyacencia[min_indice][i]

        print("Camino más corto desde", nodoOrigen)
        for i in range(len(self.nodos)):
            if i != indiceOrigen:
                print("Hasta", self.nodos[i], ":", distancias[i])

# Ejemplo de uso
grafo = Grafo()
grafo.agregarNodo("N1")
grafo.agregarNodo("N2")
grafo.agregarNodo("N3")
grafo.agregarNodo("N4")

grafo.agregarArista("N1", "N2", 3)
grafo.agregarArista("N1", "N3", 5)
grafo.agregarArista("N2", "N3", 1)
grafo.agregarArista("N2", "N4", 6)
grafo.agregarArista("N3", "N4", 2)

grafo.mostrarMatrizAdyacencia()
grafo.dijkstra("N1")