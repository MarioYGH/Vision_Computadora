import cv2
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image

if __name__ == '__main__':
    image_path = 'Examen.jpg'
    
    # Cargar imagen en BGR
    img_bgr = cv2.imread(image_path)
    
    # Convertir la imagen a escala de grises
    img_gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
    
    # Aplicar Canny
    lower_thresh = 50
    upper_thresh = 150
    edges = cv2.Canny(img_gray, lower_thresh, upper_thresh)
    
    # Mostrar imágenes
    plt.figure()
    plt.imshow(cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB))
    plt.title("Imagen Original")
    plt.axis('off')
    plt.show()
    
    plt.figure()
    plt.imshow(img_gray, cmap='gray')
    plt.title("Imagen en Escala de Grises")
    plt.axis('off')
    plt.show()
    
    plt.figure()
    plt.imshow(edges, cmap='gray')
    plt.title("Detección de Bordes con Canny")
    plt.axis('off')
    plt.show()
