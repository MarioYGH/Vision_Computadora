import cv2 as cv
import numpy as np

if __name__ == '__main__':
    # Abrimos la camara por defecto
    cam = cv.VideoCapture(0)
    
    while True:
        ret, frame = cam.read()

        # Mostramos el cuadro capturado
        cv.imshow('Camara', frame)
        
        #Presiona 'q' para salir del ciclo
        if cv.waitKey(1) == ord('q'):
            break
        
    # Liberamos la camara
    cam.release()
    cv.destroyAllWindows()
