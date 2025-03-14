import cv2 as cv
import numpy as np

if __name__ == '__main__':
    # Abrimos la camara por defecto
    cam = cv.VideoCapture(0)
    
    while True:
        ret, frame = cam.read()
        
        frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        
        sobelx = cv.Sobel(frame, cv.CV_64F, 1, 0, ksize=3)
        sobely = cv.Sobel(frame, cv.CV_64F, 0, 1, ksize=3)
        mag_frame = np.sqrt((sobelx ** 2) + (sobely ** 2))
        laplacian = cv.Laplacian(frame, cv.CV_64F)

        
        # Mostramos el cuadro capturado
        cv.imshow('Camara', laplacian)
        
        #Presiona 'q' para salir del ciclo
        if cv.waitKey(1) == ord('q'):
            break
        
    # Liberamos la camara
    cam.release()
    cv.destroyAllWindows()
