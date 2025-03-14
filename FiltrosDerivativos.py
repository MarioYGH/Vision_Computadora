import cv2 as cv
import numpy as np

from matplotlib import pyplot as plt

if __name__ == '__main__':
    img = cv.imread('Yo.jpg', cv.IMREAD_GRAYSCALE)

    laplacian = cv.Laplacian(img, cv.CV_64F)
    sobelx = cv.Sobel(img, cv.CV_64F, 1, 0, ksize=3)
    sobely = cv.Sobel(img, cv.CV_64F, 0, 1, ksize=3)

    plt.subplot(2, 2, 1)
    plt.imshow(img, cmap='gray')
    plt.title('Original')
    plt.axis('off')

    plt.subplot(2, 2, 2)
    plt.imshow(laplacian, cmap='gray')
    plt.title('Laplacian')
    plt.axis('off')

    plt.subplot(2, 2, 3)
    plt.imshow(sobelx, cmap='gray')
    plt.title('Sobel X')
    plt.axis('off')

    plt.subplot(2, 2, 4)
    plt.imshow(sobely, cmap='gray')
    plt.title('Sobel Y')
    plt.axis('off')

    plt.show()
