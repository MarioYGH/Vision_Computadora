import cv2
import numpy as np
import matplotlib.pyplot as plt

from PIL import Image

def rgb2gray(img):
    if img.shape[2] == 3:
        return 0.2989 * img[:,:,0] + 0.5870 * img[:,:,1] + 0.1140 * img[:,:,2]
    return img

if __name__ == '__main__':
    # img = cv2.imread('Examen.jpg')
    img_rgb = np.array(Image.open('Examen.jpg'))
    img_gray = rgb2gray(img_rgb)
    
    plt.figure()
    plt.imshow(img_rgb)
    plt.show()
    
    plt.figure()
    plt.imshow(img_gray, cmap = 'gray')
    plt.show()
    
    # 1. Ruido Gaussiano 
    mu = 0
    sigma = 2.3 # [0.3, 2.3]
    gauss_noise = np.random.normal(mu, sigma, img_gray.shape)
    
    img_gray_noise = img_gray + gauss_noise
    
    print("MSE: ", np.mean((img_gray - img_gray_noise) ** 2))
    
    plt.figure()
    plt.imshow(img_gray_noise, cmap = 'gray')
    plt.axis('off')
    plt.show()
    
    # 2. Ruid Sal y Pimienta
    s_vs_p = 0.5
    amount = 0.04 #Menos ceros, más ruido, Cantidad de ruido S y P
    
    img_sp = np.copy(img_gray)
    
    # Modo sal
    num_salt = np.ceil(img_gray.size * amount * s_vs_p)
    coords = [np.random.randint(0, i-1, int(num_salt))for i in img_gray.shape]
    
    img_sp[tuple(coords)] = 255
    
    # Modo pimienta
    num_peper = np.ceil(img_gray.size * amount * (1 - s_vs_p))
    coords = [np.random.randint(0, i-1, int(num_salt))for i in img_gray.shape]
    
    img_sp[tuple(coords)] = 0
    
    plt.figure()
    plt.imshow(img_sp, cmap = 'gray')
    plt.axis('off')
    plt.show()
    
    print("MSE: ", np.mean((img_gray - img_sp) ** 2))
    
    # 3. Ruido Poisson
    vals = len(np.unique(img_gray))
    vals = 2 ** np.ceil(np.log2(vals))
    img_poisson = np.random.poisson(img_gray * vals) / vals
    
    plt.figure()
    plt.imshow(img_poisson, cmap = 'gray')
    plt.axis('off')
    plt.show()
    
    print("MSE: ", np.mean((img_gray - img_poisson) ** 2))
