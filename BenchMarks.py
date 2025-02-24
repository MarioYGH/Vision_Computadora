import cv2
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image

def rgb2gray(img):
    if img.shape[2] == 3:
        return 0.2989 * img[:,:,0] + 0.5870 * img[:,:,1] + 0.1140 * img[:,:,2]
    return img

def add_gaussian_noise(image, mu=0, sigma=25):
    noise = np.random.normal(mu, sigma, image.shape)
    noisy_image = image + noise
    noisy_image = np.clip(noisy_image, 0, 255)
    return noisy_image

def add_salt_pepper_noise(image, amount=0.04, s_vs_p=0.5):
    noisy_image = np.copy(image)
    # Modo sal
    num_salt = np.ceil(amount * image.size * s_vs_p)
    coords = [np.random.randint(0, i-1, int(num_salt)) for i in image.shape]
    noisy_image[tuple(coords)] = 255
    # Modo pimienta
    num_pepper = np.ceil(amount * image.size * (1 - s_vs_p))
    coords = [np.random.randint(0, i-1, int(num_pepper)) for i in image.shape]
    noisy_image[tuple(coords)] = 0
    return noisy_image

def add_poisson_noise(image):
    vals = len(np.unique(image))
    vals = 2 ** np.ceil(np.log2(vals))
    noisy_image = np.random.poisson(image * vals) / vals
    noisy_image = np.clip(noisy_image, 0, 255)
    return noisy_image

def add_speckle_noise(image, noise_level=0.5):
    noise = np.random.normal(0, noise_level, image.shape)
    noisy_image = image + image * noise
    noisy_image = np.clip(noisy_image, 0, 255)
    return noisy_image

def calculate_mse(original, noisy):
    return np.mean((original - noisy) ** 2)

def calculate_psnr(original, noisy):
    mse = calculate_mse(original, noisy)
    if mse == 0:
        return float('inf')
    max_pixel = 255.0
    psnr = 20 * np.log10(max_pixel / np.sqrt(mse))
    return psnr

def benchmark_1(original_image, noisy_image):
    mse = calculate_mse(original_image, noisy_image)
    psnr = calculate_psnr(original_image, noisy_image)
    print(f"MSE: {mse}")
    print(f"PSNR: {psnr} dB")
    plt.figure(figsize=(10, 5))
    plt.subplot(1, 2, 1)
    plt.imshow(original_image, cmap='gray')
    plt.title('Original Image')
    plt.axis('off')
    plt.subplot(1, 2, 2)
    plt.imshow(noisy_image, cmap='gray')
    plt.title('Noisy Image')
    plt.axis('off')
    plt.show()

def benchmark_2(image, noise_type, noise_level=None):
    if noise_type == 'gaussian':
        noisy_image = add_gaussian_noise(image, sigma=noise_level)
    elif noise_type == 'salt_pepper':
        noisy_image = add_salt_pepper_noise(image, amount=noise_level)
    elif noise_type == 'poisson':
        noisy_image = add_poisson_noise(image)
    elif noise_type == 'speckle':
        noisy_image = add_speckle_noise(image, noise_level=noise_level)
    else:
        raise ValueError("Tipo de ruido no válido.")
    
    mse = calculate_mse(image, noisy_image)
    psnr = calculate_psnr(image, noisy_image)
    print(f"MSE: {mse}")
    print(f"PSNR: {psnr} dB")
    plt.figure(figsize=(10, 5))
    plt.subplot(1, 2, 1)
    plt.imshow(image, cmap='gray')
    plt.title('Original Image')
    plt.axis('off')
    plt.subplot(1, 2, 2)
    plt.imshow(noisy_image, cmap='gray')
    plt.title(f'Noisy Image ({noise_type})')
    plt.axis('off')
    plt.show()

if __name__ == '__main__':
    # Cargar la imagen
    img_rgb = np.array(Image.open('Examen.jpg'))
    img_gray = rgb2gray(img_rgb)
    
    # Mostrar la imagen original
    plt.figure()
    plt.imshow(img_gray, cmap='gray')
    plt.title('Original Image')
    plt.axis('off')
    plt.show()
    
    # Benchmark 1: Comparar imagen con y sin ruido
    print("Benchmark 1: Comparar imagen con y sin ruido")
    img_gaussian_noise = add_gaussian_noise(img_gray, sigma=25)
    benchmark_1(img_gray, img_gaussian_noise)
    
    # Benchmark 2: Seleccionar tipo de ruido y cantidad
    print("Benchmark 2: Seleccionar tipo de ruido y cantidad")
    noise_type = 'speckle'  # Cambiar a 'gaussian', 'salt_pepper', 'poisson' o 'speckle'
    noise_level = 0.5  # Ajustar según el tipo de ruido
    benchmark_2(img_gray, noise_type, noise_level)
