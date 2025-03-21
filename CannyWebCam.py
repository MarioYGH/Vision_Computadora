import cv2
import numpy as np
import customtkinter as ctk
from PIL import Image, ImageTk

def update_canny(value):
    global lower_thresh, upper_thresh
    lower_thresh = int(lower_slider.get())
    upper_thresh = int(upper_slider.get())
    lower_label.configure(text=f"Lower: {lower_thresh}")
    upper_label.configure(text=f"Upper: {upper_thresh}")

def toggle_camera():
    global running
    running = not running
    if running:
        start_button.configure(text="Detener")
        update_frame()
    else:
        start_button.configure(text="Iniciar")

def update_frame():
    if running:
        ret, frame = cam.read()
        if ret:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(gray, lower_thresh, upper_thresh)
            edges_rgb = cv2.cvtColor(edges, cv2.COLOR_GRAY2RGB)
            img = Image.fromarray(edges_rgb)
            imgtk = ImageTk.PhotoImage(image=img)
            video_label.imgtk = imgtk
            video_label.configure(image=imgtk)
        
        app.after(10, update_frame)

# Configurar la ventana principal
app = ctk.CTk()
app.title("Canny Edge Detection")
app.geometry("600x500")

# Inicializar cámara
cam = cv2.VideoCapture(0)
if not cam.isOpened():
    print("Error: No se pudo abrir la cámara.")
    exit()

# Inicializar valores
lower_thresh = 50
upper_thresh = 150
running = False

# Crear interfaz gráfica
video_label = ctk.CTkLabel(app, text="Cámara no disponible")
video_label.pack(pady=10)

lower_frame = ctk.CTkFrame(app)
lower_frame.pack(pady=5, fill='x', padx=20)
lower_slider = ctk.CTkSlider(lower_frame, from_=0, to=255, number_of_steps=255, command=update_canny)
lower_slider.set(lower_thresh)
lower_slider.pack(side='left', fill='x', expand=True)
lower_label = ctk.CTkLabel(lower_frame, text=f"Lower: {lower_thresh}")
lower_label.pack(side='right', padx=10)

upper_frame = ctk.CTkFrame(app)
upper_frame.pack(pady=5, fill='x', padx=20)
upper_slider = ctk.CTkSlider(upper_frame, from_=0, to=255, number_of_steps=255, command=update_canny)
upper_slider.set(upper_thresh)
upper_slider.pack(side='left', fill='x', expand=True)
upper_label = ctk.CTkLabel(upper_frame, text=f"Upper: {upper_thresh}")
upper_label.pack(side='right', padx=10)

start_button = ctk.CTkButton(app, text="Iniciar", command=toggle_camera)
start_button.pack(pady=10)

app.mainloop()

# Liberar cámara al cerrar
cam.release()
cv2.destroyAllWindows()
