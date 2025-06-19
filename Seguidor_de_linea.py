import cv2
import sys
import threading
import numpy as np
import tkinter as tk
from threading import Lock
import time

sys.path.append('../utils')

from PIL import Image, ImageTk
from pyESP32_BT import BluetoothSerialESP32

# =============================================
# Configuración global y variables compartidas
# =============================================
min_area = 500
url = "http://10.10.120.48:4747/video"  # o "tcp://..." para mejor estabilidad

# Variables de estado y sincronización
camera_lock = Lock()
camera_active = True
gui_active = True

# =============================================
# Configuración inicial de la cámara
# =============================================
cap = cv2.VideoCapture()
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)      # Buffer mínimo
cap.set(cv2.CAP_PROP_FPS, 15)            # Ajustar según cámara
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)   # Reducción de resolución
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# =============================================
# Funciones de procesamiento de imagen
# =========================================9999999999999999999999====
def object_detection(raw_image):
    kernel = np.ones((10, 10), dtype=np.uint8)
    is_object = False
    cx, cy = 0, 0

    try:
        gray = cv2.cvtColor(raw_image, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, threshold_val.get(), 255, cv2.THRESH_BINARY)
        opening = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
        
        contours, _ = cv2.findContours(opening, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for cnt in contours:
            moments = cv2.moments(cnt)
            if moments['m00'] > min_area:
                cx = int(moments['m10']/moments['m00'])
                cy = int(moments['m01']/moments['m00'])
                is_object = True
                break
    except Exception as e:
        print(f"Error en detección: {e}")
    
    return is_object, binary, cx, cy

# =============================================
# Funciones de manejo de video y GUI
# =============================================
def stream_video():
    global cap, camera_active
    reconnect_attempts = 0
    max_reconnect = 5
    
    while camera_active:
        with camera_lock:
            if not cap.isOpened():
                print("Reconectando a la cámara...")
                if reconnect_attempts < max_reconnect:
                    cap.open(url)
                    reconnect_attempts += 1
                    time.sleep(2)
                else:
                    print("Error: Máximos reintentos alcanzados")
                    break
            
            if cap.isOpened():
                ret, frame = cap.read()
                if not ret:
                    print("Error de lectura de frame")
                    cap.release()
                    continue
                
                try:
                    is_obj, binary, cx, cy = object_detection(frame)
                    
                    # Dibujar marcadores
                    cv2.circle(frame, (cx, cy), 10, (0, 0, 255), -1)
                    cv2.circle(frame, (cx_d, cy_d), 10, (0, 255, 0), -1)
                    
                    # Cálculo de control
                    if is_obj:
                        h_x = frame.shape[1]/2 - cx
                        hx_e = hx_d - h_x
                        K = 0.0035
                        u_ref = 0.2
                        w_ref = -K * hx_e
                    else:
                        u_ref = w_ref = 0
                    
                    # Envío de comandos
                    if btn_var.get() == 'Running':
                        bt_serial.write_serial_data(f"{u_ref},{w_ref}")
                    
                    # Actualización GUI
                    img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    img = Image.fromarray(img)
                    img_tk = ImageTk.PhotoImage(image=img)
                    
                    img_bin = Image.fromarray(binary)
                    img_tk_bin = ImageTk.PhotoImage(image=img_bin)
                    
                    root.after(0, update_gui, img_tk, img_tk_bin)
                
                except Exception as e:
                    print(f"Error en procesamiento: {e}")
        
        time.sleep(0.01)  # Aliviar carga CPU

def update_gui(img_tk, img_tk_bin):
    if gui_active:
        try:
            label_left.img_tk = img_tk
            label_left.configure(image=img_tk)
            
            label_right.img_tk = img_tk_bin
            label_right.configure(image=img_tk_bin)
        except tk.TclError:
            print("GUI cerrada, deteniendo actualizaciones")

# =============================================
# Funciones de control y configuración
# =============================================
def onClosing():
    global camera_active, gui_active
    
    camera_active = False
    gui_active = False
    
    with camera_lock:
        if cap.isOpened():
            cap.release()
    
    bt_serial.write_serial_data("0,0")
    bt_serial.stop()
    root.quit()
    root.destroy()
    print("Sistema apagado correctamente")

def get_threshold_val(val):
    threshold_val.set(slider.get())

def toggle():
    current_state = btn_var.get()
    button.config(text="Running" if current_state == "Run" else "Run")

# =============================================
# Configuración inicial y main loop
# =============================================
if __name__ == '__main__':
    # Inicialización cámara
    if not cap.open(url):
        sys.exit("Error: No se pudo conectar a la cámara")
    
    # Configuración posición deseada
    ret, frame = cap.read()
    if not ret:
        sys.exit("Error: No se pudo obtener frame inicial")
    
    cx_d = int(frame.shape[1]/2)
    cy_d = int(frame.shape[0]/2)
    hx_d = 0

    # Configuración Bluetooth
    bt_serial = BluetoothSerialESP32(port='/dev/rfcomm0', n_vars=2)
    bt_serial.start()

    # Configuración GUI
    root = tk.Tk()
    root.protocol("WM_DELETE_WINDOW", onClosing)
    root.title('Sistema de Visión Artificial')

    # Widgets de control
    threshold_val = tk.IntVar(value=127)
    btn_var = tk.StringVar(value='Run')
    
    # Layout GUI
    button = tk.Checkbutton(root, textvariable=btn_var, width=12,
                          variable=btn_var, onvalue='Running', offvalue='Run',
                          indicator=False, command=toggle)
    button.grid(row=1, column=1, padx=20, pady=10)

    slider = tk.Scale(root, label='Umbral', from_=0, to=255,
                    orient=tk.HORIZONTAL, variable=threshold_val,
                    command=get_threshold_val, length=400)
    slider.grid(row=1, column=0, padx=20, pady=10)

    frame_left = tk.Frame(root, width=640, height=480)
    frame_left.grid(row=0, column=0, padx=10, pady=10)
    
    frame_right = tk.Frame(root, width=640, height=480)
    frame_right.grid(row=0, column=1, padx=10, pady=10)

    label_left = tk.Label(frame_left)
    label_left.pack(fill=tk.BOTH, expand=True)
    
    label_right = tk.Label(frame_right)
    label_right.pack(fill=tk.BOTH, expand=True)

    # Hilo de video
    video_thread = threading.Thread(target=stream_video, daemon=True)
    video_thread.start()

    # Main loop
    root.mainloop()
