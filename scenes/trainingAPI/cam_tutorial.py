

from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import numpy as np
from PIL import Image
import cv2
import time

def salva_immagine_telecamera():
    
    print('Inizio del programma')

    # Crea un'istanza del client API
    client = RemoteAPIClient()

    # Richiedi l'oggetto sim per interagire con CoppeliaSim
    sim = client.require('sim')
    simVision = client.require('simVision')

    # Carica la tua scena personalizzata
    sim.loadScene('/Users/saramorandi/Desktop/telecamera_3d_new.ttt')

    # Ottieni l'handle del sensore di visione (sostituisci 'Vision_sensor' con il nome effettivo del tuo sensore)
    visionSensorHandle = sim.getObject('./sensor')

    time.sleep(1)

    # Ottieni i dati dell'immagine dal sensore di vision -> FIRST TEST
    image, resolution = sim.getVisionSensorImg(visionSensorHandle, 0 , 0.0, [0,0], [128, 128])
    print(image)
    print(resolution)
    image_integers = [b for b in image]
    image_array = np.array(image_integers, dtype=np.uint8).reshape((resolution[1], resolution[0], 3))
    img = Image.fromarray(image_array, 'RGB')

    # # Ottieni i dati dell'immagine dal sensore di vision -> SECOND TEST
    # img, [resX, resY] = sim.getVisionSensorImg(visionSensorHandle)
    # img = np.frombuffer(img, dtype=np.uint8).reshape(resY, resX, 3)
    # img = cv2.flip(cv2.cvtColor(img, cv2.COLOR_BGR2RGB), 0)
    # cv2.imwrite('/Users/saramorandi/Desktop/smart robotics/img_3d.png', img)

        # Salva l'immagine su disco
    img.save('immagine_telecamera.png')
    print("Immagine salvata come 'immagine_telecamera.png'")

if _name_ == '_main_':
    salva_immagine_telecamera()