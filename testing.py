import cv2
import time
from tkinter import *
import threading
from PIL import Image, ImageTk
import numpy as np


# --------------------

def con_bri(imag, alpha, beta):
    img2 = np.zeros(imag.shape, imag.dtype)
    size = imag.shape
    if imag.ndim == 3:
        x, y, c = size

        for y in range(imag.shape[0]):
            for x in range(imag.shape[1]):
                for c in range(imag.shape[2]):
                    img2[y, x, c] = np.clip(int(alpha * imag[y, x, c] + beta), 0, 255)

    if imag.ndim == 2:
        x, y = size

        for y in range(imag.shape[0]):

            for x in range(imag.shape[1]):
                img2[y, x] = np.clip(int(alpha * imag[y, x] + beta), 0, 255)
    return (img2)


# ---------------

fenetre = Tk()

ar = cv2.imread('image.jpg', 0)
array1 = cv2.resize(ar, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_CUBIC)



aux = np.array([1., 0., 1., 0., 1.])

image = ImageTk.PhotoImage(Image.fromarray(array1), master=fenetre)


def recupere():
    print(entree1.get())
    print(entree2.get())
    aux[0] = float(entree1.get())
    aux[1] = float(entree2.get())
    aux[-1] = 1.


canvas = Canvas(fenetre, width=array1.shape[1], height=array1.shape[0])
canvas.pack()

value1 = StringVar()
value1.set(1)

value2 = StringVar()
value2.set(0)

# label


label1 = Label(fenetre, text="Contrast", bg="yellow")
label1.pack()

entree1 = Entry(fenetre, textvariable=value1, width=30)
entree1.pack()

label2 = Label(fenetre, text="Brightness", bg="yellow")
label2.pack()

entree2 = Entry(fenetre, textvariable=value2, width=30)
entree2.pack()

bouton = Button(fenetre, text="Valider", command=recupere)
bouton.pack()

image_on_canvas = canvas.create_image(20, 20, anchor=NW, image=image)

a = [1]


def main_loop():
    array3 = array1

    while True:
        machin = aux[-1]
        a[0] = a[0] + 0.01
        print(a[0])
        if machin == 1:
            array3 = con_bri(array3, aux[0] * aux[2], aux[1] + aux[3])
            aux[-1] = 0
            aux[2] = 1. / aux[0]
            aux[3] = -aux[1]
        time.sleep(1)
        image3 = ImageTk.PhotoImage(Image.fromarray(array3), master=fenetre)
        canvas.itemconfigure(image_on_canvas, image=image3)


# time.sleep(1)
# cv2.imshow('image',array1)
# cv2.waitKey(1)


thread = threading.Thread(target=main_loop)
thread.start()

fenetre.mainloop()




