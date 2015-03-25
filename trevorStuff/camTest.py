#!/usr/bin/env python
import numpy as np
import os
import cv
import cv2
import csv
import time
import sys
from subprocess import call
from Tkinter import *
import Image
import ImageTk
import transformations
import threading


def getImage(cap):
    ret,frame = cap.read();
    filePATH = "tempTestFile_IDoNotNeed.jpg"
    cv2.imwrite(filePATH,frame)
    return Image.open(filePATH)

def foo_target():
    while True:
        img = ImageTk.PhotoImage(getImage(cap))
        imLabel.configure(image = img)
        imLabel.image = img
        time.sleep(.05)

gui = Tk()
camId = 0;
cap = cv2.VideoCapture(camId)
newImage = getImage(cap)
photo = ImageTk.PhotoImage(newImage)
imLabel = Label(image=photo)
imLabel.image = photo
imLabel.grid()
t = threading.Thread(target=foo_target)
t.daemon = True
t.start()

mainloop()
