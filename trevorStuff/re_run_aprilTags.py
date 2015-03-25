#!/usr/bin/env python 
import numpy as np 
import os
import cv 
import cv2
import csv
import time
import sys
from subprocess import call
import Image
import ImageTk


# Writen by Trevor Decker

def runFolder(directory,start,end,runNum):
    #path to folder of files to process 

    #TODO copy image to be a new place
    for i in xrange(start,end):
        imgFile = directory+"/"+str(i)+".jpg"
        call(["mv",directory+"/"+str(i)+".txt",directory+"/"+str(i)+"_run"+str(runNum)+".txt"])
        call(["./apriltags_demo",imgFile,'-F 768 -W 640 -H 480 -S .086'])


runFolder("log_files/1420757910.12",1,66,1)
runFolder("log_files/1421051684.58",1,6688,1)
runFolder("log_files/1421074243.57",1,17291,1)

        
