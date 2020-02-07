# noam demri and fergus riche fr293 wrote this to control the Selective Plane Illumination Magnetic Manipulator
# Microscope [SPIMMM]
# description of function: this script generates a preview of the SPIMMM cameras

# libraries to import ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# os allows the code to interface with the operating system
# PySpin is the API for the cameras
# pyplot is a plotting package, used to show images
# image is an image processing package, used to manipulate images
# numpy is a numerical processing package, used to manipulate images

import os
from Tkinter import IntVar

import PySpin
import matplotlib.pyplot as plt
import cv2
import csv
import matplotlib.image as mpimg
import numpy as np
from PySpin import CValuePtr
import imageio
import Queue
import expo
import time
from matplotlib.animation import FuncAnimation
import msvcrt
import thread as th
from tkinter import *
import threading
from PIL import Image, ImageTk
import power_supply_current_controller as pscc
import power_supply_current_controller_threaded as pscct
from tkinter import filedialog
import serial
# some_file.py
import sys

import spimm_automated_experiment as sae

import spimmm_obj as so

import Trigger as tr

print('hello')
class TriggerType:
    SOFTWARE = 1
    HARDWARE = 2
CHOSEN_TRIGGER = TriggerType.HARDWARE

delay=[0.1]

# register and list cameras
# ask user for input on the camera to use
# adjust camera settings for preview mode
# create a function to acquire an image
# create window showing preview image
# update window with new preview image every 0.1s
# if process is terminated, close down camera and finish acquisition

# Function to register the different cameras
def camera_registration(cam_list, num_cameras):
    print('Number of cameras detected: %d' % num_cameras)
    print (' ')

    for i, cam in enumerate(cam_list):
        try:
            print ('Camera number: %d' % i)

            # Retrieve TL device nodemap and print device information
            nodemap = cam.GetTLDeviceNodeMap()

            try:

                node_device_information = PySpin.CCategoryPtr(nodemap.GetNode('DeviceInformation'))

                if PySpin.IsAvailable(node_device_information) and PySpin.IsReadable(node_device_information):
                    features = node_device_information.GetFeatures()
                    node_feature = PySpin.CValuePtr(features[0])
                    print ('%s: %s' % (node_feature.GetName(),
                                       node_feature.ToString() if PySpin.IsReadable(
                                           node_feature) else 'Node not readable'))

                else:
                    print ('Device control information not available.')

            except PySpin.SpinnakerException as ex:
                print ('Error: %s' % ex)
            print (' ')


        except PySpin.SpinnakerException as ex:
            print ('Error: %s' % ex)

def framerate(cam):
    cam.AcquisitionFrameRateEnable.SetValue(True)
    #print('hhh %s' % cam.AcquisitionFrameRateEnable())
    print('max frame rate= %s' % cam.AcquisitionFrameRate.GetMax())
    cam.AcquisitionFrameRate.SetValue(cam.AcquisitionFrameRate.GetMax())
    print('resulting frame rate: %s Hz' % cam.AcquisitionResultingFrameRate())
    #cam.AcquisitionFrameRate.SetValue(cam.AcquisitionFrameRate.GetMax())

# This function gets one image from the selected camera every time it is called upon
def acquire_images(cam, nodemap_tldevice, time=True):
    global image_data

    try:

        # device_serial_number = ''
        node_device_serial_number = PySpin.CStringPtr(nodemap_tldevice.GetNode('DeviceSerialNumber'))
        if PySpin.IsAvailable(node_device_serial_number) and PySpin.IsReadable(node_device_serial_number):
            # device_serial_number = node_device_serial_number.GetValue()

            try:
                # With getnewimage we can save the current image
                image_result = cam.GetNextImage()

                if image_result.IsIncomplete():
                    print ('Image incomplete with image status %d ...' % image_result.GetImageStatus())

                else:

                    # width = image_result.GetWidth()
                    # height = image_result.GetHeight()
                    # print 'Grabbed Image %d, width = %d, height = %d' % (i, width, height)

                    # Now the saved image has to be modified in order to be read as a numpy array
                    image_converted = image_result.Convert(PySpin.PixelFormat_Mono8, PySpin.HQ_LINEAR)
                    image_data = image_converted.GetNDArray()
                    image_result.Release()


            except PySpin.SpinnakerException as ex:
                print ('Error: %s' % ex)
                return False

    except PySpin.SpinnakerException as ex:
        print ('Error: %s' % ex)
        return False

    # The numpy array that contains the image is just modified in order to get a smaller image
    #image_data2 = cv2.resize(image_data, None, fx=ratio, fy=ratio, interpolation=cv2.INTER_CUBIC)
    if time==True:
        return image_data
    else:
        return image_data,image_result.GetTimeStamp()

def acquire_images2(cam_list):
    """
    This function acquires and saves 10 images from each device.

    :param cam_list: List of cameras
    :type cam_list: CameraList
    :return: True if successful, False otherwise.
    :rtype: bool
    """


    try:
        result = []

        # Retrieve, convert, and save images for each camera
        #
        # *** NOTES ***
        # In order to work with simultaneous camera streams, nested loops are
        # needed. It is important that the inner loop be the one iterating
        # through the cameras; otherwise, all images will be grabbed from a
        # single camera before grabbing any images from another.

        for i,cam in enumerate(cam_list):
            try:
                # Retrieve next received image and ensure image completion
                image_result = cam.GetNextImage()

                if image_result.IsIncomplete():
                    print'Image incomplete with image status %d ... \n' % image_result.GetImageStatus()
                else:
                    # Print image information
                    width = image_result.GetWidth()
                    height = image_result.GetHeight()


                    # Convert image to mono 8
                    image_converted = image_result.Convert(PySpin.PixelFormat_Mono8, PySpin.HQ_LINEAR)
                    image_data = image_converted.GetNDArray()
                    image_data2 = cv2.resize(image_data, None, fx=0.25, fy=0.25, interpolation=cv2.INTER_CUBIC)
                    result.append(image_data2)



                # Release image
                image_result.Release()


            except PySpin.SpinnakerException as ex:
                print'Error: %s' % ex

    except PySpin.SpinnakerException as ex:
        print'Error: %s' % ex

    return result

def camera_mode(cam,mode):
    # -------------------------
    # Buffer handling

    # Retrieve Stream Parameters device nodemap
    s_node_map = cam.GetTLStreamNodeMap()

    # Retrieve Buffer Handling Mode Information
    handling_mode = PySpin.CEnumerationPtr(s_node_map.GetNode('StreamBufferHandlingMode'))
    if not PySpin.IsAvailable(handling_mode) or not PySpin.IsWritable(handling_mode):
        print 'Unable to set Buffer Handling mode (node retrieval). Aborting...\n'
        return False

    handling_mode_entry = PySpin.CEnumEntryPtr(handling_mode.GetCurrentEntry())
    if not PySpin.IsAvailable(handling_mode_entry) or not PySpin.IsReadable(handling_mode_entry):
        print 'Unable to set Buffer Handling mode (Entry retrieval). Aborting...\n'
        return False

    # Set stream buffer Count Mode to manual
    stream_buffer_count_mode = PySpin.CEnumerationPtr(s_node_map.GetNode('StreamBufferCountMode'))
    if not PySpin.IsAvailable(stream_buffer_count_mode) or not PySpin.IsWritable(stream_buffer_count_mode):
        print 'Unable to set Buffer Count Mode (node retrieval). Aborting...\n'
        return False

    stream_buffer_count_mode_manual = PySpin.CEnumEntryPtr(stream_buffer_count_mode.GetEntryByName('Manual'))
    if not PySpin.IsAvailable(stream_buffer_count_mode_manual) or not PySpin.IsReadable(
            stream_buffer_count_mode_manual):
        print 'Unable to set Buffer Count Mode entry (Entry retrieval). Aborting...\n'
        return False

    stream_buffer_count_mode.SetIntValue(stream_buffer_count_mode_manual.GetValue())
    # print 'Stream Buffer Count Mode set to manual...'

    # Retrieve and modify Stream Buffer Count
    buffer_count = PySpin.CIntegerPtr(s_node_map.GetNode('StreamBufferCountManual'))
    if not PySpin.IsAvailable(buffer_count) or not PySpin.IsWritable(buffer_count):
        print 'Unable to set Buffer Count (Integer node retrieval). Aborting...\n'
        return False



    buffer_count.SetValue(200)

    # print 'Buffer count now set to: %d' % buffer_count.GetValue()

    handling_mode_entry = handling_mode.GetEntryByName(mode)
    handling_mode.SetIntValue(handling_mode_entry.GetValue())
    print '\nBuffer Handling Mode has been set to %s \n' % handling_mode_entry.GetDisplayName()

    # Display Buffer Info
    print '\nDefault Buffer Handling Mode: %s' % handling_mode_entry.GetDisplayName()
    print 'Default Buffer Count: %d' % buffer_count.GetValue()
    print 'Maximum Buffer Count: %d' % buffer_count.GetMax()

# This function sets up the camera so that it's ready for acquisition
def create_window(cam, nodemap):
    try:

        framerate(cam)

        node_acquisition_mode = PySpin.CEnumerationPtr(nodemap.GetNode('AcquisitionMode'))
        if not PySpin.IsAvailable(node_acquisition_mode) or not PySpin.IsWritable(node_acquisition_mode):
            print ('Unable to set acquisition mode to continuous (enum retrieval). Aborting...')
            return False

        # Retrieve entry node from enumeration node
        node_acquisition_mode_continuous = node_acquisition_mode.GetEntryByName('Continuous')

        if not PySpin.IsAvailable(node_acquisition_mode_continuous) or not PySpin.IsReadable(
                node_acquisition_mode_continuous):
            print ('Unable to set acquisition mode to continuous (entry retrieval). Aborting...')
            return False

        # Retrieve integer value from entry node
        acquisition_mode_continuous = node_acquisition_mode_continuous.GetValue()

        # Set integer value from entry node as new value of enumeration node
        node_acquisition_mode.SetIntValue(acquisition_mode_continuous)

        print('Acquisition mode set to continuous...')

        camera_mode(cam,'NewestOnly')

        # ---------------------
        #  Begin acquiring images
        #
        #  *** NOTES ***
        #  What happens when the camera begins acquiring images depends on the
        #  acquisition mode. Single frame captures only a single image, multi
        #  frame catures a set number of images, and continuous captures a
        #  continuous stream of images. Because the example calls for the
        #  retrieval of 10 images, continuous mode has been set.
        #
        #  *** LATER ***
        #  Image acquisition must be ended when no more images are needed.
        cam.BeginAcquisition()
        print ('Acquiring images...\n\n')

    except PySpin.SpinnakerException as ex:
        print ('Error: %s' % ex)
        return False

    return 1


# If you're using cv2 to display the image this function can update the frames
def update_window(cam, nodemap_tldevice):
    image = acquire_images(cam, nodemap_tldevice)
    cv2.imshow('preview', image)
    cv2.waitKey(1)


# This function resets the settings of contrast and gain to their original values, and then ends acquisition
def terminate(cam):
    # plt.close()

    expo.reset_exposure(cam)
    cam.GainAuto.SetValue(PySpin.GainAuto_Continuous)
    cam.EndAcquisition()
    # Deinitialize camera
    cam.DeInit()

    return True


def main():
    # ----------------------

    # This function will modify the settings once called upon
    global num

    def recupere():
        aux[0] = float(entree1.get())
        aux[1] = float(entree2.get())
        aux[2] = int(entree3.get())
        aux[3] = entree4.get()
        aux[4] = 1. / float(entree6.get())

        expo.configure_exposure2(cam_list[0], aux[2])
        expo.gain(cam_list[0], aux[3])

    def recupere2():
        aux2[0] = float(entree21.get())
        aux2[1] = float(entree22.get())
        aux2[2] = int(entree23.get())
        aux2[3] = entree24.get()
        aux2[4] = 1. / float(entree26.get())

        expo.configure_exposure2(cam_list[1], aux2[2])
        expo.gain(cam_list[1], aux2[3])

    # This function saves the current frame once called upon
    def save():
        #print('hi')
        #print(image7)
        image27 = np.zeros(np.shape(image7), dtype=np.int64)

        for i in range(np.shape(image7)[0]):
            for j in range(np.shape(image7)[1]):

                if image7[i, j, 1] < 0:
                    image27[i, j, 1] = image7[i, j, 1] + 255
                else:
                    image27[i, j, 1] = image7[i, j, 1]

                if image7[i, j, 0] < 0:
                    image27[i, j, 0] = image7[i, j, 0] + 255
                else:
                    image27[i, j, 0] = image7[i, j, 0]

        truc=(image27[:,:,1]+image27[:, :, 0])/2

        image37 = np.zeros((np.shape(image7)[0],np.shape(image7)[1]), dtype=np.uint8)

        for i in range(np.shape(image7)[0]):
            for j in range(np.shape(image7)[1]):

                if truc[i, j] > 127:
                    image37[i, j] = truc[i, j] - 255

                else:
                    image37[i, j] = truc[i, j]
        #print(truc)
        #new_im = Image.fromarray(truc)

        #new_im = new_im.convert('RGB')

        #print(image27)



        #print(np.min(image27))
        #print(np.max(image27))

        pic_que = Queue.Queue()
        file = filedialog.asksaveasfilename(initialdir="/", title="Select file",
                                            filetypes=(("tiff files", "*.tiff"), ("all files", "*.*")))
        if file!=None:
            if variablecoul.get()==1:

                pic_que.put(image7)
                with imageio.get_writer(file + '.tiff') as stack:
                    while not pic_que.empty():
                        stack.append_data(pic_que.get())
                #cv2.imwrite("%s.png" % file,image27)
            else:
                pic_que.put(image37)

                with imageio.get_writer(file + '.tiff') as stack:
                    while not pic_que.empty():
                        stack.append_data(pic_que.get())
   #             cv2.imwrite("%s.png" % file, truc)
            print 'Image saved as %s.tiff' % file
#
    # The following functions allow the user to save a video from the moment they press start recording to the moment
    # they press stop recording

    vid = [0]
    vidname = ['hh']

    def video():

        pic_que = Queue.Queue()
        time_que = Queue.Queue()

        while vid[0] == 0:
            pic_que.put(np.copy(image7))
            pictime = time.time()
            time_que.put(pictime)

            #print('hi')
            #print(image7)
            #print('hello')
            #print(frame_array[-1])
            #print('hi')
            plt.pause(0.1)
        time.sleep(0.5)

        while vid[0] == 1:
            pass
        file = vidname[0]



        if variablecoul.get()==1:
            #for i in range(np.shape(frame_array)[0]):
                # print(frame_array[i])
                # print('coucou')
                #ret[:, :, 0] = frame_array[i][:, :, 2]
                #ret[:, :, 1] = frame_array[i][:, :, 1]
                #ret[:, :, 2] = frame_array[i][:, :, 0]
                #video2.write(ret)
            with imageio.get_writer(file + '.tiff') as stack:
                with open(file + '_time.csv', 'ab') as f:
                    writer = csv.writer(f)
                    while not pic_que.empty():
                        auxi=pic_que.get()
                        auxit=time_que.get()
                        stack.append_data(auxi)
                        writer.writerow([auxit])
        else:
            with imageio.get_writer(file + '.tiff') as stack:
                with open(file + '_time.csv', 'ab') as f:
                    writer = csv.writer(f)
                    while not pic_que.empty():
                        interm=pic_que.get()

                        image27 = np.zeros(np.shape(image7), dtype=np.uint8)

                        for i in range(np.shape(image7)[0]):
                            for j in range(np.shape(image7)[1]):

                                if interm[i, j, 1] < 0:
                                    image27[i, j, 1] = interm[i, j, 1] + 255
                                else:
                                    image27[i, j, 1] = interm[i, j, 1]

                                if image7[i, j, 0] < 0:
                                    image27[i, j, 0] = interm[i, j, 0] + 255
                                else:
                                    image27[i, j, 0] = interm[i, j, 0]

                        truc = (image27[:, :, 1] + image27[:, :, 0]) / 2

                        image37 = np.zeros((np.shape(image7)[0], np.shape(image7)[1]), dtype=np.int8)

                        for i in range(np.shape(image7)[0]):
                            for j in range(np.shape(image7)[1]):

                                if truc[i, j] > 127:
                                    image37[i, j] = truc[i, j] - 255

                                else:
                                    image37[i, j] = truc[i, j]

                        stack.append_data(image37)
                        writer.writerow([time_que.get()])
            #for i in range(np.shape(frame_array)[0]):
                # print(frame_array[i])
                # print('coucou')
             #   ret[:, :, 0] = (frame_array[i][:, :, 2]+frame_array[i][:, :, 1]+frame_array[i][:, :, 0])/3
              #  ret[:, :, 1] = (frame_array[i][:, :, 2]+frame_array[i][:, :, 1]+frame_array[i][:, :, 0])/3
               # ret[:, :, 2] = (frame_array[i][:, :, 2]+frame_array[i][:, :, 1]+frame_array[i][:, :, 0])/3
                #video2.write(ret)

       # video2.release()
        vid[0] = 0
        print('Video saved as %s.tiff' % file)

        bouton8.configure(state=DISABLED)
        bouton7.configure(state=NORMAL)

    def start_video():
        bouton7.configure(state=DISABLED)
        bouton8.configure(state=NORMAL)
        thread2 = threading.Thread(target=video)
        thread2.start()

    def stop_vid():
        vid[0] = 1

        vidname[0] = filedialog.asksaveasfilename(initialdir="/", title="Select file",
                                            filetypes=(("tiff files", "*.tiff"), ("all files", "*.*")))
        vid[0] = 2

    # This function, once called upon, will allow the while loop in the mainloop threaded function to stop and will
    # then close the window
    def close():
        restart[0] = 1
        aux[-1] = 1
        plt.pause(0.2)
        fenetre.destroy()

    # These two function control q variable that can stop or resume the main loop
    stop = [0]

    def freeze():
        stop[0] = 1

    def resume():
        stop[0] = 0

    # The following functions are designed to thread a process that activates the magnets with the parameters chosen
    # by the user
    def magnet():

        conf = direction.get()
        amp = amplitude.get()
        dur = float(duration.get())

        connection_object = pscc.open_controller()
        time.sleep(1)
        pscc.write_values(connection_object, conf, amp)
        time.sleep(0.02)
        current_controller = threading.Thread(name='current_controller', target=pscct.trigger_currents,
                                              args=(connection_object, dur))

        current_controller.start()
        current_controller.join()
        time.sleep(0.02)
        pscc.close_controller(connection_object)

    def button_control():
        time_stop = float(duration.get()) + 1.1
        boutonM1.config(state=DISABLED)
        boutonM2.config(state=DISABLED)
        plt.pause(time_stop)
        boutonM1.config(state=NORMAL)
        boutonM2.config(state=NORMAL)

    def start_magnet():
        bout = threading.Thread(target=button_control)
        bout.start()

        mag = threading.Thread(target=magnet)
        mag.start()

    def magnet2():

        conf = direction.get()
        amp = amplitude.get()
        dur = 0.5

        connection_object = pscc.open_controller()
        time.sleep(1)
        pscc.write_values(connection_object, conf, amp)
        time.sleep(0.02)
        current_controller = threading.Thread(name='current_controller', target=pscct.trigger_currents,
                                              args=(connection_object, dur))

        current_controller.start()
        current_controller.join()
        time.sleep(0.02)
        pscc.close_controller(connection_object)

    def start_magnet2():
        bout = threading.Thread(target=button_control)
        bout.start()

        mag = threading.Thread(target=magnet2)
        mag.start()

    # The following functions allow you to run automated experiments

    def automated1():
        auto[0] = 1
        aux[-1] = 1
        time.sleep(0.2)
        fenetre.destroy()

    rows = 50
    columns = 8
    params2 = [[]]

    def automated2():

        auxi2 = columns
        auxi = 0
        while auxi2 == columns and auxi < rows:

            auxi2 = 0
            for j in range(columns):
                if Value[auxi][j].get() != '':
                    auxi2 = auxi2 + 1
            if auxi2 == columns:
                auxi = auxi + 1

        params2[0] = [[0 for j in range(columns)] for i in range(auxi)]

        for rank in range(0, auxi):
            params2[0][rank] = [Value[rank][0].get(), int(Value[rank][1].get()), int(Value[rank][2].get()),
                                float(Value[rank][3].get()), float(Value[rank][4].get()), int(Value[rank][5].get()),
                                float(Value[rank][6].get()), int(Value[rank][7].get())]

        auto[0] = 2
        aux[-1] = 1
        time.sleep(0.2)
        fenetre.destroy()

    # Temperature control

    def set_temp():

        temp[0]=temp[0]+1
        labelT.configure(text=" TEMPERATURE: %s" % temp[0])

    temp = [2]

    # Sweep volume control

    def sweep():

        print('Starting sweep')

        spim.dlo = float(valueSV2.get())
        spim.dup = float(valueSV1.get())
        spim.slp = float(valueSV3.get())
        spim.exp = VarTR1.get()/1000
        spim.frt = valueSV6.get()/1000
        spim.ste = float(valueSV7.get())
        print(spim.ste)
        spim.sendcfg()
        spim.readcfg()


        stop[0] = 1

        time.sleep(0.1)
        #spim.dlo=6
        #spim.dup=6.05
        #spim.slp=float(valueSV3.get())
        #time.sleep(1)
        #spim.sendcfg()
        #time.sleep(1)
        #spim.startvol()
        set_trigger()


        for cam in cam_list:
            camera_mode(cam,"NewestFirst")
        sw=threading.Thread(target=sweep_thread)
        #spim.readcfg()
        if valueSV5.get() == 1:
            ss = threading.Thread(target=save_sweep)
            ss.start()


        sw.start()

        #while vid[0]!=1:
        #    pass
        #stop_vid()
        #print('hi')



    def sweep_thread():

        spim.tkv()

    def save_sweep():

        #for cam in cam_list:
        #    cam.EndAcquisition()
        #    cam.Width.SetValue(1500)
        #    framerate(cam)
        #    cam.BeginAcquisition()

        labelSV2[0].place(x=250, y=250)
        pos_que= Queue.Queue()

        while True:
            respa = spim.ard.readline()
            if respa[-2:]=='o\n':

                #pic_que.put(np.copy(image7))
                pos_que.put(respa)
                print(respa)
            if respa == "VOL\n":
                break
        pic_pile = []
        time_pile1 = []
        time_pile2 = []
        print(pos_que.qsize()-1)
        image72=np.zeros((dim[0],dim[1],3), dtype=np.uint8)
        image7_aux = np.zeros((dim[0], dim[1]), dtype=np.uint8)
        for i in range(pos_que.qsize()):
            print(i)
            aux_thing1= acquire_images(cam_list[0], nodemap_tldevice[0],False)
            aux_thing2 = acquire_images(cam_list[1], nodemap_tldevice[1], False)
            print('hk')
            thing1 = np.clip(aux[0] *aux_thing1[0] ** (aux[4]) + aux[1], 0, 255)
            print('hu')
            thing2 = np.clip(aux2[0] * aux_thing2[0] ** (aux2[4]) + aux2[1], 0, 255)
            print('ha')



            if varc[0].get() == 1:
                image72[:, :, 1] = np.copy(thing1)
            else:
                image72[:, :, 1] = np.copy(image7_aux)
            print('hi')
            if varc[1].get() == 1:
                image72[:, :, 0] = np.copy(thing2)
            else:
                image72[:, :, 0] = np.copy(image7_aux)
            pic_pile.append(np.copy(image72))


            time_pile1.append(aux_thing1[1]/10.0**9)
            time_pile2.append(aux_thing2[1] / 10.0 ** 9)
        print(thing1)
        print('koooo')
        print(thing2)
        print('koooo')
        print(image72)
        print('min')
        print(np.min(thing2))
        print('max')
        print(np.max(thing2))
          #  print('ho')
        #print(respa)

        file=valueSV4.get()

        with imageio.get_writer(file + '.tiff',bigtiff=True) as stack:
            with open(file + '_position.csv', 'ab') as f:
                writer = csv.writer(f)
                j=0
                while not pos_que.empty():
                    auxip = pos_que.get()
                    auxit1 = time_pile1.pop()
                    auxit2 = time_pile2.pop()
                    print(j)
                    j=j+1
                    writer.writerow([str(auxit1) + ';' + str(auxit2) + ';' + auxip[:5] + ';' + auxip[6:-1]])
                    auxi = pic_pile.pop()
                    stack.append_data(auxi)

        print('Volume swept')
        for cam in cam_list:
            camera_mode(cam,"NewestOnly")
        set_continuous()
        labelSV2[0].place(x=-1000,y=-1000)

        stop[0] = 0

       # print('finish')

    def volumestate():
      #  print('hi')
        if valueSV5.get()==0:

            entreeSV4.config(state=DISABLED)
        if valueSV5.get()==1:
            entreeSV4.config(state=NORMAL)

    # Laser control functions

    def onlaser1():
        spim.open_ports()
        spim.lst1 = True
        spim.laser_power()

    def onlaser2():
        spim.open_ports()
        spim.lst2 = True
        spim.laser_power()

    def offlaser1():
        spim.lst1 = False
        spim.laser_power()
        spim.las1.close()

    def offlaser2():
        spim.lst2 = False
        spim.laser_power()
        spim.las2.close()

    def powlaser1():
        pow = float(valueL1.get()) / 1000.
        spim.pwr1 = pow
        spim.laser_power()

    def powlaser2():
        pow2 = float(valueL2.get()) / 1000.
        spim.pwr2 = pow2
        spim.laser_power()

    def laser():
        spim.open_ports()
        spim.pwr1 = float(valueL1.get()) / 1000.
        spim.pwr2 = float(valueL2.get()) / 1000.

        if valueL3.get()==1:

            spim.lst1=True
        else:

            spim.lst1 = False

        if valueL4.get()==1:

            spim.lst2=True
        else:

            spim.lst2 = False

        spim.laser_power()


    #Stage control

    def mirror():
        spim.posadj=float(valueS.get())
        #spim.stage(5.9)
        #spim.mirror(4095)

    def stage():
        pos = float(valueS.get())

        if valueS2.get()==0:
            spim.stage(pos)
        if valueS2.get()==1:
            spim.focus(pos)





    bb=[0]
    def engage():
        boutonS2.config(state=DISABLED)

        spim.rbt()
        plt.pause(3)

        boutonS3.config(state=NORMAL)
        spim.engage()
        boutonS1.config(state=NORMAL)
        bb[0]=1

    def disengage():
        boutonS3.config(state=DISABLED)
        boutonS2.config(state=NORMAL)
        spim.engage()
        boutonS1.config(state=DISABLED)
        bb[0]=0


    def raisestage():
        if bb[0]==1:
            spim.stage(-5)
        if bb[0]==0:
            spim.engage()
            spim.stage(-5)
            plt.pause(2)
            spim.disengage()

    def stagerr():
        auxaux=spim.err()[-2]
        labelS2.configure(text=' %s ' % auxaux)


    #Trigger mode

    def set_trigger():
        buttonTR.config(state='disabled')
        buttonTR2.config(state='normal')
        buttonTR3.config(state='normal')
        spim.exp=VarTR1.get()/1000
        spim.sendcfg()
        for cam in cam_list:

            tr.configure_trigger(cam)

    def set_continuous():
        buttonTR.config(state='normal')
        buttonTR2.config(state='disabled')
        buttonTR3.config(state='disabled')
        for i, cam in enumerate(cam_list):
            tr.reset_trigger(nodemap[i])

    def trigger():
        spim.exp = VarTR1.get()/1000
        spim.sendcfg()
        print(spim.exp)
        spim.ard.write('FRM2\r')
        #tic=time.time()
        #while True:
        #    print ('hey')
        #    respa = spim.ard.readline()
        #    print(respa)
        #    if respa == "VOL\n":
        #        break
        #toc=time.time()-tic
        #print('ext=%s' % toc)
    # --------------------
    restart = [0]
    first = 0
    auto = [0]



    while restart[0] == 0:

        spim = so.SPIMMM()

        offlaser1()
        offlaser2()
        
        restart[0] = 1
        # Retrieve singleton reference to system object
        system = PySpin.System.GetInstance()

        # Get current library version
        version = system.GetLibraryVersion()
        print ('Library version: %d.%d.%d.%d' % (version.major, version.minor, version.type, version.build))

        # Retrieve list of cameras from the system
        cam_list = system.GetCameras()
        num_cameras = cam_list.GetSize()
        #if first == 0:
        #    num = user_selection(cam_list, num_cameras)
        # Pick the camera from the list
        #cam = cam_list[num]

        # Initialize camera
        for cam in cam_list:
            cam.Init()
            #print('width')
            #print(cam.Height())
            #cam.FactoryReset()
            #time.sleep(0.5)
            #cam.Width.SetValue(1632)
            #cam.OffsetX.SetValue(408)


        nodemap_tldevice = [cam.GetTLDeviceNodeMap() for cam in cam_list]

        # Retrieve GenICam nodemap
        nodemap = [cam.GetNodeMap() for cam in cam_list]

        # Setting initial values for exposure time and gain
        for i,cam in enumerate(cam_list):
            expo.configure_exposure2(cam, 10000)
            expo.gain(cam, 4)
            tr.reset_trigger(nodemap[i])

        # Setting up the camera for acquisition
        for i in range(num_cameras):
            create_window(cam_list[i], nodemap[i])


        # Array to store the values for contrast, brightness and time exposure in us
        aux = np.array([1., 0., 10000, 4., 1., 0])
        aux2 = np.array([1., 0., 10000, 4., 1., 0])

        # Time between each frame in s

        # Initiating the window for the preview's GUI
        fenetre = Tk()
        fenetre.geometry("1300x700")

        #Defining functions to validate entries----
        def correct(inp):

            if inp.isdigit() or inp == '':
                return True
            else:
                return False

        reg = fenetre.register(correct)

        def correctf(inp):

            if inp.replace(".", "", 1).isdigit() or inp == '':
                return True
            else:
                return False

        regf = fenetre.register(correctf)

        def correctfn(inp):

            if inp == '' or inp == '-':
                return True

            elif inp[0] == '-':
                return inp.replace(".", "", 1).replace("-", "", 1).isdigit()
            else:
                return inp.replace(".", "", 1).isdigit()

        regfn = fenetre.register(correctfn)

        #
        varc=[IntVar() for cam in cam_list]


        for i, cam in enumerate(cam_list):

            nodemap2 = cam.GetTLDeviceNodeMap()

            node_device_information = PySpin.CCategoryPtr(nodemap2.GetNode('DeviceInformation'))

            if PySpin.IsAvailable(node_device_information) and PySpin.IsReadable(node_device_information):
                features = node_device_information.GetFeatures()
                node_feature = PySpin.CValuePtr(features[0])
                Checkbutton(fenetre, text="cam: %s" % node_feature.ToString(), variable=varc[i]).place(x=20+i*100,y=520)


        dim=np.shape(acquire_images(cam_list[0],nodemap_tldevice[0]))





        image5 = [np.clip(aux[0] * cv2.resize(acquire_images(cam_list[i],nodemap_tldevice[i]), None, fx=0.25, fy=0.25, interpolation=cv2.INTER_CUBIC) ** (aux[4])+ aux[1], 0, 255) for i in range(num_cameras)]

        image7=np.zeros((np.shape(image5[0])[0],np.shape(image5[0])[1],3),dtype=np.int8)
        for i in range(num_cameras):
            if varc[i].get()==1:
                image7[:,:,i]=image5[i]




        image6 = [ImageTk.PhotoImage(Image.fromarray(image7,mode="RGB"), master=fenetre)]



        # Creating the canvas for the image
        canvas = Canvas(fenetre, width=image5[0].shape[1]+25, height=image5[0].shape[0]+25)
        canvas.place(x=-18, y=-18)

        # Creating the values stored in the different entries
        value1 = StringVar()
        value1.set(1)

        value2 = StringVar()
        value2.set(0)

        value3 = StringVar()
        value3.set(10000)

        value4 = StringVar()
        value4.set(4)

        value6 = StringVar()
        value6.set(1)

        value21 = StringVar()
        value21.set(1)

        value22 = StringVar()
        value22.set(0)

        value23 = StringVar()
        value23.set(10000)

        value24 = StringVar()
        value24.set(4)

        value26 = StringVar()
        value26.set(1)

        # labels, entries and button for the different parameters of the first camera

        for i, cam in enumerate(cam_list):

            nodemap2 = cam.GetTLDeviceNodeMap()

            node_device_information = PySpin.CCategoryPtr(nodemap2.GetNode('DeviceInformation'))

            if PySpin.IsAvailable(node_device_information) and PySpin.IsReadable(node_device_information):
                features = node_device_information.GetFeatures()
                node_feature = PySpin.CValuePtr(features[0])
                label5 = Label(fenetre, text="Parameters for camera %s" % node_feature.ToString(), bg = "cyan")
                label5.place(x=800, y=i*100)




        label6 = Label(fenetre, text="Gamma", bg="yellow")
        label6.place(x=700, y=30)

        entree6 = Entry(fenetre, textvariable=value6, width=10, validate="key", validatecommand=(regf, '%P'))
        entree6.place(x=630, y=30)

        label1 = Label(fenetre, text="Contrast", bg="yellow")
        label1.place(x=700, y=60)

        entree1 = Entry(fenetre, textvariable=value1, width=10, validate="key", validatecommand=(regf, '%P'))
        entree1.place(x=630, y=60)

        label2 = Label(fenetre, text="Brightness", bg="yellow")
        label2.place(x=835, y=30)

        entree2 = Entry(fenetre, textvariable=value2, width=10, validate="key", validatecommand=(regfn, '%P'))
        entree2.place(x=765, y=30)

        label3 = Label(fenetre, text="Exposure time in us (between 7 and 29999998)", bg="yellow")
        label3.place(x=835, y=60)

        entree3 = Entry(fenetre, textvariable=value3, width=10, validate="key", validatecommand=(reg, '%P'))
        entree3.place(x=765, y=60)

        label4 = Label(fenetre, text="Gain in Db (below 47.994294)", bg="yellow")
        label4.place(x=985, y=30)

        entree4 = Entry(fenetre, textvariable=value4, width=10, validate="key", validatecommand=(regf, '%P'))
        entree4.place(x=915, y=30)

        bouton1 = Button(fenetre, text="Change settings", command=recupere)
        bouton1.place(x=1100, y=60)

        # labels, entries and button for the different parameters of the second camera

        #label25 = Label(fenetre, text="Parameters", bg="cyan")
        #label25.place(x=855, y=110)

        label26 = Label(fenetre, text="Gamma", bg="yellow")
        label26.place(x=700, y=130)

        entree26 = Entry(fenetre, textvariable=value26, width=10, validate="key", validatecommand=(regf, '%P'))
        entree26.place(x=630, y=130)

        label21 = Label(fenetre, text="Contrast", bg="yellow")
        label21.place(x=700, y=160)

        entree21 = Entry(fenetre, textvariable=value21, width=10, validate="key", validatecommand=(regf, '%P'))
        entree21.place(x=630, y=160)

        label22 = Label(fenetre, text="Brightness", bg="yellow")
        label22.place(x=835, y=130)

        entree22 = Entry(fenetre, textvariable=value22, width=10, validate="key", validatecommand=(regfn, '%P'))
        entree22.place(x=765, y=130)

        label23 = Label(fenetre, text="Exposure time in us (between 7 and 29999998)", bg="yellow")
        label23.place(x=835, y=160)

        entree23 = Entry(fenetre, textvariable=value23, width=10, validate="key", validatecommand=(reg, '%P'))
        entree23.place(x=765, y=160)

        label24 = Label(fenetre, text="Gain in Db (below 47.994294)", bg="yellow")
        label24.place(x=985, y=130)

        entree24 = Entry(fenetre, textvariable=value24, width=10, validate="key", validatecommand=(regf, '%P'))
        entree24.place(x=915, y=130)

        bouton21 = Button(fenetre, text="Change settings", command=recupere2)
        bouton21.place(x=1100, y=160)

        # Label, entry and button to save the image
        label5 = Label(fenetre, text="Save a frame or a video", bg="cyan")
        label5.place(x=660, y=210)

        variablecoul=IntVar()
        variablecoul.set(1)

        boutongray=Radiobutton(fenetre,text='Grayscale',variable=variablecoul,value=0)
        boutongray.place(x=655, y=235)

        boutoncoul = Radiobutton(fenetre, text='Color', variable=variablecoul, value=1)
        boutoncoul.place(x=735, y=235)

        bouton2 = Button(fenetre, text="Save the current image", command=save)
        bouton2.place(x=660, y=260)

        # Label, entry and button to record a video

        bouton7 = Button(fenetre, text="Start recording", command=start_video)
        bouton7.place(x=632, y=290)
        bouton8 = Button(fenetre, text="Stop recording", command=stop_vid, state=DISABLED)
        bouton8.place(x=722, y=290)

        # Freeze and Resume button

        bouton3 = Button(fenetre, text="Freeze", command=freeze)
        bouton3.place(x=250, y=520)

        bouton4 = Button(fenetre, text="Resume", command=resume)
        bouton4.place(x=300, y=520)

        # Magnet control

        label7 = Label(fenetre, text="Magnets control", bg="Cyan")
        label7.place(x=950, y=210)

        # Select a direction for pulling with the magnet
        direction = IntVar()
        direction.set(1)

        label7 = Label(fenetre, text="Pick a direction:", bg="yellow")
        label7.place(x=870, y=240)
        boutonN = Radiobutton(fenetre, text="North (3)", variable=direction, value=3)
        boutonN.place(x=845, y=270)
        boutonS = Radiobutton(fenetre, text="South (2)", variable=direction, value=2)
        boutonS.place(x=920, y=270)
        boutonE = Radiobutton(fenetre, text="East (4)", variable=direction, value=4)
        boutonE.place(x=845, y=300)
        boutonW = Radiobutton(fenetre, text="West (1)", variable=direction, value=1)
        boutonW.place(x=920, y=300)

        # Select an amplitude

        amplitude = IntVar()
        amplitude.set(1)
        label8 = Label(fenetre, text="Pick an amplitude:", bg="yellow")
        label8.place(x=1065, y=240)
        boutonA1 = Radiobutton(fenetre, text="0.5 A (1)", variable=amplitude, value=1)
        boutonA1.place(x=1020, y=270)
        boutonA2 = Radiobutton(fenetre, text="1 A (2)", variable=amplitude, value=2)
        boutonA2.place(x=1090, y=270)
        boutonA3 = Radiobutton(fenetre, text="1.5 A (3)", variable=amplitude, value=3)
        boutonA3.place(x=1160, y=270)
        boutonA4 = Radiobutton(fenetre, text="2 A (4)", variable=amplitude, value=4)
        boutonA4.place(x=1020, y=300)
        boutonA5 = Radiobutton(fenetre, text="2.5 A (5)", variable=amplitude, value=5)
        boutonA5.place(x=1090, y=300)

        valued = StringVar()
        valued.set(1)

        duration = Entry(fenetre, textvariable=valued, width=10, validate="key", validatecommand=(regf, '%P'))
        duration.place(x=1075, y=330)
        boutonM1 = Button(fenetre, text="Run magnet for following duration (in s):", command=start_magnet)
        boutonM1.place(x=845, y=327)
        boutonM2 = Button(fenetre, text="Pulse", command=start_magnet2)
        boutonM2.place(x=1190, y=327)

        # Temperature
        label5 = Label(fenetre, text="Temperature control", bg="cyan")
        label5.place(x=670, y=335)

        valueT = StringVar()
        valueT.set(2)

        EntreeT = Entry(fenetre, textvariable=valueT, width=10)
        EntreeT.place(x=770, y=360)

        EntreeT.config(validate="key", validatecommand=(reg, '%P'))
        boutonT1 = Button(fenetre, text="Set the temperature to", command=set_temp)
        boutonT1.place(x=630, y=357)

        labelT = Label(fenetre, text=" TEMPERATURE: %s" % temp[0], bg='white')
        labelT.place(x=680, y=390)

        # Volume sweep section

        labelSV = Label(fenetre, text="Take a volume", bg="cyan")
        labelSV.place(x=670, y=430)

        valueSV1 = StringVar()
        valueSV1.set(6.3)

        valueSV2 = StringVar()
        valueSV2.set(6.11)

        valueSV3 = StringVar()
        valueSV3.set(-4486.982)



        valueSV4 = StringVar()
        valueSV4.set('Volume')



        valueSV5 = IntVar()
        valueSV5.set(1)

        valueSV6 = IntVar()
        valueSV6.set(25000)

        valueSV7 = StringVar()
        valueSV7.set(0.02)

        #This value is shared with another section
        VarTR1 = IntVar()
        VarTR1.set(10000)

        label1 = Label(fenetre, text="Upper limit", bg="yellow")
        label1.place(x=730, y=455)

        entreeSV1 = Entry(fenetre, textvariable=valueSV1, width=10, validate="key", validatecommand=(regfn, '%P'))
        entreeSV1.place(x=660, y=455)

        label1 = Label(fenetre, text="Lower limit", bg="yellow")
        label1.place(x=730, y=485)

        entreeSV2 = Entry(fenetre, textvariable=valueSV2, width=10, validate="key", validatecommand=(regfn, '%P'))
        entreeSV2.place(x=660, y=485)

        label1 = Label(fenetre, text="Slope", bg="yellow")
        label1.place(x=730, y=515)

        entreeSV3 = Entry(fenetre, textvariable=valueSV3, width=10, validate="key", validatecommand=(regfn, '%P'))
        entreeSV3.place(x=660, y=515)

        label1 = Label(fenetre, text="Trigger exposure time (us)", bg="yellow")
        label1.place(x=730, y=545)

        entreeSV4 = Entry(fenetre, textvariable=VarTR1, width=10, validate="key", validatecommand=(regfn, '%P'))
        entreeSV4.place(x=660, y=545)

        label1 = Label(fenetre, text="Time btw frames (us)", bg="yellow")
        label1.place(x=730, y=575)

        entreeSV5 = Entry(fenetre, textvariable=valueSV6, width=10, validate="key", validatecommand=(regfn, '%P'))
        entreeSV5.place(x=660, y=575)

        label1 = Label(fenetre, text="Distance btw frames (mm)", bg="yellow")
        label1.place(x=730, y=605)

        entreeSV6 = Entry(fenetre, textvariable=valueSV7, width=10, validate="key", validatecommand=(regfn, '%P'))
        entreeSV6.place(x=660, y=605)

        buttonSV2=Checkbutton(fenetre, text="Save volume as", variable=valueSV5,command=volumestate)
        buttonSV2.place(x=650,y=635)

        entreeSV4 = Entry(fenetre, textvariable=valueSV4, width=10, validate="key", validatecommand=(regfn, '%P'),state=NORMAL)
        entreeSV4.place(x=760, y=635)

        boutonSV1 = Button(fenetre, text="Sweep volume", command=sweep)
        boutonSV1.place(x=660, y=660)


        labelSV2 = [Label(fenetre,text='SWEEPING VOLUME',fg='red')]



        # Lasers control

        valueL1 = StringVar()
        valueL1.set(20)

        valueL2 = StringVar()
        valueL2.set(20)

        valueL3 = IntVar()
        valueL3.set(0)

        valueL4 = IntVar()
        valueL4.set(0)

        labelL1 = Label(fenetre, text="Lasers control", bg="Cyan")
        labelL1.place(x=960, y=380)

        labelL2 = Label(fenetre, text="488 Laser:", bg="yellow")
        labelL2.place(x=830, y=415)

        boutonL1 = Radiobutton(fenetre, text="ON", variable=valueL3, value=1)
        boutonL1.place(x=895, y=412)

        boutonL2 = Radiobutton(fenetre, text="OFF", variable=valueL3, value=0)
        boutonL2.place(x=940, y=412)

        labelL3 = Label(fenetre, text="Power (in mW):")
        labelL3.place(x=1000, y=412)

        entreeL1 = Entry(fenetre, textvariable=valueL1, width=10, validate="key", validatecommand=(reg, '%P'))
        entreeL1.place(x=1090, y=415)

        boutonL1= Button(fenetre, text="Update", command=laser)
        boutonL1.place(x=1170, y=428)

        labelL3 = Label(fenetre, text="561 Laser:", bg="yellow")
        labelL3.place(x=830, y=450)

        boutonL4 = Radiobutton(fenetre, text="ON", variable=valueL4, value=1)
        boutonL4.place(x=895, y=447)

        boutonL5 = Radiobutton(fenetre, text="OFF", variable=valueL4, value=0)
        boutonL5.place(x=940, y=447)

        boutonL6 = Label(fenetre, text="Power (in mW):")
        boutonL6.place(x=1000, y=447)

        entreeL2 = Entry(fenetre, textvariable=valueL2, width=10, validate="key", validatecommand=(reg, '%P'))
        entreeL2.place(x=1090, y=450)





        # Stage

        labelS1=Label(fenetre, text='Stage control',bg='Cyan')
        labelS1.place(x=960,y=490)

        valueS = StringVar()
        valueS.set(6.11)

        valueS2 = IntVar()
        valueS2.set(0)

        boutonS1 = Button(fenetre, text="Set stage to:", command=stage, state=DISABLED )
        boutonS1.place(x=890, y=520)

        entreeS1 = Entry(fenetre, textvariable=valueS, width=10, validate="key", validatecommand=(regfn, '%P'))
        entreeS1.place(x=970, y=523)

        boutonS2 = Button(fenetre, text='Engage', command=engage)
        boutonS2.place(x=890, y=550)

        boutonS3 = Button(fenetre, text='Disengage', command=disengage,state=DISABLED)
        boutonS3.place(x=940, y=550)

        truceee = Button(fenetre, text='Raise', command=raisestage)
        truceee.place(x=1020, y=550)

        boutonS4 = Button(fenetre, text='Error:', command=stagerr)
        boutonS4.place(x=1090, y=550)

        labelS2 = Label(fenetre, text='      ',bg='White')
        labelS2.place(x=1133, y=553)

        buttonS5= Checkbutton(fenetre,text='Mirror adjusted',variable=valueS2,command=mirror)
        buttonS5.place(x=1050,y=520)

        # Trigger
        labelTR1=Label(fenetre, text='Trigger control',bg='cyan').place(x=960,y=600)
        buttonTR = Button(fenetre, text="Triggering mode", command=set_trigger)
        buttonTR.place(x=930, y=630)
        buttonTR2 = Button(fenetre, text="Trigger", command=trigger,state='disabled')
        buttonTR2.place(x=880, y=630)
        buttonTR3 = Button(fenetre, text="Continuous mode", command=set_continuous,state='disabled')
        buttonTR3.place(x=1030, y=630)
        EntryTR1=Entry(fenetre,textvariable=VarTR1,width=10)
        EntryTR1.place(x=940,y=660)
        label1 = Label(fenetre, text="Trigger exposure time (us)", bg="yellow")
        label1.place(x=1010, y=660)

        # Automated experiment part

        boutonA1 = Button(fenetre, text="Run automated experiment from file", command=automated1)
        boutonA1.place(x=40, y=590)

        boutonA2 = Button(fenetre, text="Run automated experiment with parameters below", command=automated2)
        boutonA2.place(x=300, y=590)

        labelA2 = Label(fenetre, text="Automated Experiment",bg="Cyan")
        labelA2.place(x=20, y=560)

        # Labels and buttons for the automated experiment
        labelA3 = Label(fenetre, text="Name", bg="yellow")
        labelA3.place(x=20, y=620)

        labelA4 = Label(fenetre, text="Amplitude", bg="yellow")
        labelA4.place(x=83, y=620)

        labelA4 = Label(fenetre, text="Configuration", bg="yellow")
        labelA4.place(x=150, y=620)

        labelA4 = Label(fenetre, text="Force on", bg="yellow")
        labelA4.place(x=240, y=620)

        labelA5 = Label(fenetre, text="Force duration", bg="yellow")
        labelA5.place(x=300, y=620)

        labelA6 = Label(fenetre, text="Num frames", bg="yellow")
        labelA6.place(x=383, y=620)

        labelA7 = Label(fenetre, text="Frame period", bg="yellow")
        labelA7.place(x=457, y=620)

        labelA8 = Label(fenetre, text="Temperature", bg="yellow")
        labelA8.place(x=533, y=620)



        # ----------------------

        frame_main = Frame(fenetre, bg=""
                                       "gray")
        frame_main.grid(sticky='news')
        frame_main.place(x=0, y=640)

        # Create a frame for the canvas with non-zero row&column weights
        frame_canvas = Frame(frame_main)
        frame_canvas.grid(row=2, column=0, pady=(5, 0), sticky='nw')
        frame_canvas.grid_rowconfigure(0, weight=1)
        frame_canvas.grid_columnconfigure(0, weight=1)
        # Set grid_propagate to False to allow 5-by-5 buttons resizing later
        frame_canvas.grid_propagate(False)

        # Add a canvas in that frame
        canvas2 = Canvas(frame_canvas, bg="yellow")
        canvas2.grid(row=0, column=0, sticky="news")

        # Link a scrollbar to the canvas
        vsb = Scrollbar(frame_canvas, orient="vertical", command=canvas2.yview)
        vsb.grid(row=0, column=1, sticky='ns')
        canvas2.configure(yscrollcommand=vsb.set)

        # Create a frame to contain the buttons
        frame_buttons = Frame(canvas2, bg="blue")
        canvas2.create_window((0, 0), window=frame_buttons, anchor='nw')

        # Add 9-by-5 buttons to the frame

        Entree = [[Entry() for j in range(columns)] for i in range(rows)]
        Value = [[StringVar() for j in range(columns)] for i in range(rows)]

        for i in range(0, rows):
            for j in range(0, columns):
                if first == 0:
                    Value[i][j].set('')
                if i == 0:
                    Entree[i][j] = Entry(frame_buttons, textvariable=Value[i][j], width=12)
                else:
                    Entree[i][j] = Entry(frame_buttons, textvariable=Value[i][j], width=12, validate="key",
                                         validatecommand=(regf, '%P'))
                Entree[i][j].grid(row=i, column=j, sticky='news')

        # Update buttons frames idle tasks to let tkinter calculate buttons sizes
        frame_buttons.update_idletasks()

        # Resize the canvas frame to show exactly 5-by-5 buttons and the scrollbar
        first5columns_width = sum([Entree[0][j].winfo_width() for j in range(0, 8)])
        first5rows_height = sum([Entree[i][0].winfo_height() for i in range(0, 4)])
        frame_canvas.config(width=first5columns_width + vsb.winfo_width(),
                            height=first5rows_height)

        # Set the canvas scrolling region
        canvas2.config(scrollregion=canvas2.bbox("all"))

        # ----------------------

        # Placing the first image in the canvas
        image_on_canvas = canvas.create_image(20, 20, anchor=NW, image=image6[0])

        # a = [1]

        image7_aux = np.zeros((np.shape(image5[0])[0], np.shape(image5[0])[1]), dtype=np.int8)
        def main_loop():

            while aux[-1] != 1:

                while stop[0] == 1:
                    pass

                # Acquiring the new image, changing contrast and brightness, and changing its format so that it's
                # compatible with tkinter
                #time.sleep(3)
                aux_thing1=acquire_images(cam_list[0],nodemap_tldevice[0])
                aux_thing2=acquire_images(cam_list[1], nodemap_tldevice[1])
                thing1=np.clip(aux[0] * cv2.resize(aux_thing1, None, fx=0.25, fy=0.25, interpolation=cv2.INTER_CUBIC) ** (aux[4])+ aux[1], 0, 255)
                thing2 = np.clip(aux2[0] * cv2.resize(aux_thing2, None, fx=0.25, fy=0.25, interpolation=cv2.INTER_CUBIC) ** (aux2[4]) + aux2[1], 0, 255)

                #image8[0] = np.clip(aux[0] * acquire_images(cam_list[1], nodemap_tldevice[1]) ** (aux[4])+ aux[1], 0, 255)

                #print('thing1 min' , np.min(thing1))
                #print(type(np.min(thing1)))
                #print('thing1 max' , np.max(thing1))
                #print('thing2 min' , np.min(thing2))
                #print(type(np.min(thing2)))
                #print('thing2 max' , np.max(thing2))


                if varc[0].get() == 1:
                    image7[:, :, 1] = np.copy(thing1)
                else:
                    image7[:, :, 1] = np.copy(image7_aux)

                if varc[1].get() == 1:
                    image7[:, :, 0] = np.copy(thing2)
                else:
                    image7[:, :, 0] = np.copy(image7_aux)


                #image7[:, :, 0] = thing1
                #image7[:, :, 1] = thing2
                #print('hey')
                #print(image7)
                #print('hey')
                plt.pause(delay[0])


                #print('image7 min' , np.min(image7))
                #print('image7 max' , np.max(image7))

                #image72 = cv2.resize(image_data, None, fx=0.25, fy=0.25, interpolation=cv2.INTER_CUBIC)

                image6 = [ImageTk.PhotoImage(Image.fromarray(image7
                                                             ,"RGB"), master=fenetre)]
                # Changing the image on the canvas

                canvas.itemconfigure(image_on_canvas, image=image6[0])



        #function to close the window and leave the devices in the right state
        def doSomething():

            stop[0]=0
            aux[-1]=1
            set_continuous()
            plt.pause(0.2)
            fenetre.destroy()
        fenetre.protocol('WM_DELETE_WINDOW', doSomething)

                # canvas.itemconfigure(image_on_canvas, image=image6[0])

        #T1 = threading.Thread(target=thread1)
        #T2 = threading.Thread(target=thread2)
        #T1.start()
        #T2.start()



        thread = threading.Thread(target=main_loop)
        thread.start()


        fenetre.mainloop()
        aux[-1] = 1
        plt.pause(1)
        thread.join()


        # Terminating the camera


        for i,cam in enumerate(cam_list):

            terminate(cam)
            cam = None

        # Clear camera list before releasing system
        cam_list.Clear()

        # Release system instance
        system.ReleaseInstance()

        del system

        offlaser1()
        offlaser2()
        raisestage()
        spim.disengage()
        spim.close_ports()

        if auto[0] == 1:
            sae.function()
            restart[0]=0
        if auto[0] == 2:
            sae.function(2, params2[0])
            restart[0]=0
        auto[0] = 0
        first = 1






    print('blackfly_preview successfully exited')

    return True


if __name__ == '__main__':
    main()
