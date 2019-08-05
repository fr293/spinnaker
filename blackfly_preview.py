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
import PySpin
import matplotlib.pyplot as plt
import cv2
import matplotlib.image as mpimg
import numpy as np
import expo
import time
from matplotlib.animation import FuncAnimation
import msvcrt
import thread as th
from tkinter import *
import threading
from PIL import Image, ImageTk
import power_supply_current_controller_copy as pscc
from tkinter import filedialog
import serial
# some_file.py
import sys
# insert at 1, 0 is the script path (or '' in REPL)
sys.path.insert(0, 'D:\Spimm')

import spimm_automated_experiment as sae

# insert at 1, 0 is the script path (or '' in REPL)
sys.path.insert(1, 'C:\Users\User\Dropbox (Cambridge University)\Cambs\PhD\SPIMMM\microscope_control')

import spimmm_obj as so

# register and list cameras
# ask user for input on the camera to use
# adjust camera settings for preview mode
# create a function to acquire an image
# create window showing preview image
# update window with new preview image every 0.1s
# if process is terminated, close down camera and finish acquisition

#Function to register the different cameras
def camera_registration(cam_list, num_cameras):
    print('Number of cameras detected: %d' % num_cameras)
    print (' ')

    for i, cam in enumerate(cam_list):
        try:
            print ('Camera number: %d' % i)
            result = True

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
            result = False

#Function that allows you to pick which camera you want to use via a graphic interface
def user_selection(cam_list, num_cameras):

    #Initiating a tkinter window
    root=Tk()

    #We will save in res the number of the camera that's been selected
    res=[3]

    #This function will save the number of the camera that has been selected and then destroy the window in order to get to the preview
    def nextstep():
        res[0]=v0.get()
        root.destroy()


    labelu1 = Label(root, text="'Number of cameras detected: %d'" % num_cameras)
    labelu1.pack()

    labelu2 = Label(root, text=" " )
    labelu2.pack()

    labelu3 = Label(root, text=" Which camera do you wish to use? ", bg="yellow")
    labelu3.pack()

    v0 = IntVar()
    v0.set(0)

    #Creating a radiobutton for each camera that can be picked
    for i, cam in enumerate(cam_list):

        nodemap = cam.GetTLDeviceNodeMap()

        node_device_information = PySpin.CCategoryPtr(nodemap.GetNode('DeviceInformation'))

        if PySpin.IsAvailable(node_device_information) and PySpin.IsReadable(node_device_information):
            features = node_device_information.GetFeatures()
            node_feature = PySpin.CValuePtr(features[0])
            Radiobutton(root, text="%s: %s"  % (node_feature.GetName(),node_feature.ToString()), variable=v0, value=i).pack()

    Button(text='Confirm', command=nextstep).pack()
    root.mainloop()


    return (res[0])


def camera_settings(cam):
    expo.configure_exposure(cam)

    return (1)

#This function gets one image from the selected camera every time it is called upin
def acquire_images(cam, nodemap_tldevice):
    global image_data
    try:


        # device_serial_number = ''
        node_device_serial_number = PySpin.CStringPtr(nodemap_tldevice.GetNode('DeviceSerialNumber'))
        if PySpin.IsAvailable(node_device_serial_number) and PySpin.IsReadable(node_device_serial_number):
            # device_serial_number = node_device_serial_number.GetValue()

            try:
                #With getnewimage we can save the current image
                image_result = cam.GetNextImage()

                if image_result.IsIncomplete():
                    print ('Image incomplete with image status %d ...' % image_result.GetImageStatus())

                else:

                    # width = image_result.GetWidth()
                    # height = image_result.GetHeight()
                    # print 'Grabbed Image %d, width = %d, height = %d' % (i, width, height)

                    #Now the saved image has to be modified in order to be read as a numpy array
                    image_converted = image_result.Convert(PySpin.PixelFormat_Mono8, PySpin.HQ_LINEAR)
                    image_data = image_converted.GetNDArray()
                    image_result.Release()


            except PySpin.SpinnakerException as ex:
                print ('Error: %s' % ex)
                return False

    except PySpin.SpinnakerException as ex:
        print ('Error: %s' % ex)
        return False

    #The numpy array that contains the image is just modified in order to get a smaller image
    image_data2 = cv2.resize(image_data, None, fx=0.25, fy=0.25, interpolation=cv2.INTER_CUBIC)

    return image_data2

#This function sets up the camera so that it's ready for acquisition
def create_window(cam,  nodemap):
    try:

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
        print ('Acquiring images...')

    except PySpin.SpinnakerException as ex:
        print ('Error: %s' % ex)
        return False

    return (1)

#If you're using cv2 to display the image this function can update the frames
def update_window(cam, nodemap_tldevice, time):
    image = acquire_images(cam, nodemap_tldevice)
    cv2.imshow('preview', image)
    cv2.waitKey(1)


#This function resets the settings of contrast and gain to their original values, and then ends acquisition
def terminate(cam):
    # plt.close()

    expo.reset_exposure(cam)
    cam.GainAuto.SetValue(PySpin.GainAuto_Continuous)
    cam.EndAcquisition()
    # Deinitialize camera
    cam.DeInit()



    return (True)


def main():

    #----------------------

    # This function will modify the settings once called upon
    def recupere():
        aux[0] = float(entree1.get())
        aux[1] = float(entree2.get())
        aux[2] = int(entree3.get())
        aux[3] = entree4.get()
        aux[4] = 1. / float(entree6.get())
        expo.configure_exposure2(cam, aux[2])
        expo.gain(cam, aux[3])

    # This function saves the current frame once called upon
    def save():
        new_im = Image.fromarray(image5[0])
        new_im = new_im.convert('RGB')
        file = filedialog.asksaveasfile(mode='w', defaultextension=".jpg")
        new_im.save(file)
        print 'Image saved as %s' % file

    # The following functions allow the user to save a video from the moment they press start recording to the moment they press stop recording

    vid = [0]
    vidname = ['hh']

    def video():

        frame_array = []
        while vid[0] == 0:
            frame_array.append(image5[0])
            time.sleep(0.1)
        time.sleep(0.5)

        while vid[0] == 1:
            pass
        name = vidname[0]

        fourcc = cv2.VideoWriter_fourcc(*'MP42')

        video = cv2.VideoWriter('%s' % name, fourcc, float(10), (image5[0].shape[1], image5[0].shape[0]))
        w, h = image5[0].shape
        ret = np.empty((w, h, 3), dtype=np.uint8)
        for frame in frame_array:
            ret[:, :, 0] = frame
            ret[:, :, 1] = frame
            ret[:, :, 2] = frame
            video.write(ret)
        video.release()
        vid[0] = 0
        print('Video saved as %s' % name)

    def start_video():

        thread2 = threading.Thread(target=video)
        thread2.start()

    def stop_vid():
        vid[0] = 1
        vidname[0] = filedialog.asksaveasfile(mode='w', defaultextension='avi').name
        vid[0] = 2

    # This function, once called upon, will allow the while loop in the mainloop threaded function to stop and will then close the window
    def close():
        restart[0] = 1
        aux[-1] = 1
        time.sleep(0.2)
        fenetre.destroy()

    # These two function control q variable that can stop or resume the main loop
    stop = [0]

    def freeze():
        stop[0] = 1

    def resume():
        stop[0] = 0

    # The following functions are designed to thread a process that activates the magnets with the parameters chosen by the user
    def magnet():

        conf = direction.get()
        amp = amplitude.get()
        dur = float(duration.get())

        connection_object = pscc.open_controller()
        time.sleep(1)
        pscc.write_values(connection_object, conf, amp)
        time.sleep(0.02)
        current_controller = threading.Thread(name='current_controller', target=pscc.trigger_currents,
                                              args=(connection_object, dur))
        pscc.light_on(connection_object)
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
        current_controller = threading.Thread(name='current_controller', target=pscc.trigger_currents,
                                              args=(connection_object, dur))
        pscc.light_on(connection_object)
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
        auto[0]=1
        aux[-1] = 1
        time.sleep(0.2)
        fenetre.destroy()



    rows = 50
    columns = 8
    params2 = [[]]

    def automated2():
        auxi2=columns
        auxi=0
        while auxi2==columns and auxi<rows:

            auxi2=0
            for j in range(columns):
                if Value[auxi][j].get()!='':

                    auxi2=auxi2+1
            if auxi2==columns:
                auxi=auxi+1

        params2[0]=[[0 for j in range(columns)]for i in range(auxi)]



        for i in range(0, auxi):
            params2[0][i]=[Value[i][0].get(),int(Value[i][1].get()),int(Value[i][2].get()),float(Value[i][3].get()),float(Value[i][4].get()),int(Value[i][5].get()),float(Value[i][6].get()),int(Value[i][7].get())]

        auto[0]=2
        aux[-1] = 1
        time.sleep(0.2)
        fenetre.destroy()

    #Temperature control

    def set_temp():
        temp[0]+=1

    temp=[2]

    #Sweep volume control

    def set_sweeplim():
        print('ho')

    #Laser control functions

    def onlaser1():
        spim.open_ports()
        spim.lst1=True
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
        pow=float(valueL1.get())/1000.
        spim.pwr1=pow
        spim.laser_power()

    def powlaser2():
        pow=float(valueL2.get())/1000.
        spim.pwr2 = pow
        spim.laser_power()


    def stage():
        pos=float(valueS.get())
        spim.focus(pos)




    #-----------------------------------------------------------
    restart=[0]
    first=0
    auto=[0]

    spim=so.SPIMMM()

    while restart[0]==0:

        t = 0.05
        # Retrieve singleton reference to system object
        system = PySpin.System.GetInstance()

        # Get current library version
        version = system.GetLibraryVersion()
        print ('Library version: %d.%d.%d.%d' % (version.major, version.minor, version.type, version.build))

        # Retrieve list of cameras from the system
        cam_list = system.GetCameras()

        num_cameras = cam_list.GetSize()
        if first==0:
            num=user_selection(cam_list,num_cameras)
        # Pick the camera from the list
        cam = cam_list[num]

        # Initialize camera
        cam.Init()

        nodemap_tldevice = cam.GetTLDeviceNodeMap()

        # Retrieve GenICam nodemap
        nodemap = cam.GetNodeMap()

        # Setting initial values for exposure time and gain
        expo.configure_exposure2(cam, 10)
        expo.gain(cam, 4)

        # Setting up the camera for acquisition
        create_window(cam, nodemap)

        # Array to store the values for contrast, brightness and time exposure in us
        aux = np.array([1., 0., 10, 4., 1., 0])

        # Time between each frame in s
        t = 0.1

        # Initiating the window for the preview's GUI
        fenetre = Tk()
        fenetre.geometry("1300x700")

        def correct(inp):
            time.sleep(0.1)
            if inp.isdigit() or inp=='':
                return(True)
            else:
                return(False)
        reg = fenetre.register(correct)

        def correctf(inp):
            time.sleep(0.1)
            if inp.replace(".", "", 1).isdigit() or inp=='':
                return(True)
            else:
                return(False)
        regf = fenetre.register(correctf)

        def correctfn(inp):
            time.sleep(0.1)
            if inp=='':
                return(True)

            elif inp[0]=='-':
                return(inp.replace(".", "", 1).replace("-", "", 1).isdigit())
            else:
                return(inp.replace(".", "", 1).isdigit())

        regfn = fenetre.register(correctfn)

        image5 = [np.clip(aux[0] * acquire_images(cam, nodemap_tldevice) + aux[1], 0, 255)]
        image6 = [ImageTk.PhotoImage(Image.fromarray(image5[0]), master=fenetre)]

        # Creating the canvas for the image
        canvas = Canvas(fenetre, width=image5[0].shape[1], height=image5[0].shape[0])
        canvas.place(x=0, y=0)

        # Creating the values stored in the different entries
        value1 = StringVar()
        value1.set(1)

        value2 = StringVar()
        value2.set(0)

        value3 = StringVar()
        value3.set(10)

        value4 = StringVar()
        value4.set(4)

        value6 = StringVar()
        value6.set(1)

                # labels, entries and button for the different parameters

        label5 = Label(fenetre, text="Parameters", bg="cyan")
        label5.place(x=855, y=0)

        label6 = Label(fenetre, text="Gamma", bg="yellow")
        label6.place(x=700, y=30)

        entree6 = Entry(fenetre, textvariable=value6, width=10, validate="key",validatecommand=(regf,'%P'))
        entree6.place(x=630, y=30)

        label1 = Label(fenetre, text="Contrast", bg="yellow")
        label1.place(x=700, y=60)

        entree1 = Entry(fenetre, textvariable=value1, width=10, validate="key",validatecommand=(regf,'%P'))
        entree1.place(x=630, y=60)

        label2 = Label(fenetre, text="Brightness", bg="yellow")
        label2.place(x=835, y=30)

        entree2 = Entry(fenetre, textvariable=value2, width=10, validate="key",validatecommand=(regfn,'%P'))
        entree2.place(x=765, y=30)

        label3 = Label(fenetre, text="Exposure time in us (between 7 and 29999998)", bg="yellow")
        label3.place(x=835, y=60)

        entree3 = Entry(fenetre, textvariable=value3, width=10, validate="key",validatecommand=(reg,'%P'))
        entree3.place(x=765, y=60)

        label4 = Label(fenetre, text="Gain in Db (below 47.994294)", bg="yellow")
        label4.place(x=985, y=30)

        entree4 = Entry(fenetre, textvariable=value4, width=10, validate="key",validatecommand=(regf,'%P'))
        entree4.place(x=915, y=30)

        bouton1 = Button(fenetre, text="Change settings", command=recupere)
        bouton1.place(x=1100, y=60)

        # Label, entry and button to save the image
        label5 = Label(fenetre, text="Save a frame or a video", bg="cyan")
        label5.place(x=660, y=110)

        bouton2 = Button(fenetre, text="Save the current image", command=save)
        bouton2.place(x=660, y=140)

        # Label, entry and button to record a video

        bouton7 = Button(fenetre, text="Start recording", command=start_video)
        bouton7.place(x=632, y=170)
        bouton8 = Button(fenetre, text="Stop recording", command=stop_vid)
        bouton8.place(x=722, y=170)

        Label(fenetre, text=" ").pack(side=TOP)

        # Exit button
        bouton5 = Button(fenetre, text="Exit", command=close, bg='red')
        bouton5.place(x=400, y=520)

        # Freeze and Resume button

        bouton3 = Button(fenetre, text="Freeze", command=freeze)
        bouton3.place(x=250, y=520)

        bouton4 = Button(fenetre, text="Resume", command=resume)
        bouton4.place(x=300, y=520)

        # Magnet control

        label7 = Label(fenetre, text="Magnets control", bg="Cyan")
        label7.place(x=950, y=110)


        #Select a direction for pulling with the magnet
        direction = IntVar()
        direction.set(1)

        label7 = Label(fenetre, text="Pick a direction:", bg="yellow")
        label7.place(x=870, y=140)
        boutonN = Radiobutton(fenetre, text="North (3)", variable=direction, value=3)
        boutonN.place(x=845, y=170)
        boutonS = Radiobutton(fenetre, text="South (2)", variable=direction, value=2)
        boutonS.place(x=920, y=170)
        boutonE = Radiobutton(fenetre, text="East (4)", variable=direction, value=4)
        boutonE.place(x=845, y=200)
        boutonW = Radiobutton(fenetre, text="West (1)", variable=direction, value=1)
        boutonW.place(x=920, y=200)

        #Select an amplitude

        amplitude = IntVar()
        amplitude.set(1)
        label8 = Label(fenetre, text="Pick an amplitude:", bg="yellow")
        label8.place(x=1065, y=140)
        boutonA1 = Radiobutton(fenetre, text="0.5 A (1)", variable=amplitude, value=1)
        boutonA1.place(x=1020, y=170)
        boutonA2 = Radiobutton(fenetre, text="1 A (2)", variable=amplitude, value=2)
        boutonA2.place(x=1090, y=170)
        boutonA3 = Radiobutton(fenetre, text="1.5 A (3)", variable=amplitude, value=3)
        boutonA3.place(x=1160, y=170)
        boutonA4 = Radiobutton(fenetre, text="2 A (4)", variable=amplitude, value=4)
        boutonA4.place(x=1020, y=200)
        boutonA5 = Radiobutton(fenetre, text="2.5 A (5)", variable=amplitude, value=5)
        boutonA5.place(x=1090, y=200)

        valued = StringVar()
        valued.set(1)

        duration = Entry(fenetre, textvariable=valued, width=10,validate="key",validatecommand=(regf,'%P'))
        duration.place(x=1075, y=230)
        boutonM1 = Button(fenetre, text="Run magnet for following duration (in s):", command=start_magnet)
        boutonM1.place(x=845, y=227)
        boutonM2 = Button(fenetre, text="Pulse", command=start_magnet2)
        boutonM2.place(x=1190, y=227)

        # Temperature
        label5 = Label(fenetre, text="Temperature control", bg="cyan")
        label5.place(x=670, y=240)

        valueT = StringVar()
        valueT.set(2)



        EntreeT = Entry(fenetre, textvariable=valueT, width=10)
        EntreeT.place(x=770, y=270)

        EntreeT.config(validate="key",validatecommand=(reg,'%P'))
        boutonT1 = Button(fenetre, text="Set the temperature to", command=set_temp)
        boutonT1.place(x=630, y=267)

        # Volume sweep section

        labelSV = Label(fenetre, text="Sweep volume limits", bg="cyan")
        labelSV.place(x=670, y=360)

        valueSV1 = StringVar()
        valueSV1.set(0)

        valueSV2 = StringVar()
        valueSV2.set(2)

        label1 = Label(fenetre, text="Upper limit", bg="yellow")
        label1.place(x=730, y=390)

        entreeSV1 = Entry(fenetre, textvariable=valueSV1, width=10, validate="key",validatecommand=(regfn,'%P'))
        entreeSV1.place(x=660, y=390)

        label1 = Label(fenetre, text="Lower limit", bg="yellow")
        label1.place(x=730, y=420)

        entreeSV2 = Entry(fenetre, textvariable=valueSV2, width=10, validate="key",validatecommand=(regfn,'%P'))
        entreeSV2.place(x=660, y=420)

        boutonSV = Button(fenetre, text="Set Sweep volume limits", command=set_sweeplim)
        boutonSV.place(x=660, y=450)

        #Lasers control

        valueL1 = StringVar()
        valueL1.set(10)

        valueL2 = StringVar()
        valueL2.set(10)

        labelL1 = Label(fenetre, text="Lasers control", bg="Cyan")
        labelL1.place(x=960, y=300)

        labelL2 = Label(fenetre, text="488 Laser:", bg="yellow")
        labelL2.place(x=830, y=335)

        boutonL1 = Button(fenetre, text="Turn on", command=onlaser1)
        boutonL1.place(x=895, y=332)

        boutonL2 = Button(fenetre, text="Turn off", command=offlaser1)
        boutonL2.place(x=950, y=332)

        boutonL3 = Button(fenetre, text="Set laser power (in mW) to:", command=powlaser1)
        boutonL3.place(x=1030, y=332)

        entreeL1 = Entry(fenetre, textvariable=valueL1, width=10, validate="key",validatecommand=(reg,'%P'))
        entreeL1.place(x=1185, y=335)



        labelL3 = Label(fenetre, text="561 Laser:", bg="yellow")
        labelL3.place(x=830, y=370)

        boutonL4 = Button(fenetre, text="Turn on", command=onlaser2)
        boutonL4.place(x=895, y=367)

        boutonL5 = Button(fenetre, text="Turn off", command=offlaser2)
        boutonL5.place(x=950, y=367)

        boutonL6 = Button(fenetre, text="Set laser power (in mW) to:", command=powlaser2)
        boutonL6.place(x=1030, y=367)

        entreeL2 = Entry(fenetre, textvariable=valueL2, width=10, validate="key",validatecommand=(reg,'%P'))
        entreeL2.place(x=1185, y=370)

        #Stage

        valueS = StringVar()
        valueS.set(1)

        boutonS1 = Button(fenetre, text="Set stage to:", command=stage)
        boutonS1.place(x=900, y=420)

        entreeS1 = Entry(fenetre, textvariable=valueS, width=10, validate="key",validatecommand=(regfn,'%P'))
        entreeS1.place(x=980, y=420)

        #Automated experiment part

        labelA1 = Label(fenetre, text="Automated experiment", bg="Cyan")
        labelA1.place(x=550, y=550)

        boutonA1 = Button(fenetre, text="Run automated experiment from file", command=automated1)
        boutonA1.place(x=100, y=620)

        boutonA2 = Button(fenetre, text="Run automated experiment with parameters on the right", command=automated2)
        boutonA2.place(x=50, y=650)

        labelA2 = Label(fenetre, text="Design your own automated experiment:")
        labelA2.place(x=500, y=580)


        #Labels and buttons for the automated experiment
        labelA3 = Label(fenetre, text="Name", bg="yellow")
        labelA3.place(x=400, y=610)

        labelA4 = Label(fenetre, text="Amplitude", bg="yellow")
        labelA4.place(x=494, y=610)

        labelA4 = Label(fenetre, text="Configuration", bg="yellow")
        labelA4.place(x=588, y=610)

        labelA4 = Label(fenetre, text="Force on", bg="yellow")
        labelA4.place(x=682, y=610)

        labelA5 = Label(fenetre, text="Force duration", bg="yellow")
        labelA5.place(x=776, y=610)

        labelA6 = Label(fenetre, text="Num frames", bg="yellow")
        labelA6.place(x=870, y=610)

        labelA7 = Label(fenetre, text="Frame period", bg="yellow")
        labelA7.place(x=964, y=610)

        labelA8 = Label(fenetre, text="Temperature", bg="yellow")
        labelA8.place(x=1058, y=610)




        #----------------------



        frame_main = Frame(fenetre, bg="gray")
        frame_main.grid(sticky='news')
        frame_main.place(x=400,y=630)

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
        Value =  [[StringVar() for j in range(columns)] for i in range(rows)]

        for i in range(0, rows):
            for j in range(0, columns):
                if first==0:
                    Value[i][j].set('')
                if i==0:
                    Entree[i][j] = Entry(frame_buttons, textvariable=Value[i][j], width=15)
                else:
                    Entree[i][j] = Entry(frame_buttons,textvariable=Value[i][j], width=15,validate="key",validatecommand=(regf,'%P'))
                Entree[i][j].grid(row=i, column=j, sticky='news')

        # Update buttons frames idle tasks to let tkinter calculate buttons sizes
        frame_buttons.update_idletasks()

        # Resize the canvas frame to show exactly 5-by-5 buttons and the scrollbar
        first5columns_width = sum([Entree[0][j].winfo_width() for j in range(0, 8)])
        first5rows_height = sum([Entree[i][0].winfo_height() for i in range(0, 3)])
        frame_canvas.config(width=first5columns_width + vsb.winfo_width(),
                            height=first5rows_height)

        # Set the canvas scrolling region
        canvas2.config(scrollregion=canvas2.bbox("all"))

        #----------------------

        # Placing the first image in the canvas
        image_on_canvas = canvas.create_image(20, 20, anchor=NW, image=image6[0])

        # a = [1]

        def main_loop():

            while aux[-1] != 1:

                while stop[0] == 1:
                    pass
                # Quick pause
                time.sleep(0.01)
                # Acquiring the new image, changing contrast and brightness, and changing its format so that it's compatible with tkinter
                image5[0] = np.clip(aux[0] * acquire_images(cam, nodemap_tldevice) ** (aux[4]) + aux[1], 0, 255)
                image6[0] = ImageTk.PhotoImage(Image.fromarray(image5[0]), master=fenetre)
                # Changing the image on the canvas
                canvas.itemconfigure(image_on_canvas, image=image6[0])

                labelT = Label(fenetre, text=" TEMPERATURE: %s" % temp[0], bg='white')
                labelT.place(x=680, y=300)

                # canvas.itemconfigure(image_on_canvas, image=image6[0])
        time.sleep(1)
        thread = threading.Thread(target=main_loop)
        thread.start()

        fenetre.mainloop()

        thread.join()

        # Terminating the camera
        terminate(cam)

        cam = None

        # Clear camera list before releasing system
        cam_list.Clear()

        # Release system instance
        system.ReleaseInstance()

        del system
        if auto[0]==1:
            sae.function()
        if auto[0]==2:
            sae.function(2,params2[0])
        auto[0]=0
        first=1

    spim.close_ports()

    print(' ')
    print('blackfly_preview successfully exited')

    return (True)


if __name__ == '__main__':
    main()
