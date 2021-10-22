# noam demri nsd31 and fergus riche fr293 wrote this to control the Selective Plane Illumination Magnetic Manipulator
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
import sys
import spimm_automated_experiment as sae
import spimmm_obj as so
import Trigger as Tr


class TriggerType:
    SOFTWARE = 1
    HARDWARE = 2

    def __init__(self):
        pass


CHOSEN_TRIGGER = TriggerType.HARDWARE
delay = [0.1]

global image_data


# a deprecated variable to track a user selected camera choice
# global num


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
    # print('hhh %s' % cam.AcquisitionFrameRateEnable())
    print('max frame rate= %s' % cam.AcquisitionFrameRate.GetMax())
    cam.AcquisitionFrameRate.SetValue(cam.AcquisitionFrameRate.GetMax())
    print('resulting frame rate: %s Hz' % cam.AcquisitionResultingFrameRate())
    # cam.AcquisitionFrameRate.SetValue(cam.AcquisitionFrameRate.GetMax())


# This function gets one image from the selected camera every time it is called upon
def acquire_images(cam, nodemap_tldevice, timestamp=False):
    # global image_data
    try:
        node_device_serial_number = PySpin.CStringPtr(nodemap_tldevice.GetNode('DeviceSerialNumber'))
        if PySpin.IsAvailable(node_device_serial_number) and PySpin.IsReadable(node_device_serial_number):
            # device_serial_number = node_device_serial_number.GetValue()
            try:
                image_result = cam.GetNextImage()

                if image_result.IsIncomplete():
                    print ('Image incomplete with image status %d ...' % image_result.GetImageStatus())
                else:
                    image_converted = image_result.Convert(PySpin.PixelFormat_Mono8, PySpin.HQ_LINEAR)
                    image_array = image_converted.GetNDArray()
                    image_result.Release()
                    # The numpy array that contains the image is just modified in order to get a smaller image
                    # image_data2 = cv2.resize(image_data, None, fx=ratio, fy=ratio, interpolation=cv2.INTER_CUBIC)
                    if timestamp:
                        return image_array, image_result.GetTimeStamp()
                    else:
                        return image_array

            except PySpin.SpinnakerException as ex:
                print ('Error: %s' % ex)
                return False

    except PySpin.SpinnakerException as ex:
        print ('Error: %s' % ex)
        return False


def acquire_images2(cam_list):
    # adapted from the multiple acquisition example
    # In order to work with simultaneous camera streams, nested loops are
    # needed. It is important that the inner loop be the one iterating
    # through the cameras; otherwise, all images will be grabbed from a
    # single camera before grabbing any images from another.
    try:
        result = []
        for i, cam in enumerate(cam_list):
            try:
                # Retrieve next received image and ensure image completion
                image_result = cam.GetNextImage()

                if image_result.IsIncomplete():
                    print'Image incomplete with image status %d ... \n' % image_result.GetImageStatus()
                else:
                    image_converted = image_result.Convert(PySpin.PixelFormat_Mono16, PySpin.HQ_LINEAR)
                    image_array = image_converted.GetNDArray()
                    image_array2 = cv2.resize(image_array, None, fx=0.25, fy=0.25, interpolation=cv2.INTER_CUBIC)
                    result.append(image_array2)
                    return result

                # Release image
                image_result.Release()

            except PySpin.SpinnakerException as ex:
                print'Error: %s' % ex

    except PySpin.SpinnakerException as ex:
        print'Error: %s' % ex


def camera_mode(cam, mode):
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
        camera_mode(cam, 'NewestOnly')
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
    stop = [0]
    vid = [0]
    vidname = ['hh']

    def change_camera1_params():
        aux[0] = float(cam1_contrast_field.get())
        aux[1] = float(cam1_brightness_field.get())
        aux[2] = int(cam1_exposure_field.get())
        aux[3] = cam1_gain_field.get()
        aux[4] = 1. / float(cam1_gamma_field.get())

        expo.configure_exposure2(cam_list[0], aux[2])
        expo.gain(cam_list[0], aux[3])

    def change_camera2_params():
        aux2[0] = float(cam2_contrast_field.get())
        aux2[1] = float(cam2_brightness_field.get())
        aux2[2] = int(cam2_exposure_field.get())
        aux2[3] = cam2_gain_field.get()
        aux2[4] = 1. / float(cam1_gamma_field.get())

        expo.configure_exposure2(cam_list[1], aux2[2])
        expo.gain(cam_list[1], aux2[3])

    # This function saves the current frame once called upon
    def save():
        image27 = np.zeros(np.shape(image7), dtype=np.int64)

        for row in range(np.shape(image7)[0]):
            for col in range(np.shape(image7)[1]):

                if image7[row, col, 1] < 0:
                    image27[row, col, 1] = image7[row, col, 1] + 255
                else:
                    image27[row, col, 1] = image7[row, col, 1]

                if image7[row, col, 0] < 0:
                    image27[row, col, 0] = image7[row, col, 0] + 255
                else:
                    image27[row, col, 0] = image7[row, col, 0]

        truc = (image27[:, :, 1] + image27[:, :, 0]) / 2

        image37 = np.zeros((np.shape(image7)[0], np.shape(image7)[1]), dtype=np.uint16)

        for row in range(np.shape(image7)[0]):
            for col in range(np.shape(image7)[1]):

                if truc[row, col] > 127:
                    image37[row, col] = truc[row, col] - 255

                else:
                    image37[row, col] = truc[row, col]
        # print(truc)
        # new_im = Image.fromarray(truc)

        # new_im = new_im.convert('RGB')

        # print(image27)

        # print(np.min(image27))
        # print(np.max(image27))

        pic_que = Queue.Queue()
        filepath = filedialog.asksaveasfilename(initialdir="/", title="Select file",
                                                filetypes=(("tiff files", "*.tiff"), ("all files", "*.*")))
        if filepath is not None:
            if save_colour_setting.get() == 1:
                pic_que.put(image7)
                with imageio.get_writer(filepath + '.tiff') as stack:
                    while not pic_que.empty():
                        stack.append_data(pic_que.get())
                # cv2.imwrite("%s.png" % file,image27)
            else:
                pic_que.put(image37)

                with imageio.get_writer(filepath + '.tiff') as stack:
                    while not pic_que.empty():
                        stack.append_data(pic_que.get())
            #             cv2.imwrite("%s.png" % file, truc)
            print 'Image saved as %s.tiff' % filepath

    #
    # The following functions allow the user to save a video from the moment they press start recording to the moment
    # they press stop recording

    def video():
        pic_que = Queue.Queue()
        time_que = Queue.Queue()

        while vid[0] == 0:
            pic_que.put(np.copy(image7))
            pictime = time.time()
            time_que.put(pictime)
            plt.pause(0.1)
        time.sleep(0.5)

        while vid[0] == 1:
            pass
        filepath = vidname[0]

        if save_colour_setting.get() == 1:
            # for i in range(np.shape(frame_array)[0]):
            # print(frame_array[i])
            # ret[:, :, 0] = frame_array[i][:, :, 2]
            # ret[:, :, 1] = frame_array[i][:, :, 1]
            # ret[:, :, 2] = frame_array[i][:, :, 0]
            # video2.write(ret)
            with imageio.get_writer(filepath + '.tiff') as stack:
                with open(filepath + '_time.csv', 'ab') as f:
                    writer = csv.writer(f)
                    while not pic_que.empty():
                        auxi = pic_que.get()
                        auxit = time_que.get()
                        stack.append_data(auxi)
                        writer.writerow([auxit])
        else:
            with imageio.get_writer(filepath + '.tiff') as stack:
                with open(filepath + '_time.csv', 'ab') as f:
                    writer = csv.writer(f)
                    while not pic_que.empty():
                        interm = pic_que.get()

                        image27 = np.zeros(np.shape(image7), dtype=np.uint16)

                        for row in range(np.shape(image7)[0]):
                            for col in range(np.shape(image7)[1]):

                                if interm[row, col, 1] < 0:
                                    image27[row, col, 1] = interm[row, col, 1] + 255
                                else:
                                    image27[row, col, 1] = interm[row, col, 1]

                                if image7[row, col, 0] < 0:
                                    image27[row, col, 0] = interm[row, col, 0] + 255
                                else:
                                    image27[row, col, 0] = interm[row, col, 0]

                        truc = (image27[:, :, 1] + image27[:, :, 0]) / 2

                        image37 = np.zeros((np.shape(image7)[0], np.shape(image7)[1]), dtype=np.int16)

                        for row in range(np.shape(image7)[0]):
                            for col in range(np.shape(image7)[1]):

                                if truc[row, col] > 127:
                                    image37[row, col] = truc[row, col] - 255

                                else:
                                    image37[row, col] = truc[row, col]

                        stack.append_data(image37)
                        writer.writerow([time_que.get()])
            # for i in range(np.shape(frame_array)[0]):
            # print(frame_array[i])
            #   ret[:, :, 0] = (frame_array[i][:, :, 2]+frame_array[i][:, :, 1]+frame_array[i][:, :, 0])/3
            #  ret[:, :, 1] = (frame_array[i][:, :, 2]+frame_array[i][:, :, 1]+frame_array[i][:, :, 0])/3
            # ret[:, :, 2] = (frame_array[i][:, :, 2]+frame_array[i][:, :, 1]+frame_array[i][:, :, 0])/3
            # video2.write(ret)

        # video2.release()
        vid[0] = 0
        print('Video saved as %s.tiff' % filepath)

        stop_record_button.configure(state='disabled')
        start_record_button.configure(state='normal')

    def start_video():
        start_record_button.configure(state='disabled')
        stop_record_button.configure(state='normal')
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
        window.destroy()

    # These two function control a variable that can stop or resume the main loop


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
        mag_button.config(state='disabled')
        mag_pulse_button.config(state='disabled')
        plt.pause(time_stop)
        mag_button.config(state='normal')
        mag_pulse_button.config(state='normal')

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

    def pulse_magnet():
        bout = threading.Thread(target=button_control)
        bout.start()

        mag = threading.Thread(target=magnet2)
        mag.start()

    # The following functions allow you to run automated experiments

    def automated1():
        auto[0] = 1
        aux[-1] = 1
        time.sleep(0.2)
        window.destroy()

    rows = 50
    columns = 8
    params2 = [[]]

    def automated2():
        auxi2 = columns
        auxi = 0
        while auxi2 == columns and auxi < rows:
            auxi2 = 0
            for col in range(columns):
                if value[auxi][col].get() != '':
                    auxi2 = auxi2 + 1
            if auxi2 == columns:
                auxi = auxi + 1
        params2[0] = [[0 for col in range(columns)] for row in range(auxi)]
        for rank in range(0, auxi):
            params2[0][rank] = [value[rank][0].get(), int(value[rank][1].get()), int(value[rank][2].get()),
                                float(value[rank][3].get()), float(value[rank][4].get()), int(value[rank][5].get()),
                                float(value[rank][6].get()), int(value[rank][7].get())]
        auto[0] = 2
        aux[-1] = 1
        time.sleep(0.2)
        window.destroy()

    # Temperature control

    def set_temp():
        # block volume button
        sweep_vol_button.config(state='disabled')
        spim.tem = float(temperature_input.get())
        spim.starttempcont()
        temperature_readout.configure(text="Bath temp: %s" % spim.tempm)

    def halt_temp():
        # release volume button
        sweep_vol_button.config(state='normal')
        spim.halttempcont()
        print('halt temp')

    # Sweep volume control

    def sweep():
        print('Starting sweep')

        spim.dlo = float(lower_z_input.get())
        spim.dup = float(upper_z_input.get())
        spim.slp = float(mirror_slope_input.get())
        spim.exp = exposure_input.get() / 1000
        spim.frt = frame_period_input.get() / 1000
        spim.ste = float(z_step_input.get())

        spim.sendcfg()
        spim.readcfg()

        stop[0] = 1

        time.sleep(0.1)
        set_trigger()

        for camera in cam_list:
            camera_mode(camera, "NewestFirst")
        sw = threading.Thread(target=sweep_thread)
        # spim.readcfg()
        if save_volume_input.get() == 1:
            ss = threading.Thread(target=save_sweep)
            ss.start()

        sw.start()

        # while vid[0]!=1:
        #    pass
        # stop_vid()
        # print('hi')

    def sweep_thread():
        spim.tkv()

    def save_sweep():
        sweeping_indicator[0].place(x=250, y=250)
        pos_que = Queue.Queue()

        while True:
            response = spim.ard.readline()
            if response[-2:] == 'o\n':
                pos_que.put(response)
            if response == "VOL\n":
                break

        pic_pile = []
        time_pile1 = []
        time_pile2 = []
        print(pos_que.qsize() - 1)
        image72 = np.zeros((dim[0], dim[1], 3), dtype=np.uint16)
        image7_aux_inner = np.zeros((dim[0], dim[1]), dtype=np.uint16)
        for item in range(pos_que.qsize()):
            aux_thing1 = acquire_images(cam_list[0], nodemap_tldevice[0], False)
            aux_thing2 = acquire_images(cam_list[1], nodemap_tldevice[1], False)
            thing1 = np.clip(aux[0] * aux_thing1[0] ** (aux[4]) + aux[1], 0, 255)
            thing2 = np.clip(aux2[0] * aux_thing2[0] ** (aux2[4]) + aux2[1], 0, 255)

            if camera_display[0].get() == 1:
                image72[:, :, 1] = np.copy(thing1)
            else:
                image72[:, :, 1] = np.copy(image7_aux_inner)
            if camera_display[1].get() == 1:
                image72[:, :, 0] = np.copy(thing2)
            else:
                image72[:, :, 0] = np.copy(image7_aux_inner)
            pic_pile.append(np.copy(image72))

            time_pile1.append(aux_thing1[1] / 10.0 ** 9)
            time_pile2.append(aux_thing2[1] / 10.0 ** 9)

        volume_filename = filename_imput.get()

        with imageio.get_writer(volume_filename + '.tiff', bigtiff=True) as stack:
            with open(volume_filename + '_position.csv', 'ab') as f:
                writer = csv.writer(f)
                while not pos_que.empty():
                    position = pos_que.get()
                    capture_time_1 = time_pile1.pop()
                    capture_time_2 = time_pile2.pop()
                    writer.writerow([str(capture_time_1) + ';' + str(capture_time_2) + ';' + position[:5] + ';' + position[6:-1]])
                    image = pic_pile.pop()
                    stack.append_data(image)

        print('Volume swept')
        for cameras in cam_list:
            camera_mode(cameras, "NewestOnly")
        set_continuous()
        sweeping_indicator[0].place(x=-1000, y=-1000)

        stop[0] = 0

    def volumestate():
        if save_volume_input.get() == 0:
            exposure_field.config(state='disabled')
        if save_volume_input.get() == 1:
            exposure_field.config(state='normal')

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
        power = float(yellow_power_input.get()) / 1000.
        spim.pwr1 = power
        spim.laser_power()

    def powlaser2():
        power = float(yellow_power_input.get()) / 1000.
        spim.pwr2 = power
        spim.laser_power()

    def laser():
        spim.open_ports()
        spim.pwr1 = float(yellow_power_input.get()) / 1000.
        spim.pwr2 = float(yellow_power_input.get()) / 1000.

        if blue_state_input.get() == 1:
            spim.lst1 = True
        else:
            spim.lst1 = False

        if yellow_state_input.get() == 1:
            spim.lst2 = True
        else:
            spim.lst2 = False

        spim.laser_power()

    # Stage control

    def mirror():
        spim.posadj = float(stage_pos_input.get())

    def stage():
        pos = float(stage_pos_input.get())

        if mirror_track_input.get() == 0:
            spim.stage(pos)
        if mirror_track_input.get() == 1:
            spim.focus(pos)

    def engage():
        stage_engage_button.config(state='disabled')
        spim.rbt()
        plt.pause(3)
        stage_disengage_button.config(state='normal')
        spim.engage()
        stage_update_button.config(state='normal')

    def disengage():
        stage_disengage_button.config(state='disabled')
        stage_engage_button.config(state='normal')
        spim.engage()
        stage_update_button.config(state='disabled')

    def raisestage():
        if spim.eng:
            spim.stage(-5)
        else:
            spim.engage()
            spim.stage(-5)
            plt.pause(2)
            spim.disengage()

    def stagerr():
        error_code = spim.err()[-2]
        stage_error_label.configure(text=' %s ' % error_code)

    # Trigger mode

    def set_trigger():
        trigger_mode_button.config(state='disabled')
        single_trigger_button.config(state='normal')
        software_trigger_button.config(state='normal')
        spim.exp = exposure_input.get() / 1000
        spim.sendcfg()
        for camera in cam_list:
            Tr.configure_trigger(camera)

    def set_continuous():
        trigger_mode_button.config(state='normal')
        single_trigger_button.config(state='disabled')
        software_trigger_button.config(state='disabled')
        for count, camera in enumerate(cam_list):
            Tr.reset_trigger(nodemap[count])

    def trigger():
        spim.exp = exposure_input.get() / 1000
        spim.sendcfg()
        print(spim.exp)
        spim.ard.write('FRM2\r')

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
        # if first == 0:
        #    num = user_selection(cam_list, num_cameras)
        # Pick the camera from the list
        # cam = cam_list[num]

        # Initialize camera
        for cam in cam_list:
            cam.Init()

        nodemap_tldevice = [cam.GetTLDeviceNodeMap() for cam in cam_list]

        # Retrieve GenICam nodemap
        nodemap = [cam.GetNodeMap() for cam in cam_list]

        # Setting initial values for exposure time and gain
        for row, cam in enumerate(cam_list):
            expo.configure_exposure2(cam, 10000)
            expo.gain(cam, 4)
            Tr.reset_trigger(nodemap[row])

        # Setting up the camera for acquisition
        for row in range(num_cameras):
            create_window(cam_list[row], nodemap[row])

        # Array to store the values for contrast, brightness and time exposure in us
        aux = np.array([1., 0., 10000, 4., 1., 0])
        aux2 = np.array([1., 0., 10000, 4., 1., 0])

        # Initiating the window for the preview's GUI
        window = Tk()
        window.title('Single Plane Illumination Magnetic Micro Manipulator')
        window.geometry("1300x700")

        # Defining functions to validate entries----
        def correct(inp):
            if inp.isdigit() or inp == '':
                return True
            else:
                return False

        reg = window.register(correct)

        def correctf(inp):
            if inp.replace(".", "", 1).isdigit() or inp == '':
                return True
            else:
                return False

        regf = window.register(correctf)

        def correctfn(inp):

            if inp == '' or inp == '-':
                return True

            elif inp[0] == '-':
                return inp.replace(".", "", 1).replace("-", "", 1).isdigit()
            else:
                return inp.replace(".", "", 1).isdigit()

        regfn = window.register(correctfn)

        camera_display = [IntVar() for cam in cam_list]

        for row, cam in enumerate(cam_list):

            nodemap2 = cam.GetTLDeviceNodeMap()

            node_device_information = PySpin.CCategoryPtr(nodemap2.GetNode('DeviceInformation'))

            if PySpin.IsAvailable(node_device_information) and PySpin.IsReadable(node_device_information):
                features = node_device_information.GetFeatures()
                node_feature = PySpin.CValuePtr(features[0])
                Checkbutton(window, text="cam: %s" % node_feature.ToString(), variable=camera_display[row]).place(x=20 + row * 100,
                                                                                                                  y=520)

        dim = np.shape(acquire_images(cam_list[0], nodemap_tldevice[0]))

        image5 = [np.clip(aux[0] * cv2.resize(acquire_images(cam_list[row], nodemap_tldevice[row]), None, fx=0.25, fy=0.25,
                                              interpolation=cv2.INTER_CUBIC) ** (aux[4]) + aux[1], 0, 255) for row in
                  range(num_cameras)]

        image7 = np.zeros((np.shape(image5[0])[0], np.shape(image5[0])[1], 3), dtype=np.int16)
        for row in range(num_cameras):
            if camera_display[row].get() == 1:
                image7[:, :, row] = image5[row]

        image6 = [ImageTk.PhotoImage(Image.fromarray(image7, mode="RGB"), master=window)]

        # Creating the canvas for the image
        canvas = Canvas(window, width=image5[0].shape[1] + 25, height=image5[0].shape[0] + 25)
        canvas.place(x=-18, y=-18)

        # Creating the values stored in the different entries
        cam1_contrast = StringVar()
        cam1_contrast.set(1)

        cam1_brightness = StringVar()
        cam1_brightness.set(0)

        cam1_exposure = StringVar()
        cam1_exposure.set(10000)

        cam1_gain = StringVar()
        cam1_gain.set(4)

        cam1_gamma = StringVar()
        cam1_gamma.set(1)

        cam2_contrast = StringVar()
        cam2_contrast.set(1)

        cam2_brightness = StringVar()
        cam2_brightness.set(0)

        cam2_exposure = StringVar()
        cam2_exposure.set(10000)

        cam2_gain = StringVar()
        cam2_gain.set(4)

        cam2_gamma = StringVar()
        cam2_gamma.set(1)

        # labels, entries and button for the different parameters of the first camera

        for row, cam in enumerate(cam_list):

            nodemap2 = cam.GetTLDeviceNodeMap()

            node_device_information = PySpin.CCategoryPtr(nodemap2.GetNode('DeviceInformation'))

            if PySpin.IsAvailable(node_device_information) and PySpin.IsReadable(node_device_information):
                features = node_device_information.GetFeatures()
                node_feature = PySpin.CValuePtr(features[0])
                save_section_label = Label(window, text="Parameters for camera %s" % node_feature.ToString(), bg="cyan")
                save_section_label.place(x=800, y=row * 100)

        cam1_gamma_label = Label(window, text="Gamma", bg="yellow")
        cam1_gamma_label.place(x=700, y=30)

        cam1_gamma_field = Entry(window, textvariable=cam1_gamma, width=10, validate="key", validatecommand=(regf, '%P'))
        cam1_gamma_field.place(x=630, y=30)

        cam1_contrast_label = Label(window, text="Contrast", bg="yellow")
        cam1_contrast_label.place(x=700, y=60)

        cam1_contrast_field = Entry(window, textvariable=cam1_contrast, width=10, validate="key", validatecommand=(regf, '%P'))
        cam1_contrast_field.place(x=630, y=60)

        cam1_brightness_label = Label(window, text="Brightness", bg="yellow")
        cam1_brightness_label.place(x=835, y=30)

        cam1_brightness_field = Entry(window, textvariable=cam1_brightness, width=10, validate="key", validatecommand=(regfn, '%P'))
        cam1_brightness_field.place(x=765, y=30)

        cam1_exposure_label = Label(window, text="Exposure time in us", bg="yellow")
        cam1_exposure_label.place(x=835, y=60)

        cam1_exposure_field = Entry(window, textvariable=cam1_exposure, width=10, validate="key", validatecommand=(reg, '%P'))
        cam1_exposure_field.place(x=765, y=60)

        cam1_gain_label = Label(window, text="Gain in Db", bg="yellow")
        cam1_gain_label.place(x=985, y=30)

        cam1_gain_field = Entry(window, textvariable=cam1_gain, width=10, validate="key", validatecommand=(regf, '%P'))
        cam1_gain_field.place(x=915, y=30)

        cam1_change_params_button = Button(window, text="Change settings", command=change_camera1_params())
        cam1_change_params_button.place(x=1100, y=60)

        # labels, entries and button for the different parameters of the second camera

        cam2_gamma_label = Label(window, text="Gamma", bg="yellow")
        cam2_gamma_label.place(x=700, y=130)

        cam2_gamma_field = Entry(window, textvariable=cam2_gamma, width=10, validate="key", validatecommand=(regf, '%P'))
        cam2_gamma_field.place(x=630, y=130)

        cam2_contrast_label = Label(window, text="Contrast", bg="yellow")
        cam2_contrast_label.place(x=700, y=160)

        cam2_contrast_field = Entry(window, textvariable=cam2_contrast, width=10, validate="key", validatecommand=(regf, '%P'))
        cam2_contrast_field.place(x=630, y=160)

        cam2_brightness_label = Label(window, text="Brightness", bg="yellow")
        cam2_brightness_label.place(x=835, y=130)

        cam2_brightness_field = Entry(window, textvariable=cam2_brightness, width=10, validate="key", validatecommand=(regfn, '%P'))
        cam2_brightness_field.place(x=765, y=130)

        cam2_exposure_label = Label(window, text="Exposure time in us", bg="yellow")
        cam2_exposure_label.place(x=835, y=160)

        cam2_exposure_field = Entry(window, textvariable=cam2_exposure, width=10, validate="key", validatecommand=(reg, '%P'))
        cam2_exposure_field.place(x=765, y=160)

        cam2_gain_label = Label(window, text="Gain in Db", bg="yellow")
        cam2_gain_label.place(x=985, y=130)

        cam2_gain_field = Entry(window, textvariable=cam2_gain, width=10, validate="key", validatecommand=(regf, '%P'))
        cam2_gain_field.place(x=915, y=130)

        cam2_change_params_button = Button(window, text="Change settings", command=change_camera2_params())
        cam2_change_params_button.place(x=1100, y=160)

        # Label, entry and button to save the image
        save_section_label = Label(window, text="Save a frame or a video", bg="cyan")
        save_section_label.place(x=660, y=210)

        save_colour_setting = IntVar()
        save_colour_setting.set(1)

        save_bw_radiobutton = Radiobutton(window, text='Grayscale', variable=save_colour_setting, value=0)
        save_bw_radiobutton.place(x=655, y=235)

        save_colour_radiobutton = Radiobutton(window, text='Color', variable=save_colour_setting, value=1)
        save_colour_radiobutton.place(x=735, y=235)

        save_image_button = Button(window, text="Save the current image", command=save)
        save_image_button.place(x=660, y=260)

        # Label, entry and button to record a video

        start_record_button = Button(window, text="Start recording", command=start_video)
        start_record_button.place(x=632, y=290)
        stop_record_button = Button(window, text="Stop recording", command=stop_vid, state='disabled')
        stop_record_button.place(x=722, y=290)

        # Freeze and Resume button

        freeze_video_button = Button(window, text="Freeze", command=freeze)
        freeze_video_button.place(x=250, y=520)

        resume_video_button = Button(window, text="Resume", command=resume)
        resume_video_button.place(x=300, y=520)

        # Magnet control

        force_direction_label = Label(window, text="Magnets control", bg="Cyan")
        force_direction_label.place(x=950, y=210)

        # Select a direction for pulling with the magnet
        direction = IntVar()
        direction.set(1)

        force_direction_label = Label(window, text="force direction:", bg="yellow")
        force_direction_label.place(x=870, y=240)
        north_button = Radiobutton(window, text="North (3)", variable=direction, value=3)
        north_button.place(x=845, y=270)
        south_button = Radiobutton(window, text="South (2)", variable=direction, value=2)
        south_button.place(x=920, y=270)
        east_button = Radiobutton(window, text="East (4)", variable=direction, value=4)
        east_button.place(x=845, y=300)
        west_button = Radiobutton(window, text="West (1)", variable=direction, value=1)
        west_button.place(x=920, y=300)

        # Select an amplitude

        amplitude = IntVar()
        amplitude.set(1)
        current_amplitude_label = Label(window, text="current amplitude:", bg="yellow")
        current_amplitude_label.place(x=1065, y=240)
        run_expt_from_file_button = Radiobutton(window, text="0.5 A (1)", variable=amplitude, value=1)
        run_expt_from_file_button.place(x=1020, y=270)
        run_expt_from_input_button = Radiobutton(window, text="1 A (2)", variable=amplitude, value=2)
        run_expt_from_input_button.place(x=1090, y=270)
        amp_button_3 = Radiobutton(window, text="1.5 A (3)", variable=amplitude, value=3)
        amp_button_3.place(x=1160, y=270)
        amp_button_4 = Radiobutton(window, text="2 A (4)", variable=amplitude, value=4)
        amp_button_4.place(x=1020, y=300)
        amp_button_5 = Radiobutton(window, text="2.5 A (5)", variable=amplitude, value=5)
        amp_button_5.place(x=1090, y=300)

        force_duration = StringVar()
        force_duration.set(1)

        duration = Entry(window, textvariable=force_duration, width=10, validate="key", validatecommand=(regf, '%P'))
        duration.place(x=1075, y=330)
        mag_button = Button(window, text="Run magnet for following duration (in s):", command=start_magnet)
        mag_button.place(x=845, y=327)
        mag_pulse_button = Button(window, text="Pulse", command=pulse_magnet)
        mag_pulse_button.place(x=1190, y=327)

        # Temperature
        save_section_label = Label(window, text="Temperature control", bg="cyan")
        save_section_label.place(x=670, y=335)

        temperature_input = StringVar()
        temperature_input.set(20)

        temperature_entry = Entry(window, textvariable=temperature_input, width=10, validate="key",
                                  validatecommand=(regfn, '%P'))
        temperature_entry.place(x=720, y=360)

        temperature_entry.config(validate="key", validatecommand=(reg, '%P'))
        start_temperature_button = Button(window, text="Set temp", command=set_temp)
        start_temperature_button.place(x=630, y=357)

        stop_temperature_button = Button(window, text="Stop control", command=halt_temp)
        stop_temperature_button.place(x=630, y=385)

        temperature_readout = Label(window, text="Bath temp: n/a", bg='white')
        temperature_readout.place(x=720, y=385)

        # Volume sweep section

        vol_capture_label = Label(window, text="volume capture", bg="cyan")
        vol_capture_label.place(x=670, y=430)

        upper_z_input = StringVar()
        upper_z_input.set(6.3)

        lower_z_input = StringVar()
        lower_z_input.set(6.1)

        mirror_slope_input = StringVar()
        mirror_slope_input.set(-4486.982)

        filename_imput = StringVar()
        filename_imput.set('volume')

        save_volume_input = IntVar()
        save_volume_input.set(1)

        frame_period_input = IntVar()
        frame_period_input.set(25000)

        z_step_input = StringVar()
        z_step_input.set(0.02)

        # This value is shared with another section
        exposure_input = IntVar()
        exposure_input.set(10000)

        trigger_exposure_label = Label(window, text="Upper limit", bg="yellow")
        trigger_exposure_label.place(x=730, y=455)

        upper_z_field = Entry(window, textvariable=upper_z_input, width=10, validate="key",
                              validatecommand=(regfn, '%P'))
        upper_z_field.place(x=660, y=455)

        trigger_exposure_label = Label(window, text="Lower limit", bg="yellow")
        trigger_exposure_label.place(x=730, y=485)

        lower_z_field = Entry(window, textvariable=lower_z_input, width=10, validate="key",
                              validatecommand=(regfn, '%P'))
        lower_z_field.place(x=660, y=485)

        trigger_exposure_label = Label(window, text="Slope", bg="yellow")
        trigger_exposure_label.place(x=730, y=515)

        mirror_slope_field = Entry(window, textvariable=mirror_slope_input, width=10, validate="key",
                                   validatecommand=(regfn, '%P'))
        mirror_slope_field.place(x=660, y=515)

        trigger_exposure_label = Label(window, text="Trigger exposure time (us)", bg="yellow")
        trigger_exposure_label.place(x=730, y=545)

        exposure_field = Entry(window, textvariable=exposure_input, width=10, validate="key",
                               validatecommand=(regfn, '%P'))
        exposure_field.place(x=660, y=545)

        trigger_exposure_label = Label(window, text="frame period (us)", bg="yellow")
        trigger_exposure_label.place(x=730, y=575)

        frame_period_field = Entry(window, textvariable=frame_period_input, width=10, validate="key",
                                   validatecommand=(regfn, '%P'))
        frame_period_field.place(x=660, y=575)

        trigger_exposure_label = Label(window, text="Distance btw frames (mm)", bg="yellow")
        trigger_exposure_label.place(x=730, y=605)

        z_step_field = Entry(window, textvariable=z_step_input, width=10, validate="key", validatecommand=(regfn, '%P'))
        z_step_field.place(x=660, y=605)

        save_vol_button = Checkbutton(window, text="Save volume as", variable=save_volume_input, command=volumestate)
        save_vol_button.place(x=650, y=635)

        exposure_field = Entry(window, textvariable=filename_imput, width=10, validate="key",
                               validatecommand=(regfn, '%P'),
                               state='normal')
        exposure_field.place(x=760, y=635)

        sweep_vol_button = Button(window, text="Sweep volume", command=sweep)
        sweep_vol_button.place(x=660, y=660)

        sweeping_indicator = [Label(window, text='SWEEPING VOLUME', fg='red')]

        # Laser control

        blue_power_input = StringVar()
        blue_power_input.set(20)

        yellow_power_input = StringVar()
        yellow_power_input.set(20)

        blue_state_input = IntVar()
        blue_state_input.set(0)

        yellow_state_input = IntVar()
        yellow_state_input.set(0)

        label_l1 = Label(window, text="Laser control", bg="Cyan")
        label_l1.place(x=960, y=380)

        label_l2 = Label(window, text="488 Laser:", bg="yellow")
        label_l2.place(x=830, y=415)

        blue_on_radiobutton = Radiobutton(window, text="ON", variable=blue_state_input, value=1)
        blue_on_radiobutton.place(x=895, y=412)

        blue_off_radiobutton = Radiobutton(window, text="OFF", variable=blue_state_input, value=0)
        blue_off_radiobutton.place(x=940, y=412)

        label_l3 = Label(window, text="Power (in mW):")
        label_l3.place(x=1000, y=412)

        yellow_power_field = Entry(window, textvariable=yellow_power_input, width=10, validate="key",
                                   validatecommand=(reg, '%P'))
        yellow_power_field.place(x=1090, y=415)

        blue_update_button = Button(window, text="Update", command=laser)
        blue_update_button.place(x=1170, y=428)

        label_l3 = Label(window, text="561 Laser:", bg="yellow")
        label_l3.place(x=830, y=450)

        yellow_on_radiobutton = Radiobutton(window, text="ON", variable=yellow_state_input, value=1)
        yellow_on_radiobutton.place(x=895, y=447)

        yellow_off_radiobutton = Radiobutton(window, text="OFF", variable=yellow_state_input, value=0)
        yellow_off_radiobutton.place(x=940, y=447)

        label_l4 = Label(window, text="Power (in mW):")
        label_l4.place(x=1000, y=447)

        yellow_power_input = Entry(window, textvariable=yellow_power_input, width=10, validate="key",
                                   validatecommand=(reg, '%P'))
        yellow_power_input.place(x=1090, y=450)

        # Stage

        stage_control_block_label = Label(window, text='stage control', bg='Cyan')
        stage_control_block_label.place(x=960, y=490)

        stage_pos_input = StringVar()
        stage_pos_input.set(5.45)

        mirror_track_input = IntVar()
        mirror_track_input.set(0)

        stage_update_button = Button(window, text="Set stage to:", command=stage, state='disabled')
        stage_update_button.place(x=890, y=520)

        stage_position_input = Entry(window, textvariable=stage_pos_input, width=10, validate="key",
                                     validatecommand=(regfn, '%P'))
        stage_position_input.place(x=970, y=523)

        stage_engage_button = Button(window, text='Engage', command=engage)
        stage_engage_button.place(x=890, y=550)

        stage_disengage_button = Button(window, text='Disengage', command=disengage, state='disabled')
        stage_disengage_button.place(x=940, y=550)

        rename_stage_button = Button(window, text='Raise', command=raisestage)
        rename_stage_button.place(x=1020, y=550)

        stage_error_button = Button(window, text='Error:', command=stagerr)
        stage_error_button.place(x=1090, y=550)

        stage_error_label = Label(window, text='      ', bg='White')
        stage_error_label.place(x=1133, y=553)

        mirror_track_checkbox = Checkbutton(window, text='mirror tracking', variable=mirror_track_input, command=mirror)
        mirror_track_checkbox.place(x=1050, y=520)

        # Trigger
        trigger_block_label = Label(window, text='Trigger control', bg='cyan')
        trigger_block_label.place(x=960, y=600)
        trigger_mode_button = Button(window, text="Triggering mode", command=set_trigger)
        trigger_mode_button.place(x=930, y=630)
        single_trigger_button = Button(window, text="Trigger", command=trigger, state='disabled')
        single_trigger_button.place(x=880, y=630)
        software_trigger_button = Button(window, text="Software Trigger", command=set_continuous, state='disabled')
        software_trigger_button.place(x=1030, y=630)
        trigger_exposure_field = Entry(window, textvariable=exposure_input, width=10)
        trigger_exposure_field.place(x=940, y=660)
        trigger_exposure_label = Label(window, text="Trigger exposure time (us)", bg="yellow")
        trigger_exposure_label.place(x=1010, y=660)

        # Automated experiment part

        run_expt_from_file_button = Button(window, text="Run automated experiment from file", command=automated1)
        run_expt_from_file_button.place(x=40, y=590)

        run_expt_from_input_button = Button(window, text="Run automated experiment with parameters below",
                                            command=automated2)
        run_expt_from_input_button.place(x=300, y=590)

        automated_expt_label = Label(window, text="Automated Experiment", bg="Cyan")
        automated_expt_label.place(x=20, y=560)

        # Labels and buttons for the automated experiment
        expt_name_label = Label(window, text="Name", bg="yellow")
        expt_name_label.place(x=20, y=620)

        expt_amplitude_label = Label(window, text="Amplitude", bg="yellow")
        expt_amplitude_label.place(x=83, y=620)

        expt_config_label = Label(window, text="Configuration", bg="yellow")
        expt_config_label.place(x=150, y=620)

        expt_force_on_time_label = Label(window, text="Force on", bg="yellow")
        expt_force_on_time_label.place(x=240, y=620)

        expt_force_duration_label = Label(window, text="Force duration", bg="yellow")
        expt_force_duration_label.place(x=300, y=620)

        expt_num_frames_label = Label(window, text="Num frames", bg="yellow")
        expt_num_frames_label.place(x=383, y=620)

        expt_frame_period_label = Label(window, text="Frame period", bg="yellow")
        expt_frame_period_label.place(x=457, y=620)

        expt_temperature_label = Label(window, text="Temperature", bg="yellow")
        expt_temperature_label.place(x=533, y=620)

        # ----------------------

        frame_main = Frame(window, bg="gray")
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

        heading = [[Entry() for col in range(columns)] for row in range(rows)]
        value = [[StringVar() for col in range(columns)] for row in range(rows)]

        for row in range(0, rows):
            for col in range(0, columns):
                if first == 0:
                    value[row][col].set('')
                if row == 0:
                    heading[row][col] = Entry(frame_buttons, textvariable=value[row][col], width=12)
                else:
                    heading[row][col] = Entry(frame_buttons, textvariable=value[row][col], width=12, validate="key",
                                              validatecommand=(regf, '%P'))
                heading[row][col].grid(row=row, column=col, sticky='news')

        # Update buttons frames idle tasks to let tkinter calculate buttons sizes
        frame_buttons.update_idletasks()

        # Resize the canvas frame to show exactly 5-by-5 buttons and the scrollbar
        first5columns_width = sum([heading[0][col].winfo_width() for col in range(0, 8)])
        first5rows_height = sum([heading[row][0].winfo_height() for row in range(0, 4)])
        frame_canvas.config(width=first5columns_width + vsb.winfo_width(),
                            height=first5rows_height)

        # Set the canvas scrolling region
        canvas2.config(scrollregion=canvas2.bbox("all"))

        # ----------------------

        # Placing the first image in the canvas
        image_on_canvas = canvas.create_image(20, 20, anchor=NW, image=image6[0])

        image7_aux = np.zeros((np.shape(image5[0])[0], np.shape(image5[0])[1]), dtype=np.int8)

        def main_loop():

            while aux[-1] != 1:

                while stop[0] == 1:
                    pass

                # Acquiring the new image, changing contrast and brightness, and changing its format so that it's
                # compatible with tkinter
                # time.sleep(3)
                aux_thing1 = acquire_images(cam_list[0], nodemap_tldevice[0])
                aux_thing2 = acquire_images(cam_list[1], nodemap_tldevice[1])
                thing1 = np.clip(
                    aux[0] * cv2.resize(aux_thing1, None, fx=0.25, fy=0.25, interpolation=cv2.INTER_CUBIC) ** (aux[4]) +
                    aux[1], 0, 255)
                thing2 = np.clip(
                    aux2[0] * cv2.resize(aux_thing2, None, fx=0.25, fy=0.25, interpolation=cv2.INTER_CUBIC) ** (
                        aux2[4]) + aux2[1], 0, 255)

                # image8[0] = np.clip(aux[0] * acquire_images(cam_list[1], nodemap_tldevice[1]) ** (aux[4])+ aux[1], 0,
                # 255)

                if camera_display[0].get() == 1:
                    image7[:, :, 1] = np.copy(thing1)
                else:
                    image7[:, :, 1] = np.copy(image7_aux)

                if camera_display[1].get() == 1:
                    image7[:, :, 0] = np.copy(thing2)
                else:
                    image7[:, :, 0] = np.copy(image7_aux)

                plt.pause(delay[0])

                # image72 = cv2.resize(image_data, None, fx=0.25, fy=0.25, interpolation=cv2.INTER_CUBIC)

                image6_innerfunction_fr293 = [ImageTk.PhotoImage(Image.fromarray(image7, "RGB"), master=window)]
                # Changing the image on the canvas
                canvas.itemconfigure(image_on_canvas, image=image6_innerfunction_fr293[0])
                # update the temperature readout
                temperature_readout.configure(text="Bath temp: %s" % spim.tempm)
                if spim.ont:
                    temperature_readout.configure(bg='green')
                else:
                    temperature_readout.configure(bg='red')

        # function to close the window and leave the devices in the right state
        def exit_window():
            stop[0] = 0
            aux[-1] = 1
            set_continuous()
            plt.pause(0.2)
            window.destroy()

        window.protocol('WM_DELETE_WINDOW', exit_window)

        thread = threading.Thread(target=main_loop)
        thread.start()

        window.mainloop()
        aux[-1] = 1
        plt.pause(1)
        thread.join()

        # Terminating the camera

        for row, cam in enumerate(cam_list):
            terminate(cam)
            cam = None

        # Clear camera list before releasing system
        cam_list.Clear()

        # Release system instance
        system.ReleaseInstance()

        del system

        offlaser1()
        offlaser2()
        spim.engage()
        raisestage()
        spim.disengage()
        spim.halttempcont()
        spim.close_ports()

        if auto[0] == 1:
            sae.function()
            restart[0] = 0
        if auto[0] == 2:
            sae.function(2, params2[0])
            restart[0] = 0
        auto[0] = 0
        first = 1

    print('spim GUI successfully exited')

    return True


if __name__ == '__main__':
    main()
