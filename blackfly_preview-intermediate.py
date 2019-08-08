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


# register and list cameras
# ask user for input on the camera to use
# adjust camera settings for preview mode
# create a function to acquire an image
# create window showing preview image
# update window with new preview image every 0.1s
# if process is terminated, close down camera and finish acquisition

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


def user_selection(num_cameras):
    number = -1

    while number < 0 or number >= num_cameras:
        number = int(input("Please enter the number of the camera you wish to use:"))
    print ('You have selected camera number: %d' % number)

    return number


def camera_settings(cam):
    expo.configure_exposure(cam)

    return 1


def acquire_images(cam, nodemap_tldevice):
    global image_data
    try:

        # device_serial_number = ''
        node_device_serial_number = PySpin.CStringPtr(nodemap_tldevice.GetNode('DeviceSerialNumber'))
        if PySpin.IsAvailable(node_device_serial_number) and PySpin.IsReadable(node_device_serial_number):
            # device_serial_number = node_device_serial_number.GetValue()

            try:

                image_result = cam.GetNextImage()

                if image_result.IsIncomplete():
                    print ('Image incomplete with image status %d ...' % image_result.GetImageStatus())

                else:

                    # width = image_result.GetWidth()
                    # height = image_result.GetHeight()
                    # print 'Grabbed Image %d, width = %d, height = %d' % (i, width, height)

                    image_converted = image_result.Convert(PySpin.PixelFormat_Mono8, PySpin.HQ_LINEAR)
                    image_data = image_converted.GetNDArray()
                    image_result.Release()


            except PySpin.SpinnakerException as ex:
                print ('Error: %s' % ex)
                return False

    except PySpin.SpinnakerException as ex:
        print ('Error: %s' % ex)
        return False

    image_data2 = cv2.resize(image_data, None, fx=0.2, fy=0.2, interpolation=cv2.INTER_CUBIC)

    return image_data2


def create_window(cam, nodemap_tldevice, nodemap):
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
        image = acquire_images(cam, nodemap_tldevice)
        cv2.imshow('preview', image)
        cv2.waitKey(1)


    except PySpin.SpinnakerException as ex:
        print ('Error: %s' % ex)
        return False

    return 1


def update_window(cam, nodemap_tldevice):
    image = acquire_images(cam, nodemap_tldevice)
    cv2.imshow('preview', image)
    cv2.waitKey(1)


def terminate(cam):
    # plt.close()

    expo.reset_exposure(cam)
    cam.EndAcquisition()

    return True


def main():
    # Retrieve singleton reference to system object
    system = PySpin.System.GetInstance()

    # Get current library version
    version = system.GetLibraryVersion()
    print ('Library version: %d.%d.%d.%d' % (version.major, version.minor, version.type, version.build))

    # Retrieve list of cameras from the system
    cam_list = system.GetCameras()

    num_cameras = cam_list.GetSize()

    camera_registration(cam_list, num_cameras)

    num = user_selection(num_cameras)

    # Pick the camera from the list
    cam = cam_list[num]

    # Initialize camera
    cam.Init()

    nodemap_tldevice = cam.GetTLDeviceNodeMap()

    # Retrieve GenICam nodemap
    nodemap = cam.GetNodeMap()

    expo.configure_exposure(cam)

    create_window(cam, nodemap_tldevice, nodemap)

    def input_thread(L, cam):
        while True:
            aux = raw_input()
            print(L)
            if aux == 'e':
                break
            if aux == 's':
                camera_settings(cam)
        L.append('e')

    # def image2_thread(image1,image2):
    #  while True:
    #       image2[0] = image1[0].Convert(PySpin.PixelFormat_Mono8, PySpin.HQ_LINEAR)

    # def image3_thread(image2,image3):
    #    while True:
    #        image3[0] = image2[0].GetNDArray()

    L = [0]
    th.start_new_thread(input_thread, (L, cam,))

    # image1=[cam.GetNextImage()]
    # th.start_new_thread(image1_thread, (image1,cam,))

    # image2 = [image1[0].Convert(PySpin.PixelFormat_Mono8, PySpin.HQ_LINEAR)]
    # th.start_new_thread(image2_thread, (image1,image2,))

    # image3 = [image2[0].GetNDArray()]
    # th.start_new_thread(image3_thread, (image2,image3,))

    print (' ')
    print('Press e to exit or s to change settings')

    while L[-1] != 'e':
        update_window(cam, nodemap_tldevice)

    # while True:
    #   update_window(cam, nodemap_tldevice,time)

    terminate(cam)

    del cam

    # Clear camera list before releasing system
    cam_list.Clear()

    # Release system instance
    system.ReleaseInstance()

    raw_input('Done! Press Enter to exit...')

    return True


if __name__ == '__main__':
    main()
