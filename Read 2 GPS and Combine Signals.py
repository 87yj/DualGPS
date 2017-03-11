# -*- coding: utf-8 -*-
"""
Created on Fri Feb 24 19:38:17 2017

@author: Lenovo
"""

import numpy as np
import matplotlib

import matplotlib.pyplot as plt
#import matplotlib.cbook as cbook
#import matplotlib.dates as md
#from scipy.misc import imread
import time 
from datetime import datetime, date
import socket
import pynmea2
import sys
import utm
#import pyproj
import serial
import threading
#from queue import Queue
from queue import LifoQueue
import signal
import re
import select

matplotlib.use('TkAgg')

from numpy import arange, sin, pi
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2TkAgg
# implement the default mpl key bindings
from matplotlib.backend_bases import key_press_handler
from matplotlib.figure import Figure

import sys
if sys.version_info[0] < 3:
    import Tkinter as Tk
else:
    import tkinter as Tk

root = Tk.Tk()
root.wm_title("Embedding in TK")

Graphing = False
OutSocket = True


exitall = False
class ServiceExit(Exception):
    """
    Custom exception which is used to trigger the clean exit
    of all running threads and the main program.
    """
    pass


##NOT working....
def read_GPS_USB(ComChannel): 
    ser1 = serial.Serial(ComChannel, 115200, timeout=0)
    if ser1.in_waiting > 0:
            print(ser1.in_waiting)
            try:
                serdata = ser1.readline()
                ser1.close
                return(serdata)
                
            except ser1.SerialTimeoutException:
                print('Data could not be read')
                ser1.close
    #           i+=1
    else:
        print("no serial data found")
        ser1.close

def read_data(S, q, qrmc, gpsnum):
    print("in thread ", gpsnum)
    #kk=0
    #delta_time = -99.99
    #gps_time_current = 0
    #distancebtwn_measured = 0
    #shutdown_flag = threading.Event()
    stringdata1 = ""
    

    while not exitall:
        #print("looping", exitall)
        ready = select.select([S], [],[], 1)
        if ready[0]:
            try:
                stringdata1 = S.recv(BUFFER_SIZE)
                stringdata1 = stringdata1.decode('utf-8')
            except:
                print("Error Connecting to TCP for GPS", gpsnum)
                #GPS1_Valid = False
                continue
        
        for lines in stringdata1.splitlines():
            if GPSsentence1 in lines:
                try: 
                    validgga = checksum_nmea(lines)
                    #print(validgga)
                    if(validgga == True):
                        gga=pynmea2.parse(lines)  # parse the GPGGA string
                        q.put(gga)
                    #print("New gga", gpsnum, gga)
                except:
                    pass
            elif GPSsentence2 in lines:
                try: 
                    validrmc = checksum_nmea(lines)
                    if (validrmc==True):
                        rmc=pynmea2.parse(lines)  # parse the string
                    #print(validrmc)
                        qrmc.put_nowait(rmc)
                    #print("New rmc", gpsnum,  rmc)
                except:
                    pass
            else:
                pass
        if (q.qsize() > 1 ):
            #print("Items backing up in GGA queue", q.qsize())
            #time.sleep(0.5)
            with q.mutex:
                q.queue.clear()
        #if (qrmc.qsize() > 3 ):
            #print("Items backing up in RMC queue", qrmc.qsize())
            #print (qrmc)
            #time.sleep(0.5)
            #with qrmc.mutex:
                #qrmc.queue.clear()
        time.sleep(0.02)        
    print("Exiting", gpsnum)
    q.empty()
    
    
    
#print(height1)
def set_color(fix_type):
    if fix_type == 4:
        col='green'
    elif fix_type == 5:
        col = 'blue'
    else:
        col = 'red'
    return(col)
    
def checksum_nmea(sentence):
    cksum = sentence[len(sentence) -2:]
    chksumdata = re.sub("(\n|\r\n)", "", sentence[sentence.find("$")+1:sentence.find("*")])
    csum = 0
    for c in chksumdata:
        csum ^= ord(c)
    if hex(csum) == hex(int(cksum,16)):
        return(True)
    return(False)
    
        
def do_GPS_Calcs(GPS1, GPS2, GPS1_Valid, GPS2_Valid, qout):
    
    distbtwn_act = 1.31 #meters
    tilt = -999
    heading = 9999
    activeGPS="None"
    col1, col2 = 'white', 'white'
    if GPS1_Valid == True:
        utmdata1=utm.from_latlon(GPS1.latitude, GPS1.longitude)
        height1 = GPS1.altitude
        tGPS1 = (datetime.combine(date.today(), GPS1.timestamp))
        col1 = set_color(GPS1.gps_qual)
        if(Graphing == True): ax.scatter(utmdata1[0],utmdata1[1], s=80, facecolor = 'white', edgecolor = col1, marker = "D", label = 'GPS 1')
        activeGPS = '1'
    if GPS2_Valid == True:
        utmdata2=utm.from_latlon(GPS2.latitude, GPS2.longitude)
        height2 = GPS2.altitude
        tGPS2 = (datetime.combine(date.today(), GPS2.timestamp))
        col2 = set_color(GPS2.gps_qual)
        if(Graphing == True): ax.scatter(utmdata2[0],utmdata2[1], s=80, facecolor = 'yellow', edgecolor = col2, marker = "o", label = 'GPS 2')
        activeGPS = '2'
    #else:
    #    col2 = '0.50'
    #print(col1,col2)    
   
    if (GPS1_Valid == True and GPS2_Valid == True):
        activeGPS = "Both"
        delta_time = (tGPS2-tGPS1).total_seconds()
        if (delta_time !=0):
            print("Mismatch! ", delta_time, "  GPS1", GPS1.timestamp, "  GPS2", GPS2.timestamp)
            col = "red"
        else:
            col = "green"
        distancebtwn_measured = np.sqrt((utmdata2[0]-utmdata1[0])**2+(utmdata2[1]-utmdata1[1])**2)
        CenterPos = [(utmdata1[0]+utmdata2[0])/2, (utmdata1[1]+utmdata2[1])/2]
        
        heading21 = np.math.degrees(np.math.atan2((utmdata2[0]-utmdata1[0]),(utmdata2[1]-utmdata1[1])))
        if (heading21 >= 0 and heading21 <90):
            heading = 360 - heading21
        elif (heading21 >=90 and heading21<=180):
            heading = heading21-90
        elif (heading21 >= -180 and heading21<0):
            heading = 270 + heading21
        else:
            print("Heading error")
        #print("Heading21 :", heading21, "  VehicleHeading: ", heading)
        #print("UTM_Delta_Easting: " , (utmdata2[0]-utmdata1[0]),"UTM_Delta_Northing: " , (utmdata2[1]-utmdata1[1]))
        #print("Heading:  ", heading)
        gps1_time_current = datetime.combine(date.today(), GPS1.timestamp)
        heightdiff = height2-height1
        if(Graphing == True): ax.scatter((utmdata1[0]+utmdata2[0])/2, (utmdata1[1]+utmdata2[1])/2, color = 'black', marker = 's', label = 'MidPt')
        if(Graphing == True):ax2.plot(gps1_time_current, heading, color = col, marker ="s")  
        CenterLatLon = utm.to_latlon(CenterPos[0], CenterPos[1], utmdata1[2], utmdata1[3])
        #print("here", heightdiff, distbtwn_act)
        if(abs(heightdiff)<distbtwn_act):
            #print("in tilt calc")
            tilt = np.math.degrees(np.math.asin(heightdiff/distbtwn_act))
            if(Graphing == True):ax3.set_visible(True)  
            col = "green"
        else:
            tilt= 0
            if(Graphing == True):ax3.text(0,gps1_time_current, "Invalid Height Data for Tilt")
            #ax3.set_visible(False)
            col = "red"
    NMEA_string = create_NMEA_out(GPS1, GPS2, tilt, heading, activeGPS, qout)
    #s3.send(NMEA_string)
    
    
def correct_for_tilt (GPS1, GPS2, tilt):
    #this function takes the absolute data and corrects for tilt as measured by the 2 GPS's
    tilterror = np.math.tan(np.math.radians(tilt))*GPS_mouting_height
    #need to break this down into X and Y UTM components, based on the heading presumably.
        
def create_NMEA_out(GPS1,GPS2, tilt, heading, activeGPS, qout):
    NMEAoutGGA = ""
    qout.put(activeGPS)
    
    if (activeGPS == 'Both'):
        utmdata1=utm.from_latlon(GPS1.latitude, GPS1.longitude)
        utmdata2=utm.from_latlon(GPS2.latitude, GPS2.longitude)
        CenterPos = [(utmdata1[0]+utmdata2[0])/2, (utmdata1[1]+utmdata2[1])/2]
        
        CenterLatLon = (utm.to_latlon(CenterPos[0], CenterPos[1], utmdata1[2], utmdata1[3]))
        CenterLatdeg = int(CenterLatLon[0])
        CenterLatmin = int((CenterLatLon[0]-CenterLatdeg)*60)
        CenterLatsec = ((CenterLatLon[0]-CenterLatdeg)*60)-CenterLatmin
        CenterLatsec = round(CenterLatsec,7)
        LatStrng = "%02d" % (CenterLatdeg)+"%02d" %(CenterLatmin)+ (str(CenterLatsec).replace('0', '', 1))
        CenterLon = abs(CenterLatLon[1])
        CenterLondeg = (int(CenterLon))
        CenterLonmin = (int((CenterLon-CenterLondeg)*60))
        CenterLonsec = (((CenterLon-CenterLondeg)*60)-CenterLonmin)
        CenterLonsec = round(CenterLonsec,7)
        LonStrng = "%03d" % (CenterLondeg)+"%02d" %(CenterLonmin)+ str(CenterLonsec).replace('0', '', 1)
        time_out = "%02d" % (GPS1.timestamp.hour) + "%02d" % (GPS1.timestamp.minute)+"%02d" % (GPS1.timestamp.second)+"."+ "%02d" % ((GPS1.timestamp.microsecond)/10000)
        latdir_out = GPS1.lat_dir
        londir_out = GPS1.lon_dir
        gpsqual_out = str((min(int(GPS1.gps_qual), int(GPS2.gps_qual))))
        numsats_out = "%02d" % ((min(int(GPS1.num_sats), int(GPS2.num_sats))))
        HDOP_out = str(max(float(GPS1.horizontal_dil), float(GPS1.horizontal_dil)))
        alt_out = str(round(((float(GPS1.altitude) + float(GPS2.altitude)))/2,3))
        altunits_out = 'M'
        geosep_out = str(round(((float(GPS1.geo_sep) + float(GPS2.geo_sep))/2),3))
        geosepunits_out = 'M'
        age_out = str(((float(GPS1.age_gps_data) + float(GPS2.age_gps_data))/2))
        refid_out = GPS1.ref_station_id
        NMEAoutGGA = pynmea2.GGA('GP', 'GGA', (time_out, LatStrng, latdir_out, LonStrng, londir_out, gpsqual_out, numsats_out, HDOP_out, alt_out, altunits_out, geosep_out, geosepunits_out, age_out, refid_out))
        NMEAoutRMC = pynmea2.RMC('GP', 'RMC', (time_out, 'A', LatStrng, latdir_out, LonStrng, londir_out, "12.0", "260.2", "070317", "1.2"))  #eedoverground, trackangle, dateDDMMYY,  MagVar))
        #print("Both GPS's active", NMEAoutGGA)
    elif (activeGPS == '1'):
        #print ("GPS1 lon dir", GPS1.lon_dir)
        time_out = "%02d" % (GPS1.timestamp.hour) + "%02d" % (GPS1.timestamp.minute)+"%02d" % (GPS1.timestamp.second)+"."+ "%02d" % ((GPS1.timestamp.microsecond)/10000)
        NMEAoutGGA = pynmea2.GGA('GP', 'GGA', (time_out, GPS1.lat, GPS1.lat_dir, GPS1.lon, GPS1.lon_dir, str(GPS1.gps_qual), GPS1.num_sats, GPS1.horizontal_dil, str(GPS1.altitude), GPS1.altitude_units, GPS1.geo_sep, GPS1.geo_sep_units, GPS1.age_gps_data, GPS1.ref_station_id))
        #NMEAoutRMC = ''
        tilt = 0
        heading = 0
    elif (activeGPS == '2'):
        time_out = "%02d" % (GPS2.timestamp.hour) + "%02d" % (GPS2.timestamp.minute)+"%02d" % (GPS2.timestamp.second)+"."+ "%02d" % ((GPS2.timestamp.microsecond)/10000)
        #print("GPS2 lat", GPS2.lat)
        NMEAoutGGA = pynmea2.GGA('GP', 'GGA', (time_out, GPS2.lat, GPS2.lat_dir, GPS2.lon, GPS2.lon_dir, str(GPS2.gps_qual), GPS2.num_sats, GPS2.horizontal_dil, str(GPS2.altitude), GPS2.altitude_units, GPS2.geo_sep, GPS2.geo_sep_units, GPS2.age_gps_data, GPS2.ref_station_id))
        #NMEAoutGGA = pynmea2.GGA('GP', 'GGA', (time_out, GPS2.lat, GPS2.lat_dir, GPS2.lon, GPS2.lon_dir, GPS2.gps_qual, GPS2.num_sats, GPS2.horizontal_dil, GPS2.altitude, GPS2.altitude_units, GPS2.geo_sep, GPS2.geo_sep_units, GPS2.age_gps_data, GPS2.ref_station_id))
        #NMEAoutRMC=''
        tilt = 0
        heading = 0
##Here NEEDS TESTED OUT
    #NMEAoutRMC = pynmea2.RMC('GP','RMC', (time_out, 'A', LatStrng, latdir_out, LonStrng, londir_out, "0", "0", "date", str(heading))
    #print(NMEAoutGGA)
    #print("tilt: ", tilt, "  " , heading)
    #print("ActiveGPS: ", activeGPS, " outputs: ", time_out, GPS2.lat, GPS2.lat_dir, GPS2.lon, GPS2.lon_dir, GPS2.gps_qual, GPS2.num_sats, GPS2.horizontal_dil, GPS2.altitude, GPS2.altitude_units, GPS2.geo_sep, GPS2.geo_sep_units, GPS2.age_gps_data, GPS2.ref_station_id)
    if(Graphing == True): plt3text3.set_text(('Tilt:', round(tilt,1), '  Heading:', round(heading,1)))
    ####SOMETHING IS REALLY BROKEN HERE AND ERRORS OUT######
    #print("NMEAGGA: ", NMEAoutGGA)
    qout.put_nowait(NMEAoutGGA)
    
    #print("Qout     ", NMEAoutGGA)
    #print("RMC: ", NMEAoutRMC)
    
    #return(NMEAoutGGA)
def output_NMEA_String(s3, qout):
    HOST = '192.168.11.19'
    PORT = 9999
#    try:
#        s3 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#        print("output UDP socket created")
#    except:
#        print("Failed to create socket")
#    print("S created")
    NMEAdata = "string"
    bNMEAdata = NMEAdata.encode('utf-8')
    
    #s.sendall('Hello, world')
    #s3.close()
    print("Output thread started...............................")
    while not exitall:
        #print("output thead looping")
        try:
            NMEAdata = qout.get()
            NMEAdata = str(NMEAdata)
            
            bNMEAdata = NMEAdata.encode('utf-8')
            print("NMEA Data out: ", NMEAdata)
            #bNMEAdata = NMEAdata.encode('utf-8')
            #s3.sendto(bNMEAdata,(HOST, PORT))
            if(OutSocket == True): s3.send(bNMEAdata)
        except socket.error:
            print ("Error on output socket")
            #sys.exit()
            time.sleep(0.05)
            #print("Error retreiving GPS data")
        
        #data = client_socket.recv(512)
        #if ( data == 'q' or data == 'Q'):
            #client_socket.close()
            #break;
        #else:
            #print "RECIEVED:" , data
            #data = raw_input ( "SEND( TYPE q or Q to Quit):" )
            #if (data <> 'Q' and data <> 'q'):
            #client_socket.send(data)
        
    
        
    if(OutSocket == True): s3.close()
    print("Exiting output string thread")
    qout.empty()
    
        
def _quit():
    root.quit()     # stops mainloop
    root.destroy()  # this is necessary on Windows to prevent
                    # Fatal Python Error: PyEval_RestoreThread: NULL tstate
                    
def service_shutdown(signum, frame):
    print('Caught signal %d' % signum)
    raise ServiceExit
            
    #ax3.plot(gps1_time_current, tilt, color = col, marker = "d")
       
        
        #print(CenterLatLon)
        #print("no bug")
        #if (heightdiff < 10 and heightdiff > -10):
            
            #msg = pynmea2.GGA('GP', 'GGA', ('184353.07', '1929.045', 'S', '02410.506', 'E', '1', '04', '2.6', '100.00', 'M', '-33.9', 'M', '', '0000'))

if __name__ == "__main__":
    TCP_IP1 = '192.168.11.10'  #Reach 1
    TCP_IP2 = '192.168.11.20' #reach 2
    TCP_IP3 = '192.168.11.19' #Localhost
    TCP_PORT = 5101
    OUT_PORT = 9999
    BUFFER_SIZE = 512
    ##Need program to be able to detect if socket is good and still work if only one of the 2 is available
    ##currently hangs is only one available
    s1=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s2=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    if(OutSocket == True): 
        s3=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    else:
        s3 = "Null"
        
    s1.close
    s2.close
    if(OutSocket == True): s3.close
    try:
        s1.connect((TCP_IP1, TCP_PORT))
    except:
        print("Unable to connect to TPC Ports for GPS1 data")
    try:
        s2.connect((TCP_IP2, TCP_PORT))
    except:
        print("Unable to connect to TPC Ports for GPS2 data")
    #s2.setblocking(0)
    if(OutSocket == True): 
        try:
            s3.connect((TCP_IP3, OUT_PORT))
        except:
            print("Unable to onnecte to TCP port for output data")    
    #try:
    #    s3.connect(('localhost', 7777))
    #except:
    #    print("Unable to connect to port 7777")
    #    time.sleep(5)
    utmdata1=[]
    utmdata2=[]
    GPSsentence1 = "$GPGGA"
    GPSsentence2 = "$GPRMC"
    ComChannel1 = "COM3"
    ComChannel2 = "COM9"
    delta_time = 0
    distbtwn_act = 1.31 #distance between antennas in feet
    print("Setup complete")


    plt.close('all')
    
    
    
    #fig, ax = plt.subplots(nrows=1, ncols=1)
    #fig.set_figwidth(10)
    #print(fig.get_figwidth)
    
    #plt.tight_layout()
    if(Graphing == True):
        fig = plt.figure(1, figsize = (8,8))
        ax = fig.add_subplot(311)
        ax.set_ylabel("UTM Northing")


        ax2 = fig.add_subplot(312)
        ax2.set_ylabel('Heading of Vehicle')
        ax3 = fig.add_subplot(313)
        ax3.set_ylabel("Tilt Angle")
    
        plt.show
    signal.signal(signal.SIGTERM, service_shutdown)
    signal.signal(signal.SIGINT, service_shutdown)
    initialstring = '$GPGGA,172814.0,3723.46587704,N,12202.26957864,W,2,6,1.2,18.893,M,-25.669,M,2.0,0031*4F'
    GPS1_Valid = False
    GPS2_Valid = False
    GPS1 = pynmea2.parse(initialstring)
    GPS2 = pynmea2.parse(initialstring)
    last_timestamp = GPS1.timestamp

   

    i=0
    loops =50
    #q1 = Queue.LifoQueue(maxsize=0)
    q1 = LifoQueue(maxsize = 0)
    q1rmc = LifoQueue(maxsize=0)
    q2 = LifoQueue(maxsize=0)
    q2rmc = LifoQueue(maxsize=0)
    qout = LifoQueue(maxsize = 0)
    #q1rmc="nothing"
    #q2rmc="nothing"
    print("Starting GPS reader threads")
    try:
        thread1 = threading.Thread(target=read_data, args=(s1, q1, q1rmc, 1))
        thread2 = threading.Thread(target=read_data, args=(s2, q2, q2rmc, 2))
        thread3 = threading.Thread(target = output_NMEA_String, args=(s3,  qout))
        thread1.deamon = True
        thread2.daemon = True
        thread3.daemon = True
        thread1.start()
        thread2.start()
        thread3.start()
        time.sleep(.5)
        print("THREADS STARTED!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        
    except ServiceExit:
        # Terminate the running threads.
        # Set the shutdown flag on each thread to trigger a clean shutdown of each thread.
        #thread1.shutdown_flag.set()
        #thread2.shutdown_flag.set()
        # Wait for the threads to close...
        #thread1.join()
        #thread2.join()
        print("Issue startging Threads")
        #system.exit()
    #threading.enumerate()
    #thread2.daemon = True
    
    lgnd = False
    

    while i<loops:  
        
        if i == loops-1:
            if(Graphing == True):plt3text2.set_text("Time to shut down")
            exitall = True    
        i+=1
        #print(i, "IN WHILE LOOP")
        if i < 2:
            print("i<2")
            try:
                GPS1=q1.get(True, .1)
                print(GPS1)
                GPS1_Update = True
                print(i, "GPS1: ", GPS1)
                last_timestamp = GPS1.timestamp
                GPS_start = (datetime.combine(date.today(),GPS1.timestamp))
            except:
                GPS1_Update = False
                pass
            try:
                GPS2=q2.get(True, .11)
                GPS2_Update = True
                print(i, "GPS2: ", GPS2)
                last_timestamp = GPS2.timestamp
                GPS_start = (datetime.combine(date.today(),GPS2.timestamp))
            except:
                GPS2_Update = False
                pass
            if(Graphing == True):
                plt3text1 = ax3.text(.99, 0.1, ("Initial Text"),
                            verticalalignment='center', horizontalalignment='right',
                            transform=ax3.transAxes,
                            color='green', fontsize=10)
                plt3text2 = ax3.text(.99, 0.2, ("Initial Text2"),
                            verticalalignment='center', horizontalalignment='right',
                            transform=ax3.transAxes,
                            color='blue', fontsize=10)
                plt3text3 = ax3.text(.99, 0.3, ("Initial Text3"),
                            verticalalignment='center', horizontalalignment='right',
                            transform=ax3.transAxes,
                            color='blue', fontsize=10)
        
                                         
        starttime = time.time()
        #print("start time: " , starttime)
       
        if(Graphing == True):plt.pause(0.010)
        elasped = time.time()-starttime
        #GPS_elapsed = (((datetime.combine(date.today(),GPS1.timestamp)) - GPS_start).total_seconds())
        #print("elapsed clock:" , elasped, "GPS elasped : ",GPS_elapsed)
        #try:
            #print("in try")
        #if q1.empty() != True:
        try:
            GPS1 = q1.get(.02)
            GPS1_Update = True
            #print("GPS1..a....",GPS1)
        except:
            GPS1_Update = False
            #print("No GPS Update GPS1")
        try:
            GPS1rmc = q1rmc.get_nowait()
            GPS1rmcnew = True
            #print("rmc get try GPS1", GPS1rmc)
        except:
            GPS1rmcnew = False
        
        try:
            GPS2 = q2.get(.02)
            GPS2_Update = True
            #time.sleep(0.01)
            #print("//////GPS2......",GPS2)
        except:
            GPS2_Update = False
        try:
            GPS2rmc = q2rmc.get_nowait()
            GPS2rmcnew = True
        except:
            Gps2rmcnew = False
            
        ##ISSUE....DOES NOT GUARANTEE THAT THE TIMESTAMPS ARE MATCHED UP.
        ## CAN HAVE 2 VALID SIGNALS, BUT AT SPEED OF LOOP, IT WILL SOMETIMES MISS
        ##NEED TO FIGURE OUT HOW TO READ AND COMPARE VALIDITY AND TIME STAMPS EACH TIME
        if(Graphing == True):plt3text2.set_text("starting ifs")  
        #time.sleep(.1)
        #plt3text3.set_text((GPS1.timestamp, GPS2.timestamp))
        if(GPS1.timestamp > GPS2.timestamp and GPS2_Update == True):
            #GPS2 = q2.get_nowait()
            #GPS2rmc = q2rmc.get_nowait()
            if(Graphing == True):
                plt3text1.set_text("GPS time 1 > GPS time 2")
                plt3text2.set_text((GPS1.timestamp, "1 waited  ",  GPS2.timestamp))
        elif (GPS1.timestamp < GPS2.timestamp and GPS1_Update == True):
            if(Graphing == True):
                ax2.set_ylabel(("2nd if", GPS1.timestamp, GPS2.timestamp))
                #GPS1 = q1.get_nowait()
                #GPS1rmc = q1rmc.get_nowait()
                plt3text1.set_text("GPS time 2 > GPS time 1")
                plt3text2.set_text((GPS1.timestamp, "2 waited  ",  GPS2.timestamp))
        #elif (GPS1.timestamp== GPS2.timestamp and GPS1.timestamp > last_timestamp):
        #    plt3text1.set_text(("executing update statement", i))
        #    plt3text2.set_text((GPS2))
        #    #plt3text1.set_text((GPS1))
        #    do_GPS_Calcs(GPS1, GPS2, GPS1_Valid, GPS2_Valid)
        #    last_timestamp = GPS1.timestamp
        else:
            if(Graphing == True):
                plt3text2.set_text(("else", GPS1.timestamp, GPS2.timestamp, last_timestamp))
            #print("Dropped to else")
        time.sleep(.10)
            #continue
        # need to add way of checking timestamp and only triggering if the valid GPS is new.
        if (GPS1_Update == True and GPS2_Update == True):
            do_GPS_Calcs(GPS1, GPS2, GPS1_Update, GPS2_Update, qout)
            #print(GPS1_Update, GPS2_Update)
            
        elif(GPS1_Update == True and GPS1.timestamp > last_timestamp):
                last_timestamp = GPS1.timestamp
                do_GPS_Calcs(GPS1, GPS2, GPS1_Update, GPS2_Update, qout)
        elif(GPS2_Update == True and GPS2.timestamp > last_timestamp):
            do_GPS_Calcs(GPS1, GPS2, GPS1_Update, GPS2_Update, qout)    
            last_timestamp = GPS2.timestamp
        if(Graphing == True):
            if lgnd == False:
                lgnd=True
                #ax.legend()
                ax.legend(loc='upper center', bbox_to_anchor=(0.5, 1.05), ncol=3)
            
                                
                                
        #timestamp = GPS1[7:16]
        #print(timestamp)
                                
    if(Graphing == True):plt3text2.set_text("trying to close")
    time.sleep(1)
#    while (q1.empty != True):
#        try:
#            q1.get_nowait()
#            print("Emptying 1") 
#        except:
#            thread1._stop()
    thread1.join(3)
    thread2.join(3)
    thread3.join(3)
    #plt.close('all')
    print("exited 1:")
    #thread2._stop()
    print("exited 2:")

print("GoodBye")

    

    #s1.close
    #s2.close
            
                                        
                                        
 
    


#print(stringdata2)


#
#PASHR - RT300 proprietary roll and pitch sentence
#         1           2   3    4      5      6     7     8     9  10 11 12
#         |           |   |    |      |      |     |     |     |   | |  |
#$PASHR,hhmmss.sss,hhh.hh,T,rrr.rr,ppp.pp,xxx.xx,a.aaa,b.bbb,c.ccc,d,e*hh<CR><LF>
#Field number:
#
#hhmmss.sss - UTC time
#
#hhh.hh - Heading in degrees
#
#T - flag to indicate that the Heading is True Heading (i.e. to True North)
#
#rrr.rr - Roll Angle in degrees
#
#ppp.pp - Pitch Angle in degrees
#
#xxx.xx - Heave
#
#a.aaa - Roll Angle Accuracy Estimate (Stdev) in degrees
#
#b.bbb - Pitch Angle Accuracy Estimate (Stdev) in degrees
#
#c.ccc - Heading Angle Accuracy Estimate (Stdev) in degrees
#
#d - Aiding Status
#
#e - IMU Status
#
#hh - Checksum
#
#[PASHR] describes this sentence as NMEA, though other sources say it is Ashtech proprietary and describe a different format.
#
#Example:
#
#$PASHR,085335.000,224.19,T,-01.26,+00.83,+00.00,0.101,0.113,0.267,1,0*06
