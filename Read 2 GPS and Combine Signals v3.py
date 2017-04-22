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
from queue import Queue
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
latesttimestamp = 0
global delta_time_err
delta_time_err = 0
UpdateRate = 5  #in Hertz - samples per second
UpdateInterval = 1/UpdateRate #time bewteen expected updates in seconds



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

def read_data(S, q, qvtg, gpsnum):
    print("in thread ", gpsnum)
    stringdata1 = ""
    #filename = "Sample1input"+str(gpsnum)+".txt"
    #print(filename)
    #text_file = open(filename, 'w')
    ###Use this and determine if the string is at the timestamp expected
    while not exitall:
        #print("looping", exitall)
        ready = select.select([S], [],[], 1)
        #look into using selectorS instead
        if ready[0]:
            try:
                stringdata1 = S.recv(BUFFER_SIZE)
                #print(stringdata1, file=text_file)
                #print(stringdata1)
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
                    print("Invalid gga.  GPS # : ", gpsnum)
                    pass
            elif GPSsentence2 in lines:
                try: 
                    validvtg = checksum_nmea(lines)
                    if (validvtg==True):
                        vtg=pynmea2.parse(lines)  # parse the string
                    #print(validvtg)
                        qvtg.put_nowait(vtg)
                except:
                    pass
            else:
                pass
        if (q.qsize() > 1 ):
            #print("Items backing up in GGA queue", gpsnum, " ", q.qsize())
            #time.sleep(0.5)
            with q.mutex:
                q.queue.clear()
            print("***************MUX QUEUE*******************")
        #if (qvtg.qsize() > 3 ):
            #print("Items backing up in vtg queue", qvtg.qsize())
            #print (qvtg)
            #time.sleep(0.5)
            #with qvtg.mutex:
                #qvtg.queue.clear()
        time.sleep(0.005)        
    #text_file.close()
    q.empty()
    print("Exiting", gpsnum)
    
    
    
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
    global delta_time_err
    printdebug = True
    distbtwn_act = 1.31 #meters
    tilt = -999
    heading = 9999
    activeGPS="None"
    col1, col2 = 'white', 'white'
    
    ### Add in capability to take the data from one GPS only in RTK and move position to centerline to keep accurate position
    
    if GPS1_Valid == True:
        utmdata1=utm.from_latlon(GPS1.latitude, GPS1.longitude)
        height1 = GPS1.altitude
        tGPS1 = (datetime.combine(date.today(), GPS1.timestamp))
        col1 = set_color(GPS1.gps_qual)
        if(Graphing == True): ax.scatter(utmdata1[0],utmdata1[1], s=80, facecolor = 'white', edgecolor = col1, marker = "D", label = 'GPS 1')
        activeGPS = '1'
        #print("tGPS1", tGPS1)
    if GPS2_Valid == True:
        utmdata2=utm.from_latlon(GPS2.latitude, GPS2.longitude)
        height2 = GPS2.altitude
        tGPS2 = (datetime.combine(date.today(), GPS2.timestamp))
        #print("tGPS2", tGPS2)
        col2 = set_color(GPS2.gps_qual)
        if(Graphing == True): ax.scatter(utmdata2[0],utmdata2[1], s=80, facecolor = 'yellow', edgecolor = col2, marker = "o", label = 'GPS 2')
        activeGPS = '2'
    #else:
    #    col2 = '0.50'
    #print(col1,col2)    
   
    if (GPS1_Valid == True and GPS2_Valid == True):
        activeGPS = "Both"
        #print("GPS1", GPS1.timestamp, "  GPS2", GPS2.timestamp, "  Delta: ", "delta_time", end="")
        delta_time = (tGPS2-tGPS1).total_seconds()
        if (delta_time !=0):
            #print("Mismatch! ", delta_time, "  GPS1", GPS1.timestamp, "  GPS2", GPS2.timestamp)
            delta_time_err += 1
            col = "red"
        else:
            col = "green"
        if(printdebug == True):
            print("T1:",GPS1.timestamp.strftime('%H:%M:%S.%f')[:-4], "T2:", GPS2.timestamp.strftime('%H:%M:%S.%f')[:-4],"dt:", '% 0.1f' % delta_time, "  dt_err:", '%2d' % delta_time_err, " ", end="")
        
        distancebtwn_measured = np.sqrt((utmdata2[0]-utmdata1[0])**2+(utmdata2[1]-utmdata1[1])**2)
        CenterPos = [(utmdata1[0]+utmdata2[0])/2, (utmdata1[1]+utmdata2[1])/2]
        
        heading21 = np.math.degrees(np.math.atan2((utmdata2[1]-utmdata1[1]), (utmdata2[0]-utmdata1[0])))
        #print(("N2", utmdata2[1], "N1", utmdata1[1], "E2", utmdata2[0], "E1", utmdata1[0]))
        #print("heading 21:" , round(heading21,1), end="")
        if(printdebug == True):
            print("h21:", '% 3.1f' % round(heading21,1)," ", end="")
        ##Math needas further vetting here to make sure this is computing right.
        if (heading21 >= 0 and heading21 <=90):
            heading = 90 - heading21  
        elif (heading21 >90 and heading21<=180):
            heading = 450 - heading21
        elif (heading21 <0 and heading21 >= -90):
            heading = -1*heading21
        elif (heading21 >= -180 and heading21 < -90):
            heading = 90+(-1*heading21)
        else:
            print("Heading error")
        #print("Heading21 :", heading21, "  VehicleHeading: ", heading)
        #print("UTM_Delta_Easting: " , (utmdata2[0]-utmdata1[0]),"UTM_Delta_Northing: " , (utmdata2[1]-utmdata1[1]))
        #print("Heading:  ", heading)
        gps1_time_current = datetime.combine(date.today(), GPS1.timestamp)
        heightdiff = height2-height1
        if(Graphing == True):
            ax.scatter((utmdata1[0]+utmdata2[0])/2, (utmdata1[1]+utmdata2[1])/2, color = 'black', marker = 's', label = 'MidPt')
            ax2.plot(gps1_time_current, heading, color = col, marker ="s")  
            ax2.plot(gps1_time_current, heading21, color = 'black', marker ="o")
        CenterLatLon = utm.to_latlon(CenterPos[0], CenterPos[1], utmdata1[2], utmdata1[3])
        #print("here", heightdiff, distbtwn_act)
        if(abs(heightdiff)<distbtwn_act):
            #print("in tilt calc")
            tilt = np.math.degrees(np.math.asin(heightdiff/distbtwn_act))
            if(Graphing == True):ax3.set_visible(True)  
            col = "green"
            CorrectedCenterPos = correct_for_tilt(CenterPos, heading, tilt)
            CorrectedLatLon = utm.to_latlon(CorrectedCenterPos[0], CorrectedCenterPos[1], utmdata1[2], utmdata1[3])
        else:
            tilt= 0
            if(Graphing == True):ax3.text(0,gps1_time_current, "Invalid Height Data for Tilt")
            #ax3.set_visible(False)
            col = "red"
            
    
    NMEA_string = create_NMEA_out(GPS1, GPS2, tilt, heading, activeGPS, qout)
    #s3.send(NMEA_string)
    
    
def correct_for_tilt (CenterPos, heading, tilt):
    printdebug=True
    #this function takes the absolute data and corrects for tilt as measured by the 2 GPS's
    GPS_mounting_height = 2.4 #METERS
    easting_meas = CenterPos[0]
    northing_meas = CenterPos[1]
    tilterr = (np.math.tan(np.math.radians(tilt))*GPS_mounting_height)
    easting_act = easting_meas+tilterr*np.math.cos(np.math.radians(heading))
    northing_act= northing_meas-tilterr*np.math.sin(np.math.radians(heading))
    #print("Heading:",  round(heading,2), "tilt", round(tilt,2), "tilterr", round(tilterr,3))
    if(printdebug == True):
        print("H:", '%-3.1f' % round(heading,2),"tilt:", '%-02.1f' % round(tilt,2), "tiltCor:", '%-2.3f' % round(tilterr,3))
    #print("EM", round(easting_meas,3), "EA", round(easting_act,3), "NM:",round(northing_meas,3), "NA:",round(northing_act, 3))
    #print("EM-EA: ", easting_meas - easting_act, "  NM-NA: ", northing_meas-northing_act)
    return(easting_act, northing_act)

        
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
        
        #NMEAoutVTG = pynmea2.VTG('GP', 'VTG', (time_out, 'A', LatStrng, latdir_out, LonStrng, londir_out, "12.0", "260.2", "070317", "1.2"))  #eedoverground, trackangle, dateDDMMYY,  MagVar))
        #print("Both GPS's active", NMEAoutGGA)
    elif (activeGPS == '1'):
        #print ("GPS1 lon dir", GPS1.lon_dir)
 ###    Need to update this to move the GPS location to the approximated center position of the vehicle (heading + dist between w/ sin/cos)
        time_out = "%02d" % (GPS1.timestamp.hour) + "%02d" % (GPS1.timestamp.minute)+"%02d" % (GPS1.timestamp.second)+"."+ "%02d" % ((GPS1.timestamp.microsecond)/10000)
        NMEAoutGGA = pynmea2.GGA('GP', 'GGA', (time_out, GPS1.lat, GPS1.lat_dir, GPS1.lon, GPS1.lon_dir, str(GPS1.gps_qual), GPS1.num_sats, GPS1.horizontal_dil, str(GPS1.altitude), GPS1.altitude_units, GPS1.geo_sep, GPS1.geo_sep_units, GPS1.age_gps_data, GPS1.ref_station_id))
        #NMEAoutRMC = ''
        tilt = 0
        ##Modify to add in tilt from the IMU if available
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
            #print("NMEA Data out: ", NMEAdata)
            #bNMEAdata = NMEAdata.encode('utf-8')
            #s3.sendto(bNMEAdata,(HOST, PORT))
            if(OutSocket == True): s3.send(bNMEAdata)
        except socket.error:
            print ("Error on output socket")
            #sys.exit()
            #time.sleep(0.01)
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
    GPSsentence2 = "$GPVTG"
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
    GPS1_LastUpdate = 0
    GPS2_LastUpdate = 0
    gps1_age_max = 0
    gps2_age_max = 0
    gps1_late = 0
    gps2_late = 0
    total_messages = 0

   

    i=0
    loops =10000
    GPS1wait = 0
    GPS2wait = 0
    #q1 = Queue.LifoQueue(maxsize=0)
#    q1 = Queue(maxsize = 2)
#    q1vtg = Queue(maxsize=2)
#    q2 = Queue(maxsize=2)
#    q2vtg = Queue(maxsize=2)
#    qout = Queue(maxsize = 2)

    q1 = LifoQueue(maxsize = 0)
    q1vtg = LifoQueue(maxsize=0)
    q2 = LifoQueue(maxsize=0)
    q2vtg = LifoQueue(maxsize=0)
    qout = LifoQueue(maxsize = 0)




    
    #q1rmc="nothing"
    #q2rmc="nothing"
    print("Starting GPS reader threads")
    try:
        thread1 = threading.Thread(target=read_data, args=(s1, q1, q1vtg, 1))
        thread2 = threading.Thread(target=read_data, args=(s2, q2, q2vtg, 2))
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
        print("Issue starting Threads")
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
            starttime = time.time()
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
        
                                         
        
        #print("start time: " , starttime)
       
        if(Graphing == True):plt.pause(0.010)
        elasped = time.time()-starttime
        #GPS_elapsed = (((datetime.combine(date.today(),GPS1.timestamp)) - GPS_start).total_seconds())
        #print("elapsed clock:" , elasped, "GPS elasped : ",GPS_elapsed)
        #try:
            #print("in try")
        #if q1.empty() != True:
        #print(q1.qsize(),"  ",  q2.qsize())
        #millis = int(round(time.time() * 1000))
        #print (millis)
        #this works to synchronize messages, need way to allow to continue in case 1 GPS is not sending signals.
#        if (q1.qsize()==1 and q2.qsize()==1):
#            GPS1 = q1.get(GPS1wait)
#            #print(q1.qsize())
#            GPS1_Update = True
#            #print("GPS1..a....",GPS1)
#            
#            GPS2 = q2.get(GPS2wait)
#            GPS2_Update = True
#            #print("                           Update")
#            #time.sleep(0.01)
#            #print("//////GPS2......",GPS2)
        
        #Think about a method to compare the latest timestamp from each GPS and make a decision on what to do
        
        
        
        if (q1.qsize() > 0):    
            try:
                GPS1 = q1.get(GPS1wait)
                #print(q1.qsize())
                GPS1_Update = True
                GPS1_2back = GPS1_LastUpdate
                GPS1_LastUpdate = int(round(time.time() * 1000))
                #print("GPS1..a....",GPS1, time.time)
            except:
                GPS1_Update = False
                #print("No GPS Update GPS1")
        if(q1vtg.qsize() > 0):
            try:
                GPS1vtg = q1vtg.get_nowait()
                GPS1vtgnew = True
                #print("rmc get try GPS1", GPS1rmc)
            except:
                GPS1vtgnew = False
        if (q2.qsize() > 0):
            try:
                #if(GPS1_Update == True):
                    #timeout = .05  #should be equal to update rate
                #else:
                    #timeout = .02
                #GPS2 = q2.get_nowait()
                GPS2 = q2.get(GPS2wait)
                GPS2_Update = True
                GPS2_2back = GPS2_LastUpdate
                GPS2_LastUpdate = int(round(time.time() * 1000))
                #time.sleep(0.01)
                #print("//////GPS2......",GPS2)
            except:
                GPS2_Update = False
                print("No GPS Update GPS2")
        if(q2vtg.qsize()>0):
            try:
                GPS2vtg = q2vtg.get_nowait()
                GPS2vtgnew = True
            except:
                Gps2vtgnew = False
       
        ##ISSUE....DOES NOT GUARANTEE THAT THE TIMESTAMPS ARE MATCHED UP.
        ## CAN HAVE 2 VALID SIGNALS, BUT AT SPEED OF LOOP, IT WILL SOMETIMES MISS
        ##NEED TO FIGURE OUT HOW TO READ AND COMPARE VALIDITY AND TIME STAMPS EACH TIME
        if(Graphing == True):plt3text2.set_text("starting ifs")  
        #time.sleep(.1)
        #plt3text3.set_text((GPS1.timestamp, GPS2.timestamp))
        if(Graphing == True):
            if(GPS1.timestamp > GPS2.timestamp and GPS2_Update == True):
                #GPS2 = q2.get_nowait()
                #GPS2rmc = q2rmc.get_nowait()
                plt3text1.set_text("GPS time 1 > GPS time 2")
                plt3text2.set_text((GPS1.timestamp, "1 waited  ",  GPS2.timestamp))
            elif (GPS1.timestamp < GPS2.timestamp and GPS1_Update == True):
                ax2.set_ylabel(("2nd if", GPS1.timestamp, GPS2.timestamp))
                #GPS1 = q1.get_nowait()
                #GPS1rmc = q1rmc.get_nowait()
                plt3text1.set_text("GPS time 2 > GPS time 1")
                plt3text2.set_text((GPS1.timestamp, "2 waited  ",  GPS2.timestamp))
            else:
                plt3text2.set_text(("else", GPS1.timestamp, GPS2.timestamp, last_timestamp))
            #print("Dropped to else")
        
            #continue
        # need to add way of checking timestamp and only triggering if the valid GPS is new.
        #print(latest_timestamp)
        if(i>5):
            GPS1_age = int(round(time.time() * 1000)) - GPS1_LastUpdate
            GPS2_age = int(round(time.time() * 1000)) - GPS2_LastUpdate

            #print(GPS1_age, GPS2_age)
            if (GPS1_Update == True or GPS2_Update == True):
                
                #print("one update is true")
                if(GPS1_Update == True and GPS2_Update == True):
                    total_messages += 1
                    print("Both true", GPS1_age, GPS2_age)
                    do_GPS_Calcs(GPS1, GPS2, GPS1_Update, GPS2_Update, qout)
                    GPS1_Update = False
                    GPS2_Update = False
                    last_timestamp = max (GPS1.timestamp, GPS2.timestamp)
                    #print("****************BOTH EQUAL******************")
                   
                #print(GPS1_Update, GPS2_Update)
                
                elif(GPS1_Update == True and GPS1_age > 0):
                    total_messages += 1
                    print("in loop for 1 ", end="")
                    #print("GPS1")    
                    last_timestamp = GPS1.timestamp
                    GPS2_update = False
                    do_GPS_Calcs(GPS1, GPS2, GPS1_Update, GPS2_Update, qout)
                    GPS1_Update = False
                    print("    1 > 2    ", GPS1_age, "    ", GPS2_age, "    ", GPS1_LastUpdate - GPS1_2back)
                    gps2_late += 1
                elif(GPS2_Update == True and GPS2_age > 0):
                    total_messages += 1
                    print("in loop for 2 ", end = "")
                    GPS1_Update = False
                    do_GPS_Calcs(GPS1, GPS2, GPS1_Update, GPS2_Update, qout)    
                    last_timestamp = GPS2.timestamp
                    GPS2_Update = False
                    print("     2 > 1", GPS1_age, "    ",  GPS2_age, "    ", delta_time)
                    gps1_late += 1
            
                               
            if(Graphing == True):
                if lgnd == False:
                    lgnd=True
                    #ax.legend()
                    ax.legend(loc='upper center', bbox_to_anchor=(0.5, 1.05), ncol=3)
            
            #print(gps1_age_max, gps2_age_max)
            time.sleep(.01)
                   
                                
        #timestamp = GPS1[7:16]
        #print(timestamp)
                                
    if (Graphing == True):
        plt3text2.set_text("trying to close")
        time.sleep(1)
#    while (q1.empty != True):
#        try:
#            q1.get_nowait()
#            print("Emptying 1") 
#        except:
#            thread1._stop()
    print()    
    print("gps1 Late: " , gps1_late, "GPS 2 late: ", gps2_late, "total messages ", total_messages)    
    print("joining threads")
    thread1.join(3)
    print("1")
    thread2.join(3)
    print("2")
    thread3.join(3)
    print("3")
    #plt.close('all')
    print("exited 1:")
    #thread2._stop()
    print("exited 2:")

print("GoodBye")

    

    #s1.close
    #s2.close
            
                                        
                                        
 
    


#print(stringdata2)

#Track made good and speed over ground
#An example of the VTG message string is:
#
#$GPVTG,,T,,M,0.00,N,0.00,K*4E#
#
#VTG message fields
#Field	Meaning
#0	Message ID $GPVTG
#1	Track made good (degrees true)
#2	T: track made good is relative to true north
#3	Track made good (degrees magnetic)
#4	M: track made good is relative to magnetic north
#5	Speed, in knots
#6	N: speed is measured in knots
#7	Speed over ground in kilometers/hour (kph)
#8	K: speed over ground is measured in kph
#9	The checksum data, always begins with *
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
