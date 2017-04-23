#!/usr/bin/env python 
# ----------------------------------------------
# MIDG II serial read and ROS Publisher node
# ----------------------------------------------
# This code has been written by: Abdulrahman Renawi
# Any use of this material is allowed ONLY
# with proper citation. Edits CANNOT be done
# before seeking a permission.
# for inquiries and permissions:
# Contact email: abdulrahman.renawi@gmail.com
# ----------------------------------------------
# MTR Lab, American University of Sharjah.
# All rights are reserved.
# ----------------------------------------------
 

import serial, time, struct, numpy, binascii, rospy, tf

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose2D
from std_msgs.msg import Empty
from sensor_msgs.msg import Imu, NavSatFix

from array import array
from string import join
from math import pi
#from bitstring import BitArray


#possible timeout values:
#    1. None: wait forever, block call
#    2. 0: non-blocking mode, return immediately
#    3. x, x is bigger than 0, float allowed, timeout block call

ser = serial.Serial() #define ser as a Serial object

#time_stamp definition:
seq_ID = 0



######### SERIAL PORT SETTINGS #########

#ser.port = "/dev/ttyACM1"  #for arduino
#ser.port = "/dev/ttyUSB0"   #for MIDG
ser.port = "/dev/MIDG"   #for MIDG


ser.baudrate = 115200              #baudrate used (default 115200)
ser.bytesize = serial.EIGHTBITS    #number of bits per bytes
ser.parity = serial.PARITY_NONE    #set parity check: no parity
ser.stopbits = serial.STOPBITS_ONE #number of stop bits is ONE
#ser.timeout = None           #block read
#ser.timeout = 1              #non-block read
#ser.timeout = 2              #timeout block read
ser.xonxoff = False       #disable software flow control
ser.rtscts = False        #disable hardware (RTS/CTS) flow control
ser.dsrdtr = False        #disable hardware (DSR/DTR) flow control
#ser.writeTimeout = 2     #timeout for write



########## ROS NODE DEFINITIONS ##########

#TODO

pub_tf = tf.TransformBroadcaster()

#Theta = Pose2D()
#pub_Theta = rospy.Publisher('MIDG_THETA', Pose2D, queue_size=10)     #  Theta publisher definition

IMU_MSG = Imu()
pub_Imu = rospy.Publisher('MIDG_IMU', Imu, queue_size=10)     #  Imu publisher definition

GPSFIX_MSG = NavSatFix()
pub_GPSFIX = rospy.Publisher('MIDG_GPSFIX', NavSatFix, queue_size=10)     #  Imu publisher definition

rospy.init_node('MIDG_wrapper', anonymous=False)
rate = rospy.Rate(100) # 100hz

######## OPEN SERIAL PORT ##########

print "serial port used:",ser.portstr  #print the port name used 
ser.open()                             #open the serial port



######### VARIABLES DEFINITIOS #########

buff = [0]      #define buffer list (not needed)
get = [0]       #define get list
start = -1      #start = 1 / end = -1 (not used so far)
MSGCOUNT = 0    #a Counter for received messgaes
MSGID = 0       #Initialize MSGID

#Message Types List Definition:
MSGTYPESLIST = ['NAN0','STATUS','IMU_DATA','IMU_MAG','NAN4','NAN5','NAN6','NAN7','NAN8','NAN9','NAV_SENSOR','NAN11','NAV_PV','NAV_HDG','NAN14','NAV_ACC','NAN16','NAN17','NAN18','NAN19','GPS_PV','GPS_SVI','GPS_RAW','GPS_CLK','GPS_EPH','TIM_UTC','TIM_ERR','TIM_PPS','TIM_TM']



######### COMMUNICATION CODE STARTS HERE ###########

if ser.isOpen():        #Global condition (do all below only if serial port is opened)

    ser.flushInput()    #flush input buffer, discarding all its contents
    ser.flushOutput()   #flush output buffer, aborting current output 
                        #and discard all that is in buffer


######### Initial bytes read ############
       
    get[0]=ser.read(1)   #get one byte 
    check=ord(get[0])    #char to int
       
       
       
    while not rospy.is_shutdown():  #press (Ctrl C) to exit
    
    
######### ROS Related tf and headers #####

       seq_ID+=1 #sequence_id increment
       current_time = rospy.Time.now()
       
       # publishing transform
       pub_tf.sendTransform((0.0, 0.0, 0.0),
                        (0.0, 0.0, 0.0, 1.0),
                        current_time,
                        "MIDG_base_frame",
                        "base_footprint")
       
    
######### wait for start bytes ###########        
       
       buff = [0]               #clear buffer (buff) deleting old message bytes
       
                                #wait for start bytes 129 followed by 161                          
       while(check!=129):       #wait for first byte 129
        get[0]=ser.read(1)      #get one byte (initialization)
        check=ord(get[0])       #char to int 
                       
       #when we find a 129
       buff[0]=get[0]          #save first byte in buff
       
       get[0]=ser.read(1)      #get one more
       check=ord(get[0])       #char to int 
       if (check==161):        #check second byte if = 161 proceed
         buff=buff+get         #concatenate to save second byte
           
           
######### Detecting MSGID, MSGTYPE and MSGSIZE ############
         get[0]=ser.read(1)                #get one more
         check=ord(get[0])                 #char to int         
         buff=buff+get                     #concatenate
         MSGID = check                     #saving message ID
         MSGTYPE = MSGTYPESLIST[MSGID]     #retreiving MSGTYPE from MSGTYPESLIST       
         
         get[0]=ser.read(1)      #get one more
         check=ord(get[0])       #char to int 
         buff=buff+get           #concatenate
         MSGSIZE = check+6       #get the fourth byte (payload byte + 6 bytes)            
               
           
######## Start filling the buffer (buff) one byte at a time ########
         
         #start=start*-1     #start = 1  
         i=0
         while(i<MSGSIZE-4):      #saving msg bytes
           get[0]=ser.read(1)     #get one byte           
           check=ord(get[0])      #char to int     
           buff=buff+get          #concatenate
           i+=1


	 
######## Using recieved MSG #########
           
         MSG=buff          #Saving Acknowldged Message at a time instant
         MSGCOUNT+=1       #Keeping track of how many messages we acknowledge

       
         #print "Message No.",MSGCOUNT,"\nMessage ID:",MSGID,"\nMessage Type:",MSGTYPE,"\nMessage Size:",MSGSIZE ,"bytes"
         #print "raw MSG bytes:",MSG
     

     
######## Publishing Received Message #######

#MSG is the message container starts from [129 161...] of size = (payload+6) = MSGSIZE


######### NAV_SENSOR (IMUs):

       ## Indentation* inside while(1):  #looping the whole thing

       if (MSGID==10):                                      #NAV_SENSOR case payload = 39 bytes
        NAV_SENSOR_MSG_PAYLOAD=MSGSIZE-6                    #PAYLOAD is the length of the data message without the 6 framing bytes
        NAV_SENSOR_MSG=MSG[4:NAV_SENSOR_MSG_PAYLOAD+4]      #Obtaining the message without framing bytes
        
        #time stamp
        NAV_SENSOR_timestamp=NAV_SENSOR_MSG[0:4]                                #timestamp
        NAV_SENSOR_timestamp=''.join(NAV_SENSOR_timestamp)			#joining the bytes
	NAV_SENSOR_timestamp=struct.unpack(">I", NAV_SENSOR_timestamp)[0]	#unpack (convert to big endian int 32) ,unit: msec
	
	#Angular rates
        NAV_SENSOR_p=NAV_SENSOR_MSG[4:6]		    #p Angular Velocity in x ,unit: rad/sec
        NAV_SENSOR_p=''.join(NAV_SENSOR_p)
	NAV_SENSOR_p=(struct.unpack(">h",  NAV_SENSOR_p)[0])*0.01*(pi/180)         #signed int 16
        NAV_SENSOR_q=NAV_SENSOR_MSG[6:8]		    #q Angular Velocity in y ,unit: rad/sec
        NAV_SENSOR_q=''.join(NAV_SENSOR_q)
	NAV_SENSOR_q=(struct.unpack(">h",  NAV_SENSOR_q)[0])*0.01*(pi/180)	   #signed int 16
        NAV_SENSOR_r=NAV_SENSOR_MSG[8:10]		    #r Angular Velocity in z ,unit: rad/sec
        NAV_SENSOR_r=''.join(NAV_SENSOR_r)
	NAV_SENSOR_r=(struct.unpack(">h",  NAV_SENSOR_r)[0])*0.01*(pi/180)         #signed int 16
        
        #Accelerations
        NAV_SENSOR_Accx=NAV_SENSOR_MSG[10:12]		    #Accx Acceleration in x ,m/sec^2
        NAV_SENSOR_Accx=''.join(NAV_SENSOR_Accx)
	NAV_SENSOR_Accx=(struct.unpack(">h",  NAV_SENSOR_Accx)[0])*9.799096177*(10**-3)	#signed int 16
        NAV_SENSOR_Accy=NAV_SENSOR_MSG[12:14]		    #Accy Acceleration in y ,m/sec^2
        NAV_SENSOR_Accy=''.join(NAV_SENSOR_Accy)
	NAV_SENSOR_Accy=(struct.unpack(">h",  NAV_SENSOR_Accy)[0])*9.799096177*(10**-3)	#signed int 16
        NAV_SENSOR_Accz=NAV_SENSOR_MSG[14:16]		    #Accz Acceleration in z ,m/sec^2
        NAV_SENSOR_Accz=''.join(NAV_SENSOR_Accz)
	NAV_SENSOR_Accz=(struct.unpack(">h",  NAV_SENSOR_Accz)[0])*9.799096177*(10**-3)	#signed int 16
        
        #Yaw,Pitch,Roll
        NAV_SENSOR_Mz=NAV_SENSOR_MSG[16:18]		    #Mz Yaw (in z) ,unit: rad
        NAV_SENSOR_Mz=''.join(NAV_SENSOR_Mz)
	NAV_SENSOR_Mz=(struct.unpack(">h",  NAV_SENSOR_Mz)[0])*0.01*(pi/180)  
        NAV_SENSOR_My=NAV_SENSOR_MSG[18:20]		    #My Pitch (in y) ,unit: rad
        NAV_SENSOR_My=''.join(NAV_SENSOR_My)
	NAV_SENSOR_My=(struct.unpack(">h",  NAV_SENSOR_My)[0])*0.01*(pi/180)  
	NAV_SENSOR_Mx=NAV_SENSOR_MSG[20:22]		    #Mx Roll (in x) ,unit: rad
	NAV_SENSOR_Mx=''.join(NAV_SENSOR_Mx)
	NAV_SENSOR_Mx=(struct.unpack(">h",  NAV_SENSOR_Mx)[0])*0.01*(pi/180)  
	
        #Quaternions(Qw,Qx,Qy,Qz)
        NAV_SENSOR_Qw=NAV_SENSOR_MSG[22:26]			#Orientaion Quaternion Qw
        NAV_SENSOR_Qw=''.join(NAV_SENSOR_Qw)
        NAV_SENSOR_Qw=(struct.unpack(">i",  NAV_SENSOR_Qw)[0])*(2**-30)        
        NAV_SENSOR_Qx=NAV_SENSOR_MSG[26:30]			#Orientaion Quaternion Qx
        NAV_SENSOR_Qx=''.join(NAV_SENSOR_Qx)
        NAV_SENSOR_Qx=(struct.unpack(">i",  NAV_SENSOR_Qx)[0])*(2**-30)      
        NAV_SENSOR_Qy=NAV_SENSOR_MSG[30:34]			#Orientaion Quaternion Qy
        NAV_SENSOR_Qy=''.join(NAV_SENSOR_Qy)
        NAV_SENSOR_Qy=(struct.unpack(">i",  NAV_SENSOR_Qy)[0])*(2**-30)        
        NAV_SENSOR_Qz=NAV_SENSOR_MSG[34:38]			#Orientaion Quaternion Qz
        NAV_SENSOR_Qz=''.join(NAV_SENSOR_Qz)
        NAV_SENSOR_Qz=(struct.unpack(">i",  NAV_SENSOR_Qz)[0])*(2**-30)
              
	
	NAV_SENSOR_HEADER=MSGTYPE     #Header to identify the published message
	
	
	#TYPE Imu.msg
	#feed Imu header
	IMU_MSG.header.seq = seq_ID
	IMU_MSG.header.stamp = current_time
	IMU_MSG.header.frame_id = "MIDG_base_frame"
	
	#feed Imu Qw,Qx,Qy,Qz orientation
	IMU_MSG.orientation.x = NAV_SENSOR_Qx
	IMU_MSG.orientation.y = NAV_SENSOR_Qy
	IMU_MSG.orientation.z = NAV_SENSOR_Qz
	IMU_MSG.orientation.w = NAV_SENSOR_Qw	
	IMU_MSG.orientation_covariance = [0.0005, 0.0, 0.0,
					  0.0, 0.0005, 0.0,
					  0.0, 0.0, 0.0005]
	
	#feed Imu p(x),q(y),r(z) angular_velocity 
	IMU_MSG.angular_velocity.x = NAV_SENSOR_p
	IMU_MSG.angular_velocity.y = NAV_SENSOR_q
	IMU_MSG.angular_velocity.z = NAV_SENSOR_r
	IMU_MSG.angular_velocity_covariance = [0.0005, 0.0, 0.0,
					       0.0, 0.0005, 0.0,
					       0.0, 0.0, 0.0005]
						
	#publish Accx,Accy,Accz (m/s^2)
	IMU_MSG.linear_acceleration.x = NAV_SENSOR_Accx
	IMU_MSG.linear_acceleration.y = NAV_SENSOR_Accy
	IMU_MSG.linear_acceleration.z = NAV_SENSOR_Accz
	IMU_MSG.linear_acceleration_covariance = [0.0005, 0.0, 0.0,
					  	  0.0, 0.0005, 0.0,
					  	  0.0, 0.0, 0.0005]
	pub_Imu.publish(IMU_MSG)
	

	
	
	#Combining the message parts as one array of data:
	NAV_SENSOR_OUT=[MSGTYPE,NAV_SENSOR_timestamp,NAV_SENSOR_p,NAV_SENSOR_q,NAV_SENSOR_r,NAV_SENSOR_Accx,NAV_SENSOR_Accy,NAV_SENSOR_Accz,NAV_SENSOR_Mz,NAV_SENSOR_My,NAV_SENSOR_Mx,NAV_SENSOR_Qw,NAV_SENSOR_Qx,NAV_SENSOR_Qy,NAV_SENSOR_Qz]
	
        #print NAV_SENSOR_OUT
        


######### NAV_PV (IMU INS Solution):        

       ## Indentation* inside while(1):  #looping the whole thing
       
       if (MSGID==12):					#NAV_PV case payload = 29 bytes
       	NAV_PV_MSG_PAYLOAD=MSGSIZE-6                    #PAYLOAD is the length of the data message without the 6 framing bytes
        NAV_PV_MSG=MSG[4:NAV_PV_MSG_PAYLOAD+4]          #Obtaining the message without framing bytes
	
	#time stamp
	NAV_PV_timestamp=NAV_PV_MSG[0:4]                                #timestamp
        NAV_PV_timestamp=''.join(NAV_PV_timestamp)			#joining the bytes
	NAV_PV_timestamp=struct.unpack(">I", NAV_PV_timestamp)[0]	#unpack (convert to big endian int 32) ,unit: msec

	#This position in converted to float 64 :Latitude degrees,Longtitude degrees,Altitude meters for ROS NavSatFix.msg TODO
	#position 
	NAV_PV_X=NAV_PV_MSG[4:8]		    	#X-axis Position ,unit: (cm(ECEF) or 10-7 deg(LLA))  I chose LLA to use NavSatFix.msg in ROS
        NAV_PV_X=''.join(NAV_PV_X)
	NAV_PV_X=(struct.unpack(">i",  NAV_PV_X)[0])    #signed int 32
	
	NAV_PV_Y=NAV_PV_MSG[8:12]		    	#Y-axis Position ,unit: (cm or 10-7 deg)
        NAV_PV_Y=''.join(NAV_PV_Y)
	NAV_PV_Y=(struct.unpack(">i",  NAV_PV_Y)[0])    #signed int 32
	
	NAV_PV_Z=NAV_PV_MSG[12:16]		    	#Z-axis Position ,unit: cm
        NAV_PV_Z=''.join(NAV_PV_Z)
	NAV_PV_Z=(struct.unpack(">i",  NAV_PV_Z)[0])    #signed int 32
	
	#velocities...	
	
	#solution details bits   
	NAV_PV_BITS=NAV_PV_MSG[28:29]			#Solution Details BITS
	NAV_PV_BITS=''.join(NAV_PV_BITS)		#TO BE CONVERTED TO BINARY BITS
	NAV_PV_BITS=bin(int(binascii.hexlify(NAV_PV_BITS), 16))

	
	NAV_PV_HEADER=MSGTYPE     #Header to identify the published message
	
	
	#TYPE Point.msg   (cm)
	#publish X
	#publish Y
	#publish Z
	#TODO

		
	#Combining the message parts as one array of data:
	NAV_PV_OUT=[NAV_PV_HEADER,NAV_PV_timestamp,NAV_PV_X,NAV_PV_Y,NAV_PV_Z,NAV_PV_BITS]

	#print NAV_PV_OUT


######### NAV_HDG:    

       ## Indentation* inside while(1):  #looping the whole thing

       if (MSGID==13):					  #NAV_HDG case payload = 13 bytes
	NAV_HDG_MSG_PAYLOAD=MSGSIZE-6                     #PAYLOAD is the length of the data message without the 6 framing bytes
        NAV_HDG_MSG=MSG[4:NAV_HDG_MSG_PAYLOAD+4]          #Obtaining the message without framing bytes
	
	#time stamp
	NAV_HDG_timestamp=NAV_HDG_MSG[0:4]                              #timestamp
        NAV_HDG_timestamp=''.join(NAV_HDG_timestamp)			#joining the bytes
	NAV_HDG_timestamp=struct.unpack(">I", NAV_HDG_timestamp)[0]	#unpack (convert to big endian unsigned int 32) ,unit: msec
	
	#Magnetic Heading
	NAV_HDG_MHDG=NAV_HDG_MSG[4:6]		    #Magnetic Heading ,unit: deg
        NAV_HDG_MHDG=''.join(NAV_HDG_MHDG)
	NAV_HDG_MHDG=(struct.unpack(">h",  NAV_HDG_MHDG)[0])*0.01*(pi/180)	#signed int 16
	
	
	NAV_HDG_HEADER=MSGTYPE     #Header to identify the published message
	
	
	#TYPE Pose2D.msg 
	#publish theta
	#TODO

	
	
	#Combining the message parts as one array of data:
	NAV_HDG_OUT=[NAV_HDG_HEADER,NAV_HDG_timestamp,NAV_HDG_MHDG]
	
	#print NAV_HDG_OUT	
	
	
######### GPS_PV:  NavSatFix.msg without INS
	
       ## Indentation* inside while(1):  #looping the whole thing	

       if (MSGID==20):					#GPS_PV case payload = 38 bytes	
	GPS_PV_MSG_PAYLOAD=MSGSIZE-6                    #PAYLOAD is the length of the data message without the 6 framing bytes
        GPS_PV_MSG=MSG[4:GPS_PV_MSG_PAYLOAD+4]         #Obtaining the message without framing bytes
	
	#GPS time	
	GPS_PV_GPSTIME=GPS_PV_MSG[0:4]                          #GPS TIME
        GPS_PV_GPSTIME=''.join(GPS_PV_GPSTIME)			#joining the bytes
	GPS_PV_GPSTIME=struct.unpack(">I", GPS_PV_GPSTIME)[0]	#unpack (convert to big endian unsigned int 32) ,unit: msec
	
	#GPS week
	GPS_PV_GPSWEEK=GPS_PV_MSG[4:6]				#GPS WEEK
	GPS_PV_GPSWEEK=''.join(GPS_PV_GPSWEEK)			#joining the bytes
	GPS_PV_GPSWEEK=struct.unpack(">H", GPS_PV_GPSWEEK)[0]	#unpack (convert to big endian unsigned int 16) ,unit: Count
	
	#solution details bits
	GPS_PV_BITS=GPS_PV_MSG[6:8]				#Solution Details 16 BITS
	#GPS_PV_BITS=''.join(GPS_PV_BITS)			#TO BE CONVERTED TO BINARY BITS
 	GPS_PV_BITS[0]=struct.unpack(">B", GPS_PV_BITS[0])[0]	#Number of Satellites and GPS Fix Type
 	GPS_PV_BITS[1]=struct.unpack(">B", GPS_PV_BITS[1])[0]	#Configuration Bits
 	GPS_PV_SAT = int(GPS_PV_BITS[0]/10)			#Number of Satellites used in solution
 	GPS_PV_FIXTYPE = int(GPS_PV_BITS[0]-10*GPS_PV_SAT)	#GPS Fix Type
 	GPS_PV_BITS = [GPS_PV_SAT,GPS_PV_FIXTYPE,GPS_PV_BITS[1]]
	
	
	#position in LLA	
	GPS_PV_X=GPS_PV_MSG[8:12]				#X-axis Position ,unit: obtained from BITS (10^-7 deg)
	GPS_PV_X=''.join(GPS_PV_X)
	GPS_PV_X=struct.unpack(">i", GPS_PV_X)[0]*10**-7	#signed float degrees longitude
	
	GPS_PV_Y=GPS_PV_MSG[12:16]				#Y-axis Position ,unit: obtained from BITS (cm or 10^-7 deg)
	GPS_PV_Y=''.join(GPS_PV_Y)
	GPS_PV_Y=struct.unpack(">i", GPS_PV_Y)[0]*10**-7	#signed float degrees latitude
	
	GPS_PV_Z=GPS_PV_MSG[16:20]				#Y-axis Position ,unit: cm
	GPS_PV_Z=''.join(GPS_PV_Z)
	GPS_PV_Z=struct.unpack(">i", GPS_PV_Z)[0]*10**-2	#signed float meters altitude
	
	
	#velocities...
	
	#Position DOP....
	
	#Position Accuracy = sqrt(Variance)
	GPS_PV_POSACCURACY=GPS_PV_MSG[34:36]
	GPS_PV_POSACCURACY=''.join(GPS_PV_POSACCURACY)
	GPS_PV_POSACCURACY=struct.unpack(">H", GPS_PV_POSACCURACY)[0]*10**-2	#position_covariance in meters
	
	#Velocity Accuracy... 
	
	GPS_PV_HEADER=MSGTYPE     #Header to identify the published message
	
	
	
	#TYPE NavSatFix.msg
	#feed NavSatFix header
	GPSFIX_MSG.header.seq = seq_ID
	GPSFIX_MSG.header.stamp = current_time
	GPSFIX_MSG.header.frame_id = "MIDG_base_frame"

	#feed NavSatFix status
	if GPS_PV_FIXTYPE==0: #if Bits[8:11]==0 NavSatFix.status.status = -1 (NO Fix!)
		GPSFIX_MSG.status.status = -1
	else:
		GPSFIX_MSG.status.status = 0 #Unaugmented fix
	##if Bits[8:11]==1 NavSatFix.status.status = 0 ( Unaugmented fix; dead reckoning only, 2D or 3D Fix)
	#if Bits[8:11]==4 NavSatFix.status.status = 2 (ground augmented; GPS+Dead reckoning)    
	
	#feed NavSatFix latitude
	GPSFIX_MSG.latitude=GPS_PV_Y
	#feed NavSatFix longitude
	GPSFIX_MSG.longitude=GPS_PV_X
	#feed NavSatFix altitude
	GPSFIX_MSG.altitude=GPS_PV_Z
	
	#feed NavSatFix position_covariance = Accuracy^2
	GPSFIX_MSG.position_covariance[0]=GPS_PV_POSACCURACY*GPS_PV_POSACCURACY
	GPSFIX_MSG.position_covariance[4]=GPS_PV_POSACCURACY*GPS_PV_POSACCURACY
	GPSFIX_MSG.position_covariance[8]=GPS_PV_POSACCURACY*GPS_PV_POSACCURACY
	
	#feed position_covariance_type
	GPSFIX_MSG.position_covariance_type=2  #0: Unknown - 2:Diagonal
	
	pub_GPSFIX.publish(GPSFIX_MSG)
		
	#Combining the message parts as one array of data:
	GPS_PV_OUT=[GPS_PV_HEADER,GPS_PV_GPSTIME,GPS_PV_GPSWEEK,GPS_PV_BITS,GPS_PV_X,GPS_PV_Y,GPS_PV_Z,GPS_PV_POSACCURACY]
	
	#print GPS_PV_OUT
	

########################## -------------- END END END --------------- ###########################
	


       #ser.close()  #Closing the serial port used
         
                 
         #UNUSED CODES:    
         #time.sleep(5)  #give the serial port some time to receive the data 
         #data = bytearray(ser.read(35),"bz2_codec")   #NAV_PV msg size is 35 bytes (read the whole msg) and save it as an array of bytes
         #raw=array('c', ser.read(35))  #NAV_PV msg size is 35 bytes (read the a whole msg)

         
         

  
