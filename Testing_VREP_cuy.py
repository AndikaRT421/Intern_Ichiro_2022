import sim
import sys
import math
import numpy as np
import time



PI = math.pi

sim.simxFinish(-1) # close all opened connections

clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5)  #client ID ke CoppeliaSim

if clientID!=-1:
    print ('Tersambung cuy')
else:
    print ('Gak berhasil tersambung')
    sys.exit('coba ulangi lagi')
    
errorCode,leftMotor_handle=sim.simxGetObjectHandle (clientID,'./leftMotor',sim.simx_opmode_blocking)
errorCode,rightMotor_handle=sim.simxGetObjectHandle (clientID,'./rightMotor',sim.simx_opmode_blocking)

sensor_h=[] #array untuk handle
sensor_val=np.array([]) #array untuk sensor

sensor_loc=np.array([-PI/2, -50/180.0*PI,-30/180.0*PI,-10/180.0*PI,10/180.0*PI,30/180.0*PI,50/180.0*PI,PI/2,PI/2,130/180.0*PI,150/180.0*PI,170/180.0*PI,-170/180.0*PI,-150/180.0*PI,-130/180.0*PI,-PI/2]) 

for x in range(1,16+1):
        errorCode,sensor_handle=sim.simxGetObjectHandle(clientID,'./PioneerP3DX/ultrasonicSensor'+str(x),sim.simx_opmode_blocking)
        sensor_h.append(sensor_handle) #isi array sensor_h        
        errorCode,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector=sim.simxReadProximitySensor(clientID,sensor_handle,sim.simx_opmode_streaming)  
        #Proximity sensor menghasilkan nilai dala  bentuk 3 dimensi, jadi perlu aljabar linear (linalg) untuk mengubah nilai menjadi 1 dimensi              
        sensor_val=np.append(sensor_val,np.linalg.norm(detectedPoint)) #isi array sensor_val

t = time.time()

while (time.time()-t)<60:
    sensor_val = np.array([])
    for x in range(1,16+1):
        errorCode,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector=sim.simxReadProximitySensor(clientID,sensor_h[x-1],sim.simx_opmode_buffer)                
        sensor_val=np.append(sensor_val,np.linalg.norm(detectedPoint))
        
    sensor_sq=sensor_val[0:8]*sensor_val[0:8] #berfokus pada sensor depan
    
    min_ind=np.where(sensor_sq==np.min(sensor_sq))
    min_ind=min_ind[0][0]
    
    if sensor_sq[min_ind]<0.2:
        steer=-1/sensor_loc[min_ind]
    else:
        steer=0
    
    v=1	#kecepatan awal jika tidak ada objek
    kp=0.5	#koefisien untuk belok jika ada objek
    vl=v+kp*steer
    vr=v-kp*steer
    print ("V_l =",vl)
    print ("V_r =",vr)
    
    errorCode=sim.simxSetJointTargetVelocity(clientID,leftMotor_handle,vl, sim.simx_opmode_streaming)
    errorCode=sim.simxSetJointTargetVelocity(clientID,rightMotor_handle,vr, sim.simx_opmode_streaming)


    time.sleep(0.002) #eksekusi looping tiap 0.002 detik (setara 500 Hz)
    
        

errorCode = sim.simxSetJointTargetVelocity(clientID,leftMotor_handle,0,sim.simx_opmode_streaming)
errorCode = sim.simxSetJointTargetVelocity(clientID,rightMotor_handle,0,sim.simx_opmode_streaming)