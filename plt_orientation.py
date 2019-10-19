#! /usr/bin/env python
# -*- coding:utf-8 -*-

import os
import math
import matplotlib.pyplot as plt
import numpy as np
def averagenum(num):
    nsum = 0
    for i in range(len(num)):
        nsum += num[i]
    return nsum / len(num)

if __name__ ==  "__main__":


    p_file = 'orientation_data.txt'
    p_f = os.path.realpath(p_file)
    p_document = open(p_f,'rw+')

    i_file = 'imu_data.txt'
    i_f = os.path.realpath(i_file)
    i_document = open(i_f,'rw+')

    p_time=[]
    p_s=[]
    p_yaw=[]
    p_at=[]
    p_atanf = []

    i_time=[]
    i_yaw=[]
    
    frame = []
    poor = []

    num = 0

    # Read perception_orientation data
    for p_line in p_document:
        p_lines = p_line.split()
        if int(p_lines[2]) == 1:           
           
           p_ts = int(p_lines[0][7:10])*100
           if len(p_lines[1])!= 9 :
              t_square = 9 - len(p_lines[1]) 
              p_tns = int(p_lines[1][0:2])/10**t_square
              p_time.append(p_ts + p_tns)   
           else :
                p_time.append(p_ts + int(p_lines[1][0:2]))

           p_at_h = math.atan2(float(p_lines[5]),float(p_lines[4]))
           #p_at_h = math.atan(float(p_lines[5])/float(p_lines[4]))
           p_at.append(math.degrees(p_at_h))

           p_yaw.append(math.degrees(float(p_lines[3])))
           p_s.append(float(p_lines[7]))
           p_atanf.append(math.degrees(float(p_lines[8])))

    
    # Read imu_orientation data
    for i_line in i_document:
        i_lines = i_line.split()

        i_ts = int(i_lines[0][7:10])*100
        if len(i_lines[1])!= 9 :
           t_square = 9 - len(i_lines[1]) 
           i_tns = int(i_lines[1][0:2])/10**t_square
           i_time.append(i_ts + i_tns)   
        else :
            i_time.append(i_ts + int(i_lines[1][0:2]))
        yaw_init = math.degrees(float(i_lines[2]))

        i_yaw.append(yaw_init)

        '''
        if yaw_init > 0 :
           i_yaw.append(yaw_init-180.0)
        else :
            i_yaw.append(yaw_init + 180.0)
        '''
    nump = 0
    numi = 0

    for p in p_time:
        while abs(p - i_time[numi]) > 10 :
            numi += 1
        poor.append(abs(p_at[nump] - i_yaw[numi]))
        frame.append(nump+1)
        nump += 1


    if len(p_time)!= len(poor):
        print('error!')
    print('abs:',averagenum(poor))

    pminl = p_time[0]
    pmaxl = p_time[len(p_time)-1]

    plt.figure("perception track test") 
    ax1 = plt.subplot(211)
    ax2 = plt.subplot(212)

    color = 'blue'
    ax1.set_xlabel('time')
    ax1.set_ylabel('orientation (degrees)', color=color)
    ax1.set_ylim(-180,180)
    ax1.plot(p_time, p_atanf,'s-', color=color, marker='o',label='perception')
    ax1.legend(loc='upper left')          
    ax1.set_title("perception track orientation test")
    
    ax3 = ax1.twinx()  # instantiate a second axes that shares the same x-axis
    color = 'm'
    #ax3.set_ylabel('imu', labelpad=15, color=color)
    ax3.set_ylim(-180,180)
    ax3.plot(i_time, i_yaw, 's-' , color=color, marker='o',label='imu')
    ax3.legend(loc='upper right') 

    ax2.set_title("range")
    color = 'red'
    ax2.set_xlabel('Frame No.')
    ax2.set_ylabel('orientation(degrees)', color=color)
    ax2.set_ylim(-0,180)
    ax2.plot(frame, poor,'s-', color=color, marker='o',label=' ')
    

    '''
    plt.scatter(i_time,i_yaw,c="r",alpha=0.5)
    plt.xlim(pminl,pmaxl)
    plt.ylim(-200,200)
    plt.xlabel('time')
    plt.ylabel('imu_yaw_red  perception_yaw_blue')

    plt.scatter(p_time,p_at,c="b",alpha=0.5)
    plt.xlim(pminl,pmaxl)
    plt.ylim(-200,200)
    '''


    '''
    plt.scatter(p_time,p_yaw,c="g",alpha=0.5)
    plt.xlim(pminl,pmaxl)
    plt.ylim(-200,200)
    '''

    plt.show()



        

