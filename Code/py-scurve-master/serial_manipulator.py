import math
from pyscurve import ScurvePlanner
import numpy as np
import matplotlib.pyplot  as plt
import keyboard
# from mpl_toolkits import mplot3d
#from stl import mesh
%matplotlib qt
acc = [[]]
vel = [[]]
pos = [[]]

const = 4800*2*np.pi

class PID_A:    
    def __init__(self,P = 10.0,I = 0.0,p = 0.0):
        self.Kp = P
        self.Ki = I
        self.kp = p
        self.prev_error = 0
        self.prev_pos = 0
        self.prev_time = 0
        self.I_ =0

    def cal(self,f_vel,f_pos,sample_time):
        """Calculates PID value for given reference feedback
        .. math::
            u(t) = K_p e(t) + K_i \int_{0}^{t} e(t)dt + K_d {de}/{dt}
        .. figure:: images/pid_1.png
           :align:   center
           Test PID with Kp=1.2, Ki=1, Kd=0.001 (test_pid.py)
        """
        error = self.setPoint - f_vel
        p_err = self.p_set -  f_pos
        #the proportional term
        P_ = self.Kp * error
        #the intergral term calculation
        self.I_ += error * sample_time
        #position term calculation       
        p_out = (self.kp * p_err)
        #formula for calculating output
        output = P_ + (self.Ki * self.I_) - p_out
        return (output,p_out)
        
    def set_setPoint(self,vset,pset):
        self.setPoint = vset
        self.p_set = pset
        
    def get_setPoint(self):
        return(self.setPoint,self.p_set)
    
        
def pid(pos,vel,time,P,I,p,num):        
    pid = PID_A(0.06,0.01,0.01)  
    pid.set_setPoint(vel[0],pos[0])
    feedback,p_feed = vel[0],pos[0]
    feedback_list,vsetpoint_l,psetpoint_l,output_list,pfeedback_list,time_list = [],[],[],[],[],[]
    sample_time = time[0]
    i = 0
    # print(max(vel))
    while i < len(time):
        output = pid.cal(feedback,p_feed,sample_time)
        feedback += output[0]
        p_feed += output[1] 
        vsetpoint_l.append(vel[i])
        psetpoint_l.append(pos[i])
        feedback_list.append(feedback)
        pfeedback_list.append(p_feed)
        output_list.append(output)
        time_list.append(sample_time)
        pid.set_setPoint(vel[i],pos[i])
        if i > 0 and i % 5000 == 0 and num == 1:
            p_feed += 50
        # if i > 0 and i % 3000 == 0 and num == 2:
        #     # feedback = 9000
        #     p_feed += -20
        # if i > 0 and i % 5000 == 0 and num == 3:
        #     # feedback = 9000
        #     p_feed += +30
        if i>1:
            sample_time = time[i] - time[i-1]
        else:
            sample_time = time[1] - time[0]
        i+=1
        
    plt.plot(time, feedback_list)
    plt.plot(time, vsetpoint_l)
    # a = 
    plt.title("velocity of joint"+str(num))
    plt.show()
    plt.pause(1)
    plt.close()
    
    plt.plot(time, pfeedback_list)
    plt.plot(time, psetpoint_l)
    plt.title("position of joint"+str(num))
    plt.show()
    plt.pause(1)
    plt.close()
    print(max(feedback_list))
    return(feedback_list,pfeedback_list)    

def imp_ob(path):
    f = mesh.Mesh.from_file(path)

    print(f.x)
    print(f.y)
    print(f.z)
        
    volume, cog, inertia = f.get_mass_properties()
    print("Volume                                  = {0}".format(volume))
    print("Position of the center of gravity (COG) = {0}".format(cog))
    print("Inertia matrix at expressed at the COG  = {0}".format(inertia[0,:]))
    print("                                          {0}".format(inertia[1,:]))
    print("                                          {0}".format(inertia[2,:]))

    x_ = f.x
    y_ = f.y
    z_ = f.z

    # Create a new plot
    f.translate([0,0,0])
    f.rotate(axis = "y",theta = 90)
    fig = plt.figure()
    x_ = f.x
    y_ = f.y
    z_ = f.z
    axs = fig.add_subplot(111, projection='3d')
    axs.plot(x_.flatten(),y_.flatten(),z_.flatten())
    axs.fill(x_.flatten(),y_.flatten(),z_.flatten())
    axs.scatter(cog[0],cog[1],cog[2],linewidth = 10)
    plt.ion()
    axs.set_xlabel('X axis')
    axs.set_ylabel('Y axis')
    axs.set_zlabel('Z axis')  
    
    axs.axes.set_xlim3d(left=-200, right=200) 
    axs.axes.set_ylim3d(bottom=-200, top=200) 
    axs.axes.set_zlim3d(bottom=-20, top=400)     
    
    plt.show()    
    plt.pause(10)    
    plt.ioff()

def sim(pos_deg,a,angle=0):

    x,y,z = [],[],[]
    fig = plt.figure(figsize = (9,9),clear =True,frameon =False,tight_layout =True)
    a[0] = a[0]/1000
    
    l1 = 438.922 # connecting rod
    l2 = 353.182   #part of link 2
    
    for i in range(0,time_a,25):
        
        axs = fig.add_subplot(111, projection='3d')
                
        a2xy = a[1]*math.cos(math.radians(pos_deg[1][i]))/1000
        a2x = math.cos(math.radians(pos_deg[0][i]))*a2xy
                  
        a2y = math.sqrt(np.square(a2xy)-np.square(a2x))
        a2z = math.sqrt(np.square(a[1]/1000)-np.square(a2xy))
        
        if pos_deg[1][i] < 0:
            a2z = -a2z           
        
        a3xy = a[2]*math.cos(math.radians(pos_deg[2][i]-pos_deg[1][i]))/1000
        a3x = math.cos(math.radians(pos_deg[0][i]))*a3xy

        a3y = math.sqrt(np.square(a3xy)-np.square(a3x))
        a3z = math.sqrt(np.square(a[2]/1000)-np.square(a3xy))
        
        if pos_deg[0][i] < 0 and pos_deg[0][i] > -90:
            a2y = -a2y
            a3y = -a3y
            
        if pos_deg[2][i]-pos_deg[1][i] < 0:
            a3z = -a3z
            
        phi = 90 - pos_deg[1][i]
        l1_2 = pow(l2,2)-pow(l1,2)
        l2_3 = 2*l2*np.cos(np.radians(phi))
        
        coeff = [1,-l2_3,l1_2]
        l = np.abs(np.roots(coeff))
            
        x.append(a2x+a3x)
        y.append(a2y+a3y)
        z.append(1.7+a2z-a3z-0.4)
        
        #3d plot 
        axs.plot([0,0],[0,0],[0,a[0]],linewidth = 8)
        axs.plot([0,a2x],[0, a2y],[a[0], a[0]+a2z],linewidth = 6)
        axs.plot([a2x,a2x+a3x],[a2y,a2y+a3y],[a[0] + a2z, a[0]+a2z-a3z],linewidth = 4)
        axs.plot([a2x+a3x,a2x+a3x],[a2y+a3y,a2y+a3y],[a[0]+a2z-a3z,a[0]+a2z-a3z-0.4],linewidth = 2)
        axs.plot([1,1,3,3,1],[-2.1,2.1,2.1,-2.1,-2.1],"r")
        axs.plot([a2x*0.16,0],[a2y*0.16,0],[a[0]+(a2z*0.16),(a[0]+l[0]/1000)],linewidth = 4)
        axs.plot([0,0],[0,0],[a[0],a[0]+1.218],linewidth = 6)
        axs.plot([1,3],[0,0],"r",linewidth = 6)
        axs.plot(x,y,z,"bo")
    
        axs.axes.set_xlim3d(left=-4, right=4) 
        axs.axes.set_ylim3d(bottom=-4, top=4) 
        axs.axes.set_zlim3d(bottom=0, top=4.5)     

        axs.set_xlabel('X axis')
        axs.set_ylabel('Y axis')
        axs.set_zlabel('Z axis')
        # axs.view_init(0, angle)
        # angle += 0.1
        plt.show()
        plt.pause(0.00000000001)
        print("iterations left",time_a-i)
        if i  > (time_a - 30): 
            break
        else:
            plt.clf()   
    plt.pause(50)    
    plt.close()

def cal_torque(pos,vel,acc,time,time_l):

    torque = [[0 for j in range(time_a)]for i in range(3)]
    h_torque = [[0 for j in range(time_a)]for i in range(3)]
    p_torque = [[0 for j in range(time_a)]for i in range(3)]
    force = [[0 for j in range(time_a)]for i in range(3)]
    
    for i in range(len(time)):
        if len(time[i]) == time_l:
            time[0] = time[i]
    
    for i in range(time_l):
        
        if i >= (len(pos[0])):
            pos[0].append(pos[0][-1])
            vel[0].append(vel[0][-1])
            acc[0].append(acc[0][-1])
            
        if i >= (len(pos[1])):
            pos[1].append(pos[1][-1])
            vel[1].append(vel[1][-1])
            acc[1].append(acc[1][-1])
        
        if i >= (len(pos[2])):
            pos[2].append(pos[2][-1])
            vel[2].append(vel[2][-1])
            acc[2].append(acc[2][-1])
        
        b = a[2]*np.cos(np.radians(pos[2][i]-pos[1][i]))/(1000*2)
        phi_3 = np.radians(90 - pos[1][i])
        d = b/(np.sin(phi_3)*2)
        r_1 = b*(((a[1]/2*1000))+d)/(d)
           
        W_1 = (m[0]+m[1]+m[2])*9.81
        W_2 = (m[1]+m[2])*9.81
        W_3 = (m[2])*9.81
        
        ir_1 = 59.001/1000
        ir_2 = 968.417/1000
        ir_3 = 560.192/1000
        
        I_1 = m[0]*np.square(ir_1)
        I_2 = m[1]*np.square(ir_2)
        I_3 = m[2]*np.square(ir_3)

        h_torque[0][i] = W_1*r_1/1000000
        h_torque[1][i] = W_2*r_1/1000000
        h_torque[2][i] = W_3*b
        
        p_torque[0][i] = (I_1+I_2+I_3)*acc[0][i]/1000
        p_torque[1][i] = (I_2+I_3)*acc[1][i]/1000
        p_torque[2][i] = I_3*acc[2][i]/1000
         
        # torque[0][i] = h_torque[0][i] + p_torque[0][i]
        # torque[0][i] = h_torque[1][i] + p_torque[1][i]
        # torque[0][i] = h_torque[2][i] + p_torque[2][i]
        

    fg,axs = plt.subplots(3,figsize=(9,9))
    fg.suptitle("torque profile")
    axs[0].set_title("holding torque for theta 1")
    axs[0].plot(time[0],h_torque[0])
    
    axs[1].set_title("holding torque for theta 2")
    axs[1].plot(time[0],h_torque[1])
    
    axs[2].set_title("holding torque for theta 3")
    axs[2].plot(time[0],h_torque[2])        
    plt.show()
    plt.pause(1)
    plt.clf()
    plt.close(fg)
    
    fg,axs = plt.subplots(3,figsize=(9,9))
    fg.suptitle("torque profile")
    axs[0].set_title("moving torque for theta 1")
    axs[0].plot(time[0],p_torque[0])
    
    axs[1].set_title("moving torque for theta 2")
    axs[1].plot(time[0],p_torque[1])
    
    axs[2].set_title("moving torque for theta 3")
    axs[2].plot(time[0],p_torque[2])        
    plt.show()
    plt.pause(1)
    plt.clf()
    plt.close(fg)
    
    # fg,axs = plt.subplots(3,figsize=(9,9))
    # fg.suptitle("torque profile")
    # axs[0].set_title("torque for theta 1")
    # axs[0].plot(time[0],torque[0])
    
    # axs[1].set_title("torque for theta 2")
    # axs[1].plot(time[0],torque[1])
    
    # axs[2].set_title("torque for theta 3")
    # axs[2].plot(time[0],torque[2])        
    # plt.show()
    # plt.pause(10)
    # plt.clf()
    # plt.close(fg)
    
    
    # fg,axs = plt.subplots(3,figsize=(9,9))
    # fg.suptitle("force profile")
    # axs[0].set_title("force for theta 1")
    # axs[0].plot(time[0],force[0])
    
    # axs[1].set_title("force for theta 2")
    # axs[1].plot(time[0],force[1])
    
    # axs[2].set_title("force for theta 3")
    # axs[2].plot(time[0],force[2])        
    # plt.show()
    # plt.pause(10)
    # plt.clf()
    # plt.close(fg)

    return pos,vel,acc,time

def plot_graphs(acc,vel,pos,pos_deg,time,num):
    
    fig, axs = plt.subplots(2,2,figsize=(9, 9))
    a = str(num) + " joint"
    fig.suptitle(a)
    for j in range(len(time)):    
        if i >= len(pos_deg):
            pos_deg[i].append(pos_deg[-1])
            vel[i].append(vel[-1])
            acc[i].append(acc[-1])
    print("time",len(time),len(pos_deg))            

    axs[0,0].set_title("acceleration profile")
    axs[0,0].plot(time, acc)

    axs[0,1].set_title("speed profile")
    axs[0,1].plot(time, vel)
    
    axs[1,0].set_title("position profile")
    axs[1,0].plot(time, pos)

    axs[1,1].set_title("position in degree profile")
    axs[1,1].plot(time, pos_deg)
    plt.show()
    plt.pause(0.01)
    plt.clf()
    plt.close(fig)
    return pos_deg,vel,acc,time

def curves(profile,time,num):
    acc_ = profile[0][:]
    vel_ = profile[1][:]
    pos_ = profile[2][:]
    pos_deg,acc,vel,pos = [],[],[],[]
    
    for i in range(len(pos_)):
        pos_deg.append(np.degrees(pos_[i])/const)
        acc.append(acc_[i])
        pos.append(pos_[i])
        vel.append(vel_[i])
    
#    print(" total time in seconds",time[-1],"\n","total time in minutes",time[-1]/60)
    return plot_graphs(acc,vel,pos,pos_deg,time,num)    
    

def plot_trajectory(traj, dt):
    dof = traj.dof
#    print(traj.time)
    timesteps = int(max(traj.time) / dt)
#    print("timesteps",timesteps)
    time = np.linspace(0, max(traj.time), timesteps)
    # NOW
    # profiles[t]           --- profiles for each DOF at time x[t]
    # profiles[t][d]        --- profile for d DOF at time x[t]
    # profiles[t][d][k]     --- accel/vel/pos profile for d DOF at time x[t]
    p_list = [traj(t) for t in time]
    profiles = np.asarray(p_list)
    # NEED
    # profiles[d]       --- profiles for each DOF 0 <= d <= DOF number
    # profiles[d][k]    --- accel/vel/pos profile for DOF d where j
    # profiles[d][k][t] --- accel/vel/pos at time x[k] for DOF i
    # profiles = np.reshape(profiles, (dof, 3, timesteps))
    r_profiles = np.zeros((dof, 3, timesteps))
    for d in range(dof):
        for p in range(3):
            r_profiles[d, p, :] = profiles[:, d, p]
    for i, profile in zip(range(dof), r_profiles):
        continue
    return(profile,time)
    
# Here is the function that generates the values of scurve     
def s_curve(i_p,f_p,i_v,f_v,max_speed,a_m,j_m,num):
    ini_p = [np.radians(i_p)*const]
    fin_p = [np.radians(f_p)*const]
    ini_v = [i_v]
    fin_v = [f_v]
    v_max = max_speed
    a_max = a_m
    j_max = j_m
    p = ScurvePlanner()
    tr = p.plan_trajectory(ini_p, fin_p, ini_v, fin_v, v_max, a_max, j_max)
    #this passes the values to the plot_trajectory to oragnize the data properly
    profile,time = plot_trajectory(tr,0.001)
    # this the where the graph is plotted
    return curves(profile,time,num)
    
#this is the fuction for calculating the inverse kinematics    
def inv_kin(pos):
    
    #parameters for calculating i have followed angel sodemann's method for finding the parameters at elbom up position
    r_1 = np.sqrt(pow(pos[0],2)+pow(pos[1],2))
    r_2 = pos[2] - a[0]
    
    theta_1 = (np.arctan(pos[1]/pos[0]))*180/math.pi
    
    phi_2 = np.arctan(r_2/r_1)
    
    r = np.sqrt(pow(r_1,2)+pow(r_2,2))
    #cosine law for finding phi_!
    p = pow(a[2],2)-pow(r,2)-pow(a[1],2)
    q = -2*a[1]*r
    
    phi_1 = np.arccos(p/q) 
    theta_2 = (phi_1) + (phi_2)
    theta_2 = (phi_1*180/math.pi) + (phi_2*180/math.pi)
    
    # "cosine law" for finding phi_3
    p = pow(r,2)-pow(a[1],2)-pow(a[2],2)
    q = -2*a[1]*a[2]
    phi_3 = np.arccos(p/q)
    theta_3 = 180 - np.degrees(phi_3)
    
    l1 = 375
    l2 = 300
    phi = 90 - theta_2
    l1_2 = pow(l2,2)-pow(l1,2)
    l2_3 = 2*l2*np.cos(np.radians(phi))
    
    coeff = [1,-l2_3,l1_2]
    l = np.roots(coeff)
    
    return ([theta_1,theta_2,theta_3,l[0]])    

    
if __name__ == "__main__" :
    # the link lenghts
    a = [1700,2200,1800]
    
    pos_deg = [0,0,0]
    acc_a = [0,0,0]
    time_arr =[0,0,0]
    vel_a = [0,0,0]
    #mass of each link
    m = [9,6+1,5+2]
    def_EF_pos = [-10,50,500]
    i_con = inv_kin(def_EF_pos)
    target_pos = [427,500,50]
    f_con = inv_kin(target_pos)
    print(i_con,f_con)
    for i in range(0,len(i_con)-1):       
        
        if i == 0:
            i_p,f_p = -90,90
            
        elif i == 1:
            i_p,f_p = 0,45
            
        elif i == 2:
            i_p,f_p = 40,-10    
        i_p,f_p = i_con[i],f_con[i]    
        pos,vel,acc,time = s_curve(i_p,f_p ,i_v = 0,f_v = 0,max_speed = 24000, a_m = 7000, j_m = 100000, num=i+1)       
        pos_deg[i] = pos
        time_arr[i] = time
        vel_a[i] = vel
        acc_a[i] = acc
        #print(pos_deg)
        if (keyboard.is_pressed("q")):
            break
        
    time_a = max(len(time_arr[0]),len(time_arr[1]),len(time_arr[2]))
    
    pos_deg,vel,acc,time = cal_torque(pos_deg,vel_a,acc_a,time_arr,time_a)
    distance = np.subtract(i_con,f_con)
    np.set_printoptions(formatter={'float_kind':'{:f}'.format})    
    print("theta 1",distance[0])
    print("theta 2",distance[1])
    print("theta 3",distance[2])
    
    for i in range(len(pos_deg)):
        print("vel",max(vel[i]))
        v,p = pid(pos_deg[i], vel[i], time[0],num = i+1, P = 0.5, I = 0.004, p = 0.1)
        pos_deg[i] = p
        vel[i] = v 
    sim(pos_deg,a,angle = 0)
    
    # path = "D:\projects\Intelligent Picking\Part3.stl"
    # imp_ob(path)
    
        
    
    