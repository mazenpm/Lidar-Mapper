# Mazen Maamon
# 400313173



import serial
import numpy as np
import open3d as o3d
import math


s=serial.Serial('COM3', baudrate=115200,timeout=10)
print("Opening: " + s.name)

# reset the buffers
s.reset_output_buffer()
s.reset_input_buffer()

# wait for user to start the program
input("Press Enter to start communication...")
# send the character 's' to MCU via UART

s.write('s'.encode())

y=[]
z=[]
index=0
pi = math.pi
# recieve measurement distances from UART of MCU
s.readline()


for x in range(3):

    print("x="+str(float(x+1)*100))

    for m in range(8):
        while 1:
            p=s.readline()
            if(p!=''):
                break;
        a=p
        distance=int(float(a))
        z.insert(index,(distance*math.cos(pi*m/4)))
        y.insert(index,(distance*math.sin(pi*m/4)))
        print("x,y,z:"+str(float(x+1)*100)+", "+str(y[index])+", "+str(z[index]))
        index+=1

        



# this will close the port
print("Closing: " + s.name)
s.close()


if __name__ == "__main__":
   
    #create new file for test data 
    f = open("demo2dx.xyz", "w")    #create a new file for writing 
    

    #   use for-loop to generate the test data in xyz format
    index=0
    for x in range(3):
        for j in range(8):
            f.write('{0:f} {1:f} {2:f}\n'.format(float(x*100),y[index],z[index]))
            index = index+1

    f.close()                                 
    
    #Read the test data in from the file that was created       
    print("Read in the prism point cloud data (pcd)")
    pcd = o3d.io.read_point_cloud("demo2dx.xyz", format="xyz")

    #Check what the point cloud data looks like numerically       
    print("The PCD array:")
    print(np.asarray(pcd.points))

    #Check what the point cloud data looks like graphically       
    print("Lets visualize the PCD: (spawns seperate interactive window)")
    o3d.visualization.draw_geometries([pcd])



    
    yz_slice_vertex = []
    for x in range(0,24):
        yz_slice_vertex.append([x])

    #here this will define coordinates to connect lines in each yz slice        
    lines = []  
    for x in range(0,24,8):
        lines.append([yz_slice_vertex[x], yz_slice_vertex[x+1]])
        lines.append([yz_slice_vertex[x+1], yz_slice_vertex[x+2]])
        lines.append([yz_slice_vertex[x+2], yz_slice_vertex[x+3]])
        lines.append([yz_slice_vertex[x+3], yz_slice_vertex[x+4]])
        lines.append([yz_slice_vertex[x+4], yz_slice_vertex[x+5]])
        lines.append([yz_slice_vertex[x+5], yz_slice_vertex[x+6]])
        lines.append([yz_slice_vertex[x+6], yz_slice_vertex[x+7]])
        lines.append([yz_slice_vertex[x+7], yz_slice_vertex[x]])

    #here this will define coordinates to connect lines between current and next yz slice        
    for x in range(0,15,8):
        lines.append([yz_slice_vertex[x], yz_slice_vertex[x+8]])
        lines.append([yz_slice_vertex[x+1], yz_slice_vertex[x+9]])
        lines.append([yz_slice_vertex[x+2], yz_slice_vertex[x+10]])
        lines.append([yz_slice_vertex[x+3], yz_slice_vertex[x+11]])
        lines.append([yz_slice_vertex[x+4], yz_slice_vertex[x+12]])
        lines.append([yz_slice_vertex[x+5], yz_slice_vertex[x+13]])
        lines.append([yz_slice_vertex[x+6], yz_slice_vertex[x+14]])
        lines.append([yz_slice_vertex[x+7], yz_slice_vertex[x+15]])

    #This would line map the lines to the 3d coordinate vertices
    line_set = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(np.asarray(pcd.points)),lines=o3d.utility.Vector2iVector(lines))

    #Check what the point cloud data with lines looks like graphically       
    o3d.visualization.draw_geometries([line_set])
