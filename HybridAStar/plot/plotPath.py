import numpy as np
import matplotlib.pyplot as plt
import string

x_f, y_f, x_b, y_b, d = [], [], [], [], []
obst_x_vec, obst_y_vec = [], []
with open("./build/exportedPath.txt") as A:
    for eachline in A:
        tmp = eachline.split()
 
         
        x_f.append(string.atof(tmp[0]))
        y_f.append(string.atof(tmp[1]))
         

with open("./build/obstacleLine.txt") as A:
    for eachline in A:
        tmp = eachline.split()
        obst_x, obst_y = [], []
        obst_x.append(string.atof(tmp[0]))
        obst_x.append(string.atof(tmp[2]))
        obst_y.append(string.atof(tmp[1]))
        obst_y.append(string.atof(tmp[3]))
        obst_x_vec.append(obst_x)
        obst_y_vec.append(obst_y)

 

plt.figure()
plt.plot(x_f, y_f, "b")
for i in range(len(obst_x_vec)):
    plt.plot(obst_x_vec[i], obst_y_vec[i], "r")

plt.xlim(-30, 30)
plt.ylim(-30, 30)
plt.savefig("exportedPath.jpg")
