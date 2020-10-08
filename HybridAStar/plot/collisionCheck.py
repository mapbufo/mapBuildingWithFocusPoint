import numpy as np
import matplotlib.pyplot as plt
import string

car_x, car_y, obst_x_vec, obst_y_vec = [], [], [], []
with open("./build/carPos.txt") as A:
    for eachline in A:
        tmp = eachline.split()
        car_x.append(string.atof(tmp[0]))
        car_y.append(string.atof(tmp[1]))

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
plt.plot(car_x, car_y, "b")
for i in range(len(obst_x_vec)):
    plt.plot(obst_x_vec[i], obst_y_vec[i], "r")

plt.savefig("collisionCheck.jpg")
