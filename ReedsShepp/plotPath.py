import numpy as np
import matplotlib.pyplot as plt
import string

x_f, y_f, x_b, y_b, d = [], [], [], [], []
with open("./build/exportedPath.txt") as A:
    for eachline in A:
        tmp = eachline.split()
 
         
        x_f.append(string.atof(tmp[0]))
        y_f.append(string.atof(tmp[1]))
         
plt.figure()
plt.plot(x_f, y_f, "b")


plt.savefig("exportedPath.jpg")
