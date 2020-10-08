import numpy as np
import matplotlib.pyplot as plt
import string

x_f, y_f, x_b, y_b, d = [], [], [], [], []
with open("./build/path.txt") as A:
    for eachline in A:
        tmp = eachline.split()
        d = (string.atof(tmp[3]))

        if(d > 0.0):
            x_f.append(string.atof(tmp[0]))
            y_f.append(string.atof(tmp[1]))
        else:

            x_b.append(string.atof(tmp[0]))
            y_b.append(string.atof(tmp[1]))
plt.figure()
plt.plot(x_f, y_f, "b")
plt.plot(x_b, y_b, "r")

plt.savefig("plot.jpg")
