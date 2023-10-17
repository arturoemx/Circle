#! /usr/bin/env python3
#encoding: utf-8

import numpy as np
import numpy.random as rand
import Circle as cir
import time

data = np.array([[71, 87, 0], [124, 152, 0], [119, 148, 0], [63, 78, 0], [30, 86, 0], [68, 154, 1], [61, 100, 0], [76, 159, 1], [50, 165, 0], [99, 147, 1], [95, 141, 0], [112, 124, 1], [81, 157, 1], [76, 157, 0], [103, 96, 1], [28, 160, 0], [120, 87, 0], [65, 129, 0], [87, 155, 1], [85, 163, 0], [110, 138, 1], [108, 142, 1], [111, 116, 1], [87, 74, 0], [108, 110, 1], [90, 113, 0], [43, 116, 1], [122, 161, 0], [74, 124, 0], [37, 94, 0], [94, 152, 1], [107, 101, 1], [41, 125, 1], [73, 89, 1], [115, 165, 0], [36, 131, 0], [44, 159, 0], [26, 86, 0], [45, 131, 1], [56, 98, 1], [61, 71, 0], [114, 88, 0], [109, 168, 0], [68, 89, 1], [81, 95, 0], [67, 123, 0], [81, 89, 1], [124, 170, 0], [88, 136, 0], [77, 107, 0], [114, 130, 1], [51, 92, 0], [62, 93, 1], [113, 158, 0], [97, 104, 0], [123, 144, 0], [51, 164, 0], [50, 143, 1], [115, 138, 0], [90, 87, 1], [67, 75, 0], [55, 148, 1], [46, 111, 1], [51, 101, 1], [113, 135, 0], [44, 135, 1], [100, 72, 0], [59, 152, 1], [74, 166, 0], [95, 94, 1]], ndmin = 2, dtype=float) 

rand.seed(int(time.time()))
pts=np.zeros((4,2))

pts = 100*rand.random((4,2))-50

print()
print("-" * 80)
for i in range(3):
   print ("C%d = [%f, %f]" % (1, pts[i,0], pts[i,1]))

C0 = cir.Circle()
C0.fitCircle(pts[:3, :])
if not C0.undefined:
   print("C0 = [%f, %f, %f] " % (C0.h, C0.k, C0.r))
else:
   print("No se puede definir un Circulo a partir de puntos colineales.")

print()

ext = C0.distMin(pts[3,:])
print("La distancia mínima entre el punto (%f, %f) y el circulo es %f" % (pts[3, 0], pts[3,1], ext))


pts1 = data.copy()
pts1[:,2]=np.zeros(len(pts1))
print(pts1)
C1 = cir.Circle()
nInliers, error = C1.ransacFit(pts1, 0.4) 
print(pts1)

print("-" * 80)
print("C1 = [%f, %f, %f] " % (C1.h, C1.k, C1.r))
print("Número de puntos típicos: ", nInliers) 
print("Error promedio: ", error)
print("-" * 80)
print()

