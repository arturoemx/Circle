#! /usr/bin/env python3
#encoding: utf-8

import numpy as np
import numpy.random as rand
import time

class Circle:
   def __init__(self):
      self.h = self.k = self.r = 0.
      self.undefined = True

   def setCircle(self, x, y, radius):
      self.h, self.k, self.r = x, y, radius
      if self.r > 0:
         self.undefined = False
      else:
         self.h = self.k = self.r = 0.
         self.undefined = True

   def fitCircle(self, pts, w=0.6, sigma=1.0, p=0.99):
      self.h = self.k = self.r = 0.
      self.undefined = True
      if len(pts) < 3:
         return
      if len(pts) == 3:
         self.fitCircle3P(pts)
         if not self.undefined:
            return 0
      else:
         self.ransacFit(pts, nInliers, w, sigma, p)
         return nInliers

   def fitCircle3P(self, pts):
      x1, y1 = pts[0, 0], pts[0, 1]
      x2, y2 = pts[1, 0], pts[1, 1]
      x3, y3 = pts[2, 0], pts[2, 1]

      den = x2 * y3 + x1 * (y2 - y3) - x3 * y2 + y1 * (x3 - x2)

      if den != 0.0:
         dsq1 = -y1 * y1 - x1 * x1
         dsq2 = -y2 * y2 - x2 * x2
         dsq3 = -y3 * y3 - x3 * x3

         den = 1.0/den;

         self.h = -0.5*den*((y2-y3)*dsq1+(y3-y1)*dsq2+(y1-y2)*dsq3);
         self.k = -0.5*den*((x3-x2)*dsq1+(x1-x3)*dsq2+(x2-x1)*dsq3);
         self.r = np.sqrt(self.h**2+self.k**2-den*((x2*y3-x3*y2)*dsq1+(x3*y1-x1*y3)*dsq2+(x1*y2-x2*y1)*dsq3));
         self.undefined = False;
      else:
         self.h = self.k = self.r = 0.
         self.undefined = True;

   def fitBestCircle(self, pts):
      error = 0.
      N = len(pts)
      if N < 3:
         self.h = self.k = self.r = 0.
         self.undefined = True
         return -1

      if N == 3 :
         self.fitCircle3P(pts)
         return 0

      pr, pc = pts.shape
      if pc < 3:
         pts = np.concatenate([pts, np.ones((pr, 1))], 1)
      else:
         pts[:, 2] = np.ones(pr)
      A = pts
      b = (-pts[:,0]**2-pts[:,1]**2).reshape((len(pts),1))

      X = np.linalg.lstsq(A, b, rcond=None)
      x = X[0]
      self.h = -0.5 * x[0];
      self.k = -0.5 * x[1];
      self.r = np.sqrt(self.h ** 2 + self.k ** 2 - x[2])
      self.undefined = True
      error = 0.
      for i in range(N):
         error = error + self.distMin(pts[i]) ** 2;
      error = error / N
      return error

   def selectAndTestSample(self, pts, thr):
      pr,pc = pts.shape
      if pc < 3:
         return -1
      N = len(pts)
      if N >= 3:
         idx1 = rand.randint(0, N)
         idx2 = rand.randint(0, N)
         idx3 = rand.randint(0, N)
         while idx3 == idx1 or idx3 == idx2 or idx1 == idx2:
            idx1 = rand.randint(0, N)
            idx2 = rand.randint(0, N)
            idx3 = rand.randint(0, N)
         self.undefined = False 
         self.h = self.k = self.r = 0.
         self.fitCircle3P(pts[[idx1, idx2, idx3], :])
         inl = 0
         idx=[]
         for i in range(N):
            d = self.distMin(pts[i])
            if d <= thr:
               idx.append(i)
         return idx

   def ransacFit(self, pts, w, sigma =1 , p = 0.99):
      thr = 3.84 * sigma
      rand.seed(int(time.time()))

      l = len(pts)
      if l < 3:
         self.undefined = True
         self.h = self.k = self.r = 0.
         return 0
      pr,pc = pts.shape
      if pr < 3 or pc < 2:
         self.undefined = True
         return -1, -1
      if pc < 3:
         pts = np.concatenate([pts, np.zeros((pr,1))])
      N = int(np.ceil(np.log(1. - p)/np.log(1.-w**3)))
      print("N = ", N)
      T = int(l * w)
      best = 0
      for i in range(N):
         inliers = self.selectAndTestSample(pts, thr)

         nInliers = len(inliers)
         nOutliers = N - nInliers 
         if nInliers > best:
            best = nInliers
            bestInl = np.array(inliers)
            if best != len(bestInl):
               print("GROSSO ERRORE", best, len(bestInl))
         if nInliers > T:
            break
      pts[:,2] = np.zeros(l)
      pts[bestInl, 2] = 1
      error = self.fitBestCircle(pts[bestInl,:])
      return best, error
          
   def distMin(self, p):
      return np.fabs(np.sqrt((self.h-p[0])**2+(self.k-p[1])**2)-self.r)

