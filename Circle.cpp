
#include <Circle.h>
#include <iostream>

/*!
    \file
    \brief The circle class methods.

    This file contains the code that implements the Circle structure methods.
*/

using namespace std;

Circle::Circle(float x, float y, float radius)
{
    h = x;
    k = y;
    r = radius;
    if (r>0)
        undefined = false;
    else
        undefined = true;
}

Circle::Circle(vector<Point3s> &pts)
{
    fitCircle(pts);
}

float Circle::fitCircle(vector<Point3s> &pts, float w, float sigma, float p)
{
    unsigned int nInliers;
    
    h = k = r = 0;
    undefined = true;
    if (pts.size() <3)
        return -1;
    if (pts.size() == 3)
    {
        fitCircle(pts[0].x, pts[0].y, pts[1].x, pts[1].y, pts[2].x, pts[2].y);
        if (!undefined)
            return 0;
        else
            return -1;
    }
    return ransacFit(pts, nInliers, w, sigma, p);

}


void Circle::fitCircle( float x1, float y1, float x2, float y2, float x3, float y3)
{
    float den;

    den = x2*y3+x1*(y2-y3)-x3*y2+y1*(x3-x2);

    if (den != 0.0)
    {
        float dsq1, dsq2, dsq3;

        dsq1 = -y1*y1-x1*x1;
        dsq2 = -y2*y2-x2*x2;
        dsq3 = -y3*y3-x3*x3;

        den = 1.0/den;

        h = -0.5*den*((y2-y3)*dsq1+(y3-y1)*dsq2+(y1-y2)*dsq3);
        k = -0.5*den*((x3-x2)*dsq1+(x1-x3)*dsq2+(x2-x1)*dsq3);
        r = sqrt(h*h+k*k-den*((x2*y3-x3*y2)*dsq1+(x3*y1-x1*y3)*dsq2+(x1*y2-x2*y1)*dsq3));
        undefined = false;
    }
    else
    {
        h = k = r = 0;
        undefined = true;
    }
}

float Circle::fitBestCircle(vector<Point3s> &pts, unsigned int nInliers, unsigned int *idx)
{
    unsigned int i;
    float error = 0.;

    if (nInliers < 3)
    {
        h = k = r = 0;
        undefined = true;
        return -1.;
    }

    if (nInliers == 3)
    {
        float x[3], y[3];
        for (i=0;i<3 ; ++i)
        {
            x[i] =pts[idx[i]].x;
            y[i] =pts[idx[i]].y;
        }
        fitCircle( x[0], y[0], x[1], y[1], x[2], y[2]);
        if (!undefined)
            return 0.;
        else
            return -1.;
    }
    else
    {
        Mat A, b, x;
        
        A = Mat::zeros(Size(3, nInliers), CV_32FC1); 
        b = Mat::zeros(Size(1, nInliers), CV_32FC1);
        for (i=0;i<nInliers;++i)
        {
            A.at<float>(i,0) = pts[idx[i]].x;
            A.at<float>(i,1) = pts[idx[i]].y;
            A.at<float>(i,2) = 1.;
            b.at<float>(i, 0) = -pow(pts[idx[i]].x, 2)-pow(pts[idx[i]].y, 2);
        }
        if (!solve(A, b, x, DECOMP_SVD))
        {
            cerr << "Singluar Matrix at fitCircle(vector<Point3s> &pts..." << endl << endl;
            h = k = r = 0;
            undefined = true;
        }
        h = -0.5 * x.at<float>(0,0);
        k = -0.5 * x.at<float>(1,0);
        r = sqrt(h*h+k*k-x.at<float>(2,0));
        undefined = false;
        for (i=0;i<nInliers;++i)
            error += pow(distMin(pts[idx[i]]),2);
        error /= nInliers;
        
        return error;
    }
}

void Circle::selectAndTestSample(vector <Point3s> &pts, float thr, unsigned int &nInliers, unsigned int &nOutliers, float kThr=0.)
{
    unsigned int l, inl;
    unsigned int idx1, idx2, idx3;
    float d, K;
    vector<Point3s>::iterator it,end;
    
    l = pts.size();
    if (l >= 3)
    {
        do
        {
            idx1 = (int)(l * drand48());
            idx2 = (int)(l * drand48());
            idx3 = (int)(l * drand48());
        } while (idx3 == idx1 || idx3 == idx2 || idx1 == idx2);

        undefined = true;
        h = k = r = 0;
        fitCircle( pts[idx1].x, pts[idx1].y, pts[idx2].x, pts[idx2].y, pts[idx3].x, pts[idx3].y);
        it = pts.begin();
        end = pts.end();
        inl = 0;

        //Deal with the first coordinate
        K  = computeCurvature(pts[l - 1],pts[0],pts[1]);
        if (!kThr || fabs(r * K - 1) < kThr)
        {
            if (kThr)
               cout << "K:" << K << endl;
            d = distMin(*it);
            if (d <= thr)
            {
                (*it).z = 1;
                inl++;
            }
            else
                (*it).z = 0;
        }
        else
         (*it).z = 0;
        //Deal with the rest but the last one.
        for(++it; it+1 != end; it++)
        {
            K  = computeCurvature(*(it - 1), *(it), *(it + 1));
            if (!kThr || fabs(r * K - 1) < kThr)
            {
               if (kThr)
                  cout << "K:" << K << endl;
               d = distMin(*it);
               if (d <= thr)
               {
                  (*it).z = 1;
                  inl++;
               }
               else
                  (*it).z = 0;
            }
            else
               (*it).z = 0;
        }
        //Deal with the last one.
        K  = computeCurvature(*(it - 1), *(it), pts[0]);
        if (!kThr || fabs(r * K - 1) < kThr)
        {
            if (kThr)
               cout << "K:" << K << endl;
            d = distMin(*it);
            if (d <= thr)
            {
               (*it).z = 1;
               inl++;
            }
            else
               (*it).z = 0;
        }
        else
           (*it).z = 0;
        nInliers = inl;
        nOutliers = l - inl;
    }
}

// \var p  Es la probabilidad de que un inlier sea una inliers. (p=0.99)
// \var w  Es la proporcion de inliers vs outliers.
float Circle::ransacFit(vector <Point3s> &pts, unsigned int &nInl, float w, float sigma, float p, float kThr)
{
    float error, thr = 3.84* sigma;
    unsigned int i, j, l, N, T, nInliers, nOutliers, best;
    unsigned int *bestInl, *itBest;
    vector<Point3s>::iterator itIn, endIn;

    srand48(time(0));

    l=pts.size();
    if (l<3)
    {
        nInl=0;
        undefined = true;
        return -1;
    }    
    N = log(1.-p)/log(1-pow(w,3)); //El numero máximo de iteraciones que vamos a realizar.
    T=(unsigned int)(l * w); //El valor de inliers esperado.
    best = 0;
    bestInl = new unsigned int[l];
    for (i=0;i<N;++i)
    {
        selectAndTestSample(pts, thr, nInliers, nOutliers, kThr);
        if (nInliers > best)
        {
            best = nInliers;
            itIn = pts.begin();
            endIn = pts.end();
            itBest = bestInl;
            for (j=0;itIn != endIn; ++j, ++itIn)
                if ((*itIn).z)
                {
                    *itBest = j; 
                    itBest++;
                }
        }
        if (nInliers>T)
            break;
    }
    error = fitBestCircle(pts, best, bestInl);
    for (i=0;i<pts.size();++i)
        pts[i].z = 0;
    for (i=0;i<best;++i)
        pts[bestInl[i]].z = 1;

    delete[] bestInl;
    nInl = best;
    return error;
}

float Circle::distMin(Point3s &p)
{
    return fabs(sqrt(pow(h-p.x,2)+pow(k-p.y,2))-r);
}

float Circle::computeCurvature(Point3s &p0, Point3s &p1, Point3s &p2)
{
   float dx, dy, dx2, dy2, dxb, dyb, dxf, dyf, Sb, Sf, iS2;

   dxb = p1.x - p0.x;
   dyb = p1.y - p0.y;
   Sb = sqrt(dxb * dxb + dyb * dyb);
   dxf = p2.x - p1.x;
   dyf = p2.y - p1.y;
   Sf = sqrt(dxf * dxf + dyf * dyf);
   assert(Sb != 0 && Sf !=0);
   dx = 0.5 * (dxb / Sb + dxf / Sf);
   dy = 0.5 * (dyb / Sb + dyf / Sf);
   iS2 = 2./((Sb + Sf)*Sb*Sf);
   dx2 = (Sb*dxf-Sf*dxb) * iS2;
   dy2 = (Sb*dyf-Sf*dyb) * iS2;
   return (dx * dy2 - dy * dx2) / pow(dx * dx + dy * dy, 1.5);
}
