
#include <Circle.h>
#include <iostream>

/*!
    \file
    \brief The circle class methods.

    This file contains the code that implements the Circle structure methods.
*/

CircFit::Circle::Circle(float x, float y, float radius)
{
    h = x;
    k = y;
    r = radius;
    if (r>0)
        undefined = false;
    else
        undefined = true;
}

CircFit::Circle::Circle(std::vector<cv::Point> &pts, std::vector<bool> &inliers)
{
    fitCircle(pts, inliers);
}

float CircFit::Circle::fitCircle(std::vector<cv::Point> &pts, std::vector<bool> &inliers, float w, float sigma, float p)
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
    return ransacFit(pts, inliers, nInliers, w, sigma, p);
}


void CircFit::Circle::fitCircle( float x1, float y1, float x2, float y2, float x3, float y3)
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

float CircFit::Circle::fitBestCircle(std::vector<cv::Point> &pts, unsigned int nInliers, unsigned int *idx)
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
        cv::Mat A, b, x;
        
        A = cv::Mat::zeros(cv::Size(3, nInliers), CV_32FC1); 
        b = cv::Mat::zeros(cv::Size(1, nInliers), CV_32FC1);
        for (i=0;i<nInliers;++i)
        {
            A.at<float>(i,0) = pts[idx[i]].x;
            A.at<float>(i,1) = pts[idx[i]].y;
            A.at<float>(i,2) = 1.;
            b.at<float>(i, 0) = -pow(pts[idx[i]].x, 2)-pow(pts[idx[i]].y, 2);
        }
        if (!cv::solve(A, b, x, cv::DECOMP_SVD))
        {
            std::cerr << "Singluar Matrix at fitCircle(std::vector<cv::Point> &pts..." << std::endl << std::endl;
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

void CircFit::Circle::selectAndTestSample(std::vector <cv::Point> &pts, std::vector<bool> &inliers, float thr, unsigned int &nInliers, unsigned int &nOutliers)
{
    unsigned int l, inl;
    unsigned int idx1, idx2, idx3;
    float d;
    std::vector<cv::Point>::iterator it,end;
    std::vector<bool>::iterator itInl;

    l = pts.size();
    if (l >= 3)
    {
      if (inliers.size() != pts.size())
         inliers = std::vector<bool>(l, false);
    
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
        itInl = inliers.begin();
        inl = 0;

        d = distMin(*it);
        if (d <= thr)
        {
            *itInl = true;
            inl++;
        }
         else
            *itInl = false;
        

        //Deal with the rest but the last one.
        for(++it, ++itInl; it+1 != end; it++, itInl++)
        {
            d = distMin(*it);
            if (d <= thr)
            {
               *itInl = true;
               inl++;
            }
            else
               *itInl = false;
        }
        //Deal with the last one.
         d = distMin(*it);
         if (d <= thr)
         {
            *itInl = true;
            inl++;
         }
         else
            *itInl = false;
        nInliers = inl;
        nOutliers = l - inl;
    }
}

// \var p  Es la probabilidad de que un inlier sea una inliers. (p=0.99)
// \var w  Es la proporcion de inliers vs outliers.
float CircFit::Circle::ransacFit(std::vector <cv::Point> &pts, std::vector<bool> &inliers, unsigned int &nInl, float w, float sigma, float p)
{
    float error, thr = 3.84* sigma;
    unsigned int i, j, l, N, T, nInliers, nOutliers, best;
    unsigned int *bestInl, *itBest;
    std::vector<cv::Point>::iterator it, end;
    std::vector<bool>::iterator itInl;

    srand48(time(0));

    l = pts.size();
    if (l<3)
    {
        nInl=0;
        undefined = true;
        return -1;
    }
    if (inliers.size() != l)
      inliers = std::vector<bool>(l, false);

    N = log(1.-p)/log(1-pow(w,3)); //El numero máximo de iteraciones que vamos a realizar.
    T=(unsigned int)(l * w); //El valor de inliers esperado.
    best = 0;
    bestInl = new unsigned int[l];
    for (i=0;i<N;++i)
    {
        selectAndTestSample(pts, inliers, thr, nInliers, nOutliers);
        if (nInliers > best)
        {
            best = nInliers;
            it = pts.begin();
            end = pts.end();
            itInl = inliers.begin();
            itBest = bestInl;
            for (j = 0;it != end; ++j, ++it, ++itInl)
                if (*itInl)
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
        inliers[i] = false;
    for (i=0;i<best;++i)
        inliers[bestInl[i]] = true;

    delete[] bestInl;
    nInl = best;
    return error;
}

float CircFit::Circle::distMin(cv::Point &p)
{
    return fabs(sqrt(pow(h-p.x,2)+pow(k-p.y,2))-r);
}
