#include <Circle.h>
#include <cstdlib>
#include <ctime>
#include <vector>

using namespace std;
using namespace cv;

/*!\file

    \brief This file is an example of the use of the struct Circle.
    
    It shows two different ways of using the struct. First it defines three random points and then if uniquely fit a circle to those points. Then if uses the RANSAC method to fit a circle to a set of 70 2D points; 30 of them are outliers and the rest are inliers.

*/

// there are 30 inliers and 40 outliers. the two first numbers of each row
// correspond to a pair of coordinates, and the third indicates whether this
// point is an ouitlier or an inlier (0 => outlier, 1 => inlier).
//
//The circle parameters are: h=75, k=120, r= 35

int data[70][3] = {{71, 87, 0}, {124, 152, 0}, {119, 148, 0}, {63, 78, 0}, {30, 86, 0}, {68, 154, 1}, {61, 100, 0}, {76, 159, 1}, {50, 165, 0}, {99, 147, 1}, {95, 141, 0}, {112, 124, 1}, {81, 157, 1}, {76, 157, 0}, {103, 96, 1}, {28, 160, 0}, {120, 87, 0}, {65, 129, 0}, {87, 155, 1}, {85, 163, 0}, {110, 138, 1}, {108, 142, 1}, {111, 116, 1}, {87, 74, 0}, {108, 110, 1}, {90, 113, 0}, {43, 116, 1}, {122, 161, 0}, {74, 124, 0}, {37, 94, 0}, {94, 152, 1}, {107, 101, 1}, {41, 125, 1}, {73, 89, 1}, {115, 165, 0}, {36, 131, 0}, {44, 159, 0}, {26, 86, 0}, {45, 131, 1}, {56, 98, 1}, {61, 71, 0}, {114, 88, 0}, {109, 168, 0}, {68, 89, 1}, {81, 95, 0}, {67, 123, 0}, {81, 89, 1}, {124, 170, 0}, {88, 136, 0}, {77, 107, 0}, {114, 130, 1}, {51, 92, 0}, {62, 93, 1}, {113, 158, 0}, {97, 104, 0}, {123, 144, 0}, {51, 164, 0}, {50, 143, 1}, {115, 138, 0}, {90, 87, 1}, {67, 75, 0}, {55, 148, 1}, {46, 111, 1}, {51, 101, 1}, {113, 135, 0}, {44, 135, 1}, {100, 72, 0}, {59, 152, 1}, {74, 166, 0}, {95, 94, 1}};


int main()
{
    short x[4], y[4];
    vector <Point> p;
    vector <bool> inliers;
    Point ext;
    unsigned int i, nInliers = 0;
    float error;


    srand48(time(0));
    for (i=0;i<4;++i)
    {
        x[i] = (int)(100*drand48()-50);
        y[i] = (int)(100*drand48()-50);
        if (i < 3)
        {
            p.push_back(Point(x[i],y[i]));
            cout << "p" << i+1 << " = [" << x[i] << ", " << y[i] << "]" << endl;
        }
    }

    CircFit::Circle C0(p, inliers);
    if (!C0.undefined)
        cout << endl << "C0= [" << C0.h << ", " << C0.k
            << ", " << C0.r << "]" << endl;
    else
        cout <<"Undefined circle: colineal points." << endl;

    ext = Point(x[3], y[3]);
    cout << "La distancia mínima entre el circulo y el punto (" << x[3] << ", " << y[3] << ") = " << C0.distMin(ext) << endl << endl;


    {
        vector<Point> pts(70, Point(0,0));
        vector<bool> inliers(70, false);
        CircFit::Circle C1;

        for (i=0;i<70;++i)
        {
            pts[i].x = data[i][0];
            pts[i].y = data[i][1];
            inliers[i] = (bool)data[i][2];
        }
    
        nInliers = 0;
        // Aquí definimos el umbral de curvatura como 0. porque los puntos del
        // círculo no están organizados como un contorno.
        error = C1.ransacFit(pts, inliers, nInliers, 0.42857, 0.1, 0.99);
        cout << endl << "C1= [" << C1.h << ", " << C1.k
         << ", " << C1.r << "]" << endl << endl;
         cout << "Numero de inliers: " << nInliers << endl;
         cout << "Error promedio   : " << error << endl;
         cout.flush();
    }
    return 0;
}
