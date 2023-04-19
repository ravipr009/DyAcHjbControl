

#ifndef H_3x4D_HPP_
#define H_3x4D_HPP_

#include <eigen2/Eigen/Core>
#include <math.h>

Eigen::Matrix4d J_3x4D(Eigen::Vector4d x) /* x denoted the four joint positions */
{

Eigen::MatrixXd J(3,4);
float c1=cos(x[0]);
float c2=cos(x[1]);
float c3=cos(x[2]);
float c4=cos(x[3]);

float s1=sin(x[0]);
float s2=sin(x[1]);
float s3=sin(x[2]);
float s4=sin(x[3]);

J(0,0)= (9*c4*(c1*s3 + c2*c3*s1)) - (110*s1*s2) - (9*c1*s3) - (9*s1*s2*s4) - (9*c2*c3*s1);
J(0,0)=J(0,0)/200;
J(0,1)= (110*c1*c2) - (9*c1*c3*s2) + (9*c1*c2*s4) + (9*c1*c3*c4*s2);
J(0,1)=J(0,1)/200;
J(0,2)= (9*c4*(c3*s1 + c1*c2*s3))/200 - (9*c3*s1)/200 - (9*c1*c2*s3)/200;
J(0,3)= (9*c1*c4*s2)/200 - (9*s4*(s1*s3 - c1*c2*c3))/200;
 
J(1,0)= (11*c1*s2)/20 - (9*s1*s3)/200 + (9*c4*(s1*s3 - c1*c2*c3))/200 + (9*c1*s2*s4)/200 + (9*c1*c2*c3)/200;
J(1,1)= (11*c2*s1)/20 - (9*c3*s1*s2)/200 + (9*c2*s1*s4)/200 + (9*c3*c4*s1*s2)/200;
J(1,2)= (9*c1*c3)/200 - (9*c4*(c1*c3 - c2*s1*s3))/200 - (9*c2*s1*s3)/200;
J(1,3)= (9*s4*(c1*s3 + c2*c3*s1))/200 + (9*c4*s1*s2)/200;
 
J(2,0)=0;
J(2,1)= (9*c2*c3*c4)/200 - (9*c2*c3)/200 - (9*s2*s4)/200 - (11*s2)/20;
J(2,2)= (9*s2*s3)/200 - (9*c4*s2*s3)/200;
J(2,3)= (9*c2*c4)/200 - (9*c3*s2*s4)/200;

return J;

}


#endif
