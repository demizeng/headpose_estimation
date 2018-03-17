#ifndef QSTRING_CHANGE_HPP_
#define QSTRING_CHANGE_HPP_

#include <iostream>
#include <QString>
#include<math.h>
#include <Eigen/Core>

static QString str2qstr(const std::string str)
{
    return QString::fromLocal8Bit(str.data());
}

static std::string qstr2str(const QString qstr)
{
    QByteArray cdata = qstr.toLocal8Bit();
    return std::string(cdata);
}

static void matrix2angle (Eigen::Matrix4f &result_trans,Eigen::Vector3f &result_angle)
{
  double ax,ay,az;
  if (result_trans(2,0)==1 || result_trans(2,0)==-1)
  {
      az=0;
      double dlta;
      dlta=atan2(result_trans(0,1),result_trans(0,2));
      if (result_trans(2,0)==-1)
      {
          ay=M_PI/2;
          ax=az+dlta;
      }
      else
      {
          ay=-M_PI/2;
          ax=-az+dlta;
      }
  }
  else
  {
      ay=-asin(result_trans(2,0));
      ax=atan2(result_trans(2,1)/cos(ay),result_trans(2,2)/cos(ay));
      az=atan2(result_trans(1,0)/cos(ay),result_trans(0,0)/cos(ay));
  }
  ax=ax*180/M_PI;
  ay=ay*180/M_PI;
  az=az*180/M_PI;
  result_angle<<ax,ay,az;
}

#endif // QSTRING_CHANGE_HPP_
