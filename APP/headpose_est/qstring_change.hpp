#ifndef QSTRING_CHANGE_HPP_
#define QSTRING_CHANGE_HPP_

#include <iostream>
#include <QString>

static QString str2qstr(const std::string str)
{
    return QString::fromLocal8Bit(str.data());
}

static std::string qstr2str(const QString qstr)
{
    QByteArray cdata = qstr.toLocal8Bit();
    return std::string(cdata);
}

#endif // QSTRING_CHANGE_HPP_
