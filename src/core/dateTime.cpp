#include "dateTime.hpp"

#include <QString>

//Partly taken from http://howardhinnant.github.io/date_algorithms.html
int encodeDateTime(int y, int m, int d, int h)
{
    //Compute Date value : Compute number of days since civil (1970-01-01)
    y -= m <= 2;

    int era = (y >= 0 ? y : y-399) / 400;
    unsigned int yoe = static_cast<unsigned>(y - era * 400);      // [0, 399] yoe = years of era
    unsigned int doy = (153*(m + (m > 2 ? -3 : 9)) + 2)/5 + d-1;  // [0, 365] doy = days of year
    unsigned int doe = yoe * 365 + yoe/4 - yoe/100 + doy;         // [0, 146096] doe = days of era

    int date = era * 146097 + static_cast<int>(doe) - 719468;

    //Add time
    int datetime = date * 24 + h;

    return datetime;
}


int encodeDateTime(std::string date, int hour)
{
    //Split string
    std::string sYear = date.substr(0,4);
    int year = std::stoi(sYear);

    std::string sMonth = date.substr(5,2);
    int month = std::stoi(sMonth);

    std::string sDay = date.substr(8,2);
    int day = std::stoi(sDay);

    return encodeDateTime(year,month,day,hour);
}

int encodeDateTime(std::string datetime)
{
    //Split string
    std::string sYear = datetime.substr(0,4);
    int year = std::stoi(sYear);

    std::string sMonth = datetime.substr(5,2);
    int month = std::stoi(sMonth);

    std::string sDay = datetime.substr(8,2);
    int day = std::stoi(sDay);

    std::string sHour = datetime.substr(11,2);
    int hour = std::stoi(sHour);

    return encodeDateTime(year,month,day,hour);
}

int encodeDateTime(const QDateTime& date)
{
    QString format = "yyyy-MM-dd:hh";
    std::string sDatetime = date.toString(format).toStdString();

    int pos = sDatetime.find(":");
    std::string sDate = sDatetime.substr(0,pos);
    std::string sHour = sDatetime.substr(pos + 1, std::string::npos);

    return encodeDateTime(sDate,std::stoi(sHour));
}

//Partly taken from http://howardhinnant.github.io/date_algorithms.html
std::string decodeDateTime(int dateTime)
{
    std::string sDateTime = "";

    //Gets time
    int time = dateTime % 24;

    //"Extract" time info from dateTime
    dateTime = (dateTime - time) / 24;

    dateTime += 719468; // shift the epoch from 1970-01-01 to 0000-03-01
    int era = (dateTime >= 0 ? dateTime : dateTime - 146096) / 146097;

    unsigned int doe = static_cast<unsigned>(dateTime - era * 146097);          // [0, 146096] doe = days of year
    unsigned int yoe = (doe - doe/1460 + doe/36524 - doe/146096) / 365;  // [0, 399] yoe = years of era
    int y = static_cast<int>(yoe) + era * 400; // y = year

    unsigned int doy = doe - (365*yoe + yoe/4 - yoe/100);                // [0, 365] doy = days of year
    unsigned int mp = (5*doy + 2)/153;                                   // [0, 11] mp = m' = month number starting from March

    unsigned int d = doy - (153*mp+2)/5 + 1;                             // [1, 31] d = day
    unsigned int m = mp + (mp < 10 ? 3 : -9);                            // [1, 12] m = month
    y = y + (m <= 2);


    //Convert to string
    std::string sMonth = "";
    if(m < 10)
        sMonth = "0";
    sMonth += std::to_string(m);


    std::string sDay = "";
    if(d < 10)
        sDay = "0";
    sDay += std::to_string(d);

    std::string sHour = "";
    if(time < 10)
        sHour = "0";
    sHour += std::to_string(time);

    sDateTime = std::to_string(y) + "-" + sMonth + "-" + sDay + ":" + sHour + "00";

    return sDateTime;
}
