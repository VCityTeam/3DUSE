#ifndef DATETIME_HPP
#define DATETIME_HPP

#include <iostream>

//class dateTime
//{
//public:
//    dateTime();
//};

///
/// \brief encodeDateTime Compute number of hours since civil 1970-01-01.  Negative values indicate days prior to 1970-01-01.
/// \param y year
/// \param m month
/// \param d day
/// \param h hour
/// \return Number of hours since civil 1970-01-01.  Negative values indicate days prior to 1970-01-01.
///
int encodeDateTime(int y, int m, int d, int h);

///
/// \brief decodeDateTime Generates a string representing the datetime embedded in an integer. The integer represents the number of hours since civil 1970-01-01.
/// \param dDateTime An integer representing the number of hours since civil 1970-01-01.
/// \return A string representing datetime in the following format : ddmmyyyy:hh00 (Example: 16h00, 2016-10-08 -> 08102016:1600)
///
std::string decodeDateTime(int dateTime);

#endif // DATETIME_HPP
