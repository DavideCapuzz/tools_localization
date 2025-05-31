/*
 Andrea Toscano
 U-BLOX NEO M8M Parser
*/

#ifndef UBLOX_H_INCLUDED
#define UBLOX_H_INCLUDED

#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <chrono>
#include <iostream>


class Ublox
{
public:

    class Tokeniser {
    public:
        Tokeniser(const std::string& _str, char _token)
            : str(_str), token(_token), start(0) {}

        bool next(std::string& out) {
            if (start >= str.length()) {
                return false; // No more tokens
            }

            size_t end = str.find(token, start);
            if (end == std::string::npos) {
                // No more tokens, take the last part
                out = str.substr(start);
                start = str.length(); // Move start to the end
            } else {
                out = str.substr(start, end - start);
                start = end + 1; // Move start past the token
            }

            return true;
        }

    private:
        std::string str; // The string to tokenize
        char token;      // The token character
        size_t start;    // Current position in the string
    };


    struct satellite
    {
        uint8_t prn;
        int16_t elevation;
        int16_t azimuth;
        uint8_t snr; //signal to noise ratio
    };

    struct _datetime
    {
        uint8_t day, month, year;
        uint8_t hours, minutes, seconds;
        uint16_t millis;
        bool valid; //1 = yes, 0 = no
    };

    enum _fixtype { FIX_TYPE_NONE, FIX_TYPE_GPS, FIX_TYPE_DIFF };
    enum _fix { FIX_NONE = 1, FIX_2D, FIX_3D };
    enum _op_mode { MODE_MANUAL, MODE_AUTOMATIC };


    bool encode(const std::string & msg);

    double latitude, longitude, altitude, vert_speed;
    int latlng_age, alt_age;

    //these units are in hundredths
    //so a speed of 5260 means 52.60km/h
    uint16_t speed, course, knots;
    int speed_age, course_age, knots_age;

    _fixtype fixtype; //0 = no fix, 1 = satellite only, 2 = differential fix
    int fixtype_age;
    _fix fix;
    int fix_age;

    float pdop, hdop, vdop; //positional, horizontal and vertical dilution of precision
    int dop_age;

    int8_t sats_in_use;
    int8_t sats_in_view;

    satellite sats[12];
    int sats_age;

    _datetime datetime;
    int time_age, date_age;

    _op_mode op_mode;

private:
    int millis();
    bool check_checksum(const std::string & msg);

    uint8_t parse_hex(char h);
    bool process_buf();

    //char buf[120];
    uint8_t pos;

    void read_gga(const std::string & msg);
    void read_gsa(const std::string & msg);
    void read_gsv(const std::string & msg);
    void read_rmc(const std::string & msg);
    void read_vtg(const std::string & msg);

};

//extern Ublox gps;

#endif // UBLOX_H_INCLUDED