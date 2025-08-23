#include "tools_localization/interfaces/ublox_parser.hpp"

// Ublox::Tokeniser::Tokeniser(char* _str, char _token)
// {
//     str = _str;
//     token = _token;
// }


// bool Ublox::Tokeniser::next(char* out, int len)
// {
//     uint8_t count = 0;

//     if(str[0] == 0)
//         return false;

//     while(true)
//     {
//         if(str[count] == '\0')
//         {
//             out[count] = '\0';
//             str = &str[count];
//             return true;
//         }

//         if(str[count] == token)
//         {
//             out[count] = '\0';
//             count++;
//             str = &str[count];
//             return true;
//         }

//         if(count < len)
//             out[count] = str[count];

//         count++;
//     }
//     return false;
// }


bool Ublox::encode(const std::string & msg)
{
    
    // if(!check_checksum(msg)) //if checksum is bad
    // {
    //     return false; //return
    // }

    //otherwise, what sort of message is it
    // std::cout<<msg.compare(0, 6, "$GNGGA")<<" "<<msg<<"\n";
    if(msg.compare(0, 6, "$GNGGA") == 0)    
    {      
        read_gga(msg);
    }
    if(msg.compare(0, 6, "$GNGSA") == 0)
    {
        read_gsa(msg);
    }

    if(msg.compare(0, 6, "$GPGSV") == 0)
    {
        read_gsv(msg);
    }

    if(msg.compare(0, 6, "$GNRMC") == 0)
    {
        read_rmc(msg);
    }
    if(msg.compare(0, 6, "$GNVTG") == 0)
    {
        read_vtg(msg);
    }
    return true;
}

int Ublox::millis()
{

  auto now = std::chrono::high_resolution_clock::now();
  auto milliseconds = std::chrono::time_point_cast<std::chrono::milliseconds>(now);
  auto epoch = milliseconds.time_since_epoch().count();
  return static_cast<int>(epoch);
}

// GNGGA 
void Ublox::read_gga(const std::string & msg)
{
    int counter = 0;
    std::string token;
    Tokeniser tok(msg, ',');

    while(tok.next(token))
    {
        switch(counter)
        {
        case 1: //time
        {
          if (token.compare("")!=0)
          {
            float time = std::stof(token);
            int hms = int(time);

            datetime.millis = time - hms;
            datetime.seconds = fmod(hms, 100);
            hms /= 100;
            datetime.minutes = fmod(hms, 100);
            hms /= 100;
            datetime.hours = hms;
            time_age = millis();
          }
        }
        break;
        case 2: //latitude
        {
          if (token.compare("")!=0)
          {
            double llat = std::stod(token);
            int ilat = llat/100;
            double mins = fmod(llat, 100);
            latitude = ilat + (mins/60);
          }
        }
        break;
        case 3: //north/south
        {
            if(token[0] == 'S')
                latitude = -latitude;
        }
        break;
        case 4: //longitude
        {
          if (token.compare("")!=0)
          {
            double llong = std::stod(token);
            int ilat = llong/100;
            double mins = fmod(llong, 100);
            longitude = ilat + (mins/60);
          }
        }
        break;
        case 5: //east/west
        {
            if(token[0] == 'W')
                longitude = -longitude;
            latlng_age = millis();
        }
        break;
        case 6:
        {
          if (token.compare("")!=0)
          {
            fixtype = _fixtype(std::stoi(token));
          }
        }
        break;
        case 7:
        {
          if (token.compare("")!=0)
          {
            sats_in_use = std::stoi(token);
          }
        }
        break;
        case 8:
        {
          if (token.compare("")!=0)
          {
            hdop = std::stoi(token);
          }
        }
        break;
        case 9:
        {
          if (token.compare("")!=0)
          {
            double new_alt = std::stod(token);
            vert_speed = (new_alt - altitude)/((millis()-alt_age)/1000.0);
            altitude = std::stod(token);
            alt_age = millis();
          }
        }
        break;
        }
        counter++;
    }
}


void Ublox::read_gsa(const std::string & msg)
{
    int counter = 0;
    std::string token;
    Tokeniser tok(msg, ',');

    while(tok.next(token))
    {
        switch(counter)
        {
        case 1: //operating mode
        {
          if (token.compare("")!=0)
          {
            if(token[0] == 'A')
                op_mode = MODE_AUTOMATIC;
            if(token[0] == 'M')
                op_mode = MODE_MANUAL;
          }
        }
        break;
        case 2:
        {
          if (token.compare("")!=0)
          {
            fix = _fix(std::stoi(token));
            fix_age = millis();
          }
        }
        break;
        case 14:
        {
          if (token.compare("")!=0)
          {
            pdop = std::stof(token);
          }
        }
        break;
        case 15:
        {
          if (token.compare("")!=0)
          {
            hdop = std::stof(token);
          }
        }
        break;
        case 16:
        {
          if (token.compare("")!=0)
          {
            vdop = std::stof(token);
            dop_age = millis();
          }
        }
        break;
        }
        counter++;
    }
}


void Ublox::read_gsv(const std::string & msg)
{
    std::string token;
    Tokeniser tok(msg, ',');

    tok.next(token);
    tok.next(token);

    tok.next(token);
    int mn {0};
    if (token.compare("")!=0)
    {
      mn = std::stoi(token); //msg number
    }

    tok.next(token);
    if (token.compare("")!=0)
    {
      sats_in_view = std::stoi(token); //number of sats
    }

    int8_t j = (mn-1) * 4;
    int8_t i;

    // for(i = 0; i <= 3; i++)
    // {
    //   tok.next(token);
    //   if (token.compare("")!=0)
    //   {
    //     sats[j+i].prn = std::stoi(token);
    //   }

    //   tok.next(token);
    //   if (token.compare("")!=0)
    //   {
    //     sats[j+i].elevation = std::stoi(token);
    //   }

    //   tok.next(token);
    //   if (token.compare("")!=0)
    //   {
    //     sats[j+i].azimuth = std::stoi(token);
    //   }

    //   tok.next(token);
    //   if (token.compare("")!=0)
    //   {
    //     sats[j+i].snr = std::stoi(token);
    //   }
    // }
    // sats_age = millis();
}


void Ublox::read_rmc(const std::string & msg)
{
    int counter = 0;
    std::string token;
    Tokeniser tok(msg, ',');

    while(tok.next(token))
    {
        switch(counter)
        {
        case 1: //time
        {
          if (token.compare("")!=0)
          {
            float time = std::stof(token);
            int hms = int(time);

            datetime.millis = time - hms;
            datetime.seconds = fmod(hms, 100);
            hms /= 100;
            datetime.minutes = fmod(hms, 100);
            hms /= 100;
            datetime.hours = hms;

            time_age = millis();
          }         
        }
        break;
        case 2:
        {
          if (token.compare("")!=0)
          {
            if(token[0] == 'A')
                datetime.valid = true;
            if(token[0] == 'V')
                datetime.valid = false;
          }
        }
        break;
        /*
        case 3:
        {
            float llat = std::stof(token);
            int ilat = llat/100;
            double latmins = fmod(llat, 100);
            latitude = ilat + (latmins/60);
        }
        break;
        case 4:
        {
            if(token[0] == 'S')
                latitude = -latitude;
        }
        break;
        case 5:
        {
            float llong = std::stof(token);
            float ilat = llong/100;
            double lonmins = fmod(llong, 100);
            longitude = ilat + (lonmins/60);
        }
        break;
        case 6:
        {
             if(token[0] == 'W')
                longitude = -longitude;
            latlng_age = millis();
        }
        break;
        */
        case 8:
        {
          if (token.compare("")!=0)
          {
            course = std::stof(token);
            course_age = millis();
          }            
        }
        break;
        case 9:
        {
          if (token.compare("")!=0)
          {
            uint32_t date = std::stoi(token);
            datetime.year = fmod(date, 100);
            date /= 100;
            datetime.month = fmod(date, 100);
            datetime.day = date / 100;
            date_age = millis();
          }
        }
        break;
        }
        counter++;
    }
}

void Ublox::read_vtg(const std::string & msg)
{
    int counter = 0;
    Tokeniser tok(msg, ',');
    std::string token;
    while(tok.next(token))
    {
        switch(counter)
        {
        case 1:
        {
          if (token.compare("")!=0)
          {
            course = (std::stof(token)*100);
            course_age = millis();
          }
        }
        break;
        case 5:
        {
          if (token.compare("")!=0)
          {
            knots = (std::stof(token)*100);
            knots_age = millis();
          }
        }
        break;
        case 7:
        {
          if (token.compare("")!=0)
          {
            speed = (std::stof(token)*100);
            speed_age = millis();
          }
        }
        break;
        }
        counter++;
    }
}


bool Ublox::check_checksum(const std::string & msg)
{
    if (msg[msg.size()-5] == '*')
    {
        uint16_t sum = parse_hex(msg[msg.size()-4]) * 16;
        sum += parse_hex(msg[msg.size()-3]);

        for (uint8_t i=1; i < (msg.size()-5); i++)
            sum ^= msg[i];
        if (sum != 0)
            return false;

        return true;
    }
    return false;
}


uint8_t Ublox::parse_hex(char c)
{
    if (c < '0')
        return 0;
    if (c <= '9')
        return c - '0';
    if (c < 'A')
        return 0;
    if (c <= 'F')
        return (c - 'A')+10;
    return 0;
}