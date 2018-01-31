#ifndef _ODOMETRY_MSG_H_
#define _ODOMETRY_MSG_H_

#include "../serial/msg.h"

class OdometryMsg: public serial::Msg
{
public:
    virtual int serialize(uint8_t* outbuffer) const
    {
        DataType_Float x;
        DataType_Float y;
        DataType_Float th;

        x.raw = this->x;
        y.raw = this->y;
        th.raw = this->th;

        *(outbuffer + 0) = x.byte[0];
        *(outbuffer + 1) = x.byte[1];
        *(outbuffer + 2) = x.byte[2];
        *(outbuffer + 3) = x.byte[3];

        *(outbuffer + 4) = y.byte[0];
        *(outbuffer + 5) = y.byte[1];
        *(outbuffer + 6) = y.byte[2];
        *(outbuffer + 7) = y.byte[3];

        *(outbuffer + 8) =  th.byte[0];
        *(outbuffer + 9) =  th.byte[1];
        *(outbuffer + 10) = th.byte[2];
        *(outbuffer + 11) = th.byte[3];

        return 0;
    }

    virtual int deserialize(uint8_t* inbuffer)
    {
        DataType_Float x;
        DataType_Float y;
        DataType_Float th;

        x.byte[0] = *(inbuffer + 0);
        x.byte[1] = *(inbuffer + 1);
        x.byte[2] = *(inbuffer + 2);
        x.byte[3] = *(inbuffer + 3);

        y.byte[0] = *(inbuffer + 4);
        y.byte[1] = *(inbuffer + 5);
        y.byte[2] = *(inbuffer + 6);
        y.byte[3] = *(inbuffer + 7);

        th.byte[0] = *(inbuffer + 8);
        th.byte[1] = *(inbuffer + 9);
        th.byte[2] = *(inbuffer + 10);
        th.byte[3] = *(inbuffer + 11);

        this->x = x.raw;
        this->y = y.raw;
        this->th = th.raw;

        return 0;
    }

    virtual const char* getType() { return ""; }
    virtual const char* getMD5()  { return ""; }
    int   getByte() {return 12;}

    typedef union
    {
        uint8_t byte[4];
        float raw;
    }DataType_Float;

    float x;
    float y;
    float th;
};

#endif
