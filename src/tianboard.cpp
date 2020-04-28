#include "tianboard.h"
#include "protocol.h"
#include <vector>

void Tianboard::serialDataProc(uint8_t *data, unsigned int data_len)
{
    static uint8_t state = 0;
    uint8_t *p = data;
    static vector<uint8_t> recv_msg;
    static uint32_t len;
    uint32_t j;

    while (data_len != 0)
    {
        switch (state)
        {
        case 0:
            if (*p == (PROTOCOL_HEAD&0xFF))
            {
                recv_msg.clear();
                recv_msg.push_back(PROTOCOL_HEAD&0xFF);
                state = 1;
            }
            p++;
            data_len--;
            break;

        case 1:
            if (*p == ((PROTOCOL_HEAD>>8)&0xFF))
            {
                recv_msg.push_back(((PROTOCOL_HEAD>>8)&0xFF));
                p++;
                data_len--;
                state = 2;
            }
            else
            {
                state = 0;
            }
            break;

        case 2: // len
            recv_msg.push_back(*p);
            len = *p;
            p++;
            data_len--;
            state = 3;
            break;

        case 3: // len
            recv_msg.push_back(*p);
            len += (*p) * 256;
            if (len > 1024 * 10)
            {
                state = 0;
                break;
            }
            p++;
            data_len--;
            state = 4;
            break;

        case 4: // pack_type
            recv_msg.push_back(*p);
            p++;
            data_len--;
            len--;
            state = 5;
            break;

        case 5: // pack_type
            recv_msg.push_back(*p);
            p++;
            data_len--;
            len--;
            state = 6;
            break;

        case 6: //
            if (len--)
            {
                recv_msg.push_back(*p);
                p++;
                data_len--;
            }
            else
            {
                state = 7;
            }
            break;

        case 7:
            {
                int i;
                uint8_t bcc = 0;
                recv_msg.push_back(*p);
                p++;
                data_len--;
                state = 0;
                for (i = 4; i < recv_msg.size(); i++)
                {
                    bcc ^= recv_msg[i];
                }

                if (bcc == 0)
                {
                    tianboardDataProc(&recv_msg[0], recv_msg.size()); // process recv msg
                    // for (i = 0; i < recv_msg.size(); i++)
                    // {
                    //     printf("%02x", recv_msg[i]);
                    // }
                    // printf("\r\n");
                }
                else
                {
                    printf("bcc error len %ld\n", recv_msg.size());
                    // for (i = 0; i < recv_msg.size(); i++)
                    // {
                    //     printf("%02x", recv_msg[i]);
                    // }
                    // printf("\r\n");
                }
                state = 0;
            }
            break;

        default:
            state = 0;
            break;
        }
    }
}

void Tianboard::tianboardDataProc(unsigned char *buf, int len)
{
    struct protocol_pack *p = (struct protocol_pack *)buf;
    switch(p->pack_type)
    {
    case PACK_TYPE_ODOM_RESPONSE:
        if (sizeof(struct odom) == p->len - 2)
        {
            nav_msgs::Odometry odom_msg;
            struct odom *pOdom = (struct odom *)(p->data);
            odom_msg.header.stamp = ros::Time::now();
            odom_msg.header.frame_id = "odom";

            odom_msg.pose.pose.position.x = pOdom->pose.point.x;
            odom_msg.pose.pose.position.y = pOdom->pose.point.y;
            odom_msg.pose.pose.position.z = pOdom->pose.point.z;
            odom_msg.pose.pose.orientation.x = pOdom->pose.quat.x;
            odom_msg.pose.pose.orientation.y = pOdom->pose.quat.y;
            odom_msg.pose.pose.orientation.z = pOdom->pose.quat.z;
            odom_msg.pose.pose.orientation.w = pOdom->pose.quat.w;

            //set the velocity
            odom_msg.child_frame_id = "base_link";
            odom_msg.twist.twist.linear.x = pOdom->twist.linear.x;
            odom_msg.twist.twist.linear.y = pOdom->twist.linear.y;
            odom_msg.twist.twist.linear.z = pOdom->twist.linear.z;
            odom_msg.twist.twist.angular.x = pOdom->twist.angular.x;
            odom_msg.twist.twist.angular.y = pOdom->twist.angular.y;
            odom_msg.twist.twist.angular.z = pOdom->twist.angular.z;
            //publish the message
            odom_pub_.publish(odom_msg);
        }
        break;

    default:
        break;
    }
}

void Tianboard::velocityCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    uint16_t len;
    vector<uint8_t> buf;

    uint8_t bcc = 0;
    struct twist twist;
    int i;
    uint8_t *out = (uint8_t *)&twist;
    twist.linear.x = msg->linear.x;
    twist.linear.y = msg->linear.y;
    twist.linear.z = msg->linear.z;
    twist.angular.x = msg->angular.x;
    twist.angular.y = msg->angular.y;
    twist.angular.z = msg->angular.z;

    buf.push_back(PROTOCOL_HEAD&0xFF);
    buf.push_back((PROTOCOL_HEAD>>8)&0xFF);

    len = sizeof(struct twist) + 2;

    buf.push_back(len & 0xFF);
    buf.push_back((len >> 8) & 0xFF);

    buf.push_back(PACK_TYPE_CMD_VEL & 0xFF);
    buf.push_back((PACK_TYPE_CMD_VEL >> 8) & 0xFF);

    for (i = 0; i < sizeof(struct twist); i++)
    {
        buf.push_back(out[i]);
    }

    for (i = 4; i < buf.size(); i++)
    {
        bcc ^= buf[i];
    }

    buf.push_back(bcc);

    serial_.send(&buf[0], buf.size());
}

Tianboard::Tianboard(ros::NodeHandle *nh):nh_(*nh)
{
    std::string param_serial_port;

    nh_.param<std::string>("serial_port", param_serial_port, DEFAULT_SERIAL_DEVICE);

    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 1);

    cmd_vel_sub_ = nh_.subscribe("cmd_vel", 1, &Tianboard::velocityCallback, this);

    if (serial_.open(param_serial_port.c_str(), 115200, 0, 8, 1, 'N',
                     boost::bind(&Tianboard::serialDataProc, this, _1, _2)) != true)
    {
        exit(-1);
    }
}