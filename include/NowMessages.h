#pragma once

#pragma pack(push, 1)

enum NowMsgType
{
    NOWMSG_CONNECT = 0,
    NOWMSG_CONNECT_RESULT,
    NOWMSG_CLOSE,
    NOWMSG_DATA,
    NOWMSG_KEEPALIVE_REQ,
    NOWMSG_KEEPALIVE_RESP,
};

struct NowMsgHeader
{
    unsigned char m_Type;    //type of message (SMsgType)
};

//connect request
struct NowMsgConnect
{
    NowMsgHeader m_Header;
    unsigned int m_Address;
    unsigned short m_Port;
};

//connect response (positive socket value = success)
struct NowMsgConnectResult
{
    NowMsgHeader m_Header;
    unsigned int m_Address;
    unsigned short m_Port;
    int m_Socket;
};

//close socket
struct NowMsgClose
{
    NowMsgHeader m_Header;
    int m_Socket;
};

//not sure if we want to support accepting sockets...

struct NowMsgData
{
    NowMsgHeader m_Header;
    int m_Socket;
    //data follows
};

//is this socket still being used?
struct NowMsgSocketKeepaliveRequest
{
    NowMsgHeader m_Header;
    int m_Socket;
};

//yes/no - socket is still in use
struct NowMsgSocketKeepaliveResponse
{
    NowMsgHeader m_Header;
    int m_Socket;
    unsigned char m_InUse;
};

#pragma pack(pop)
