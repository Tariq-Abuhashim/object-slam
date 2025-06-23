#include "covins_base/msgs/msg_object.hpp"

namespace covins {

MsgObject::MsgObject() {
    //...
}

MsgObject::MsgObject(bool filesave)
    : save_to_file(filesave)
{
    //...
}

MsgObject::MsgObject(MsgTypeVector msgtype)
    : msg_type(msgtype)
{
    //...
}

auto MsgObject::SetMsgType(int msg_size)->void {
    msg_type[0] = msg_size;
    msg_type[1] = (int)is_update_msg;
    msg_type[2] = id.first;
    msg_type[3] = id.second;
    msg_type[4] = 1;
}

auto MsgObject::SetMsgType(MsgTypeVector msgtype)->void {
    msg_type = msgtype;
}

} //end ns
