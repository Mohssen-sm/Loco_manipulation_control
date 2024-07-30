#ifndef CMDPANEL_H
#define CMDPANEL_H

#include <common/enumClass.h>
#include <messages/LowlevelState.h>
#include <pthread.h>

class CmdPanel
{
public:
    CmdPanel() {}
    ~CmdPanel() {}
    UserCommand getUserCmd() { return userCmd; }
    UserValue getUserValue() { return userValue; }
    void setPassive() { userCmd = UserCommand::L2_B; }
    void setZero() { userValue.setZero(); }
    void setCmdNone() { userCmd = UserCommand::NONE; }

protected:
    virtual void *run(void *arg) = 0;
    UserCommand userCmd;
    UserValue userValue;
};

#endif // CMDPANEL_H