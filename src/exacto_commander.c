#include "exacto_commander.h"

const char chartable[16U] =  {'0', '1', '2', '3', '4', '5', '6', '7','8','9','a','b','c','d','e','f'};
char HelpString[] = "cmd in hex:[0w0000]\nfirst -- trg sensor\n01--stm\n02--lsm303\n03--bmp280\n04--ism330\n[w/r]write/read and address\n[v] -- hex value";
//                              012345678

int checkCmdType(char *cmd, int cmdlen)
{
    //[00w00v00]
    //0123456789
    const int trgcmdlen = 7;
    int result = -1;
    if(trgcmdlen > cmdlen)
    {
        return -1;
    }
    for (uint8_t i = 0; i < (cmdlen - trgcmdlen); i++)
    {
        if((cmd[i] == '[')&&((cmd[i+2] == 'w')||(cmd[i+2] == 'r'))&&(cmd[i+7] == ']'))
        {
            result = i;
            break;
        }
    }
    return result;
}
int checkSrvType(char * cmd, int cmdlen)
{
    const int trgcmdlen = 10;
    int result = -1;
    if(trgcmdlen > cmdlen)
    {
        return -1;
    }
    for (uint8_t i = 0; i < (cmdlen - trgcmdlen); i++)
    {
        if((cmd[i] == 'h')&&(cmd[i+3] == 'e')&&(cmd[i+6] == 'l')&&(cmd[i+9] == 'p'))
        {
            result = i;
            break;
        }
    }
    return result;
}

uint8_t getDataCmd(char * cmd, uint8_t * trg, uint8_t * adr, uint8_t * value)
{
    if((cmd[0] == '[')&&((cmd[2] == 'w')||(cmd[2] == 'r'))&&(cmd[7] == ']'))
    {
        return 0;
    }
       *trg = charhex2int(cmd[1]);
//    *trg  = charhex2int(cmd[1])*16 + charhex2int(cmd[2]);
//    *adr  = charhex2int(cmd[4])*16 + charhex2int(cmd[5]);
//    *value  = charhex2int(cmd[7])*16 + charhex2int(cmd[8]);
    uint8_t val = charhex2int(cmd[3]);
    if(val == 16)
        return 0;
    else
        *adr = val*16;
    val = charhex2int(cmd[4]);
    if(val == 16)
        return 0;
    else
        *adr += val;
    //
    val = charhex2int(cmd[5]);
    if(val == 16)
        return 0;
    else
        *value = val*16;
    val = charhex2int(cmd[6]);
    if(val == 16)
        return 0;
    else
        *value += val;
    if(cmd[3] == 'r')
        return 1;
    else
        return 2;
}
uint8_t charhex2int(char num)
{
    for (uint8_t i = 0; i < 16 ; i++)
    {
        if(chartable[i] == num)
        {
            return i;
        }
    }
    return 16;
}
