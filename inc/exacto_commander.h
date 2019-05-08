#ifndef EXACTO_COMMANDER_H_
#define EXACTO_COMMANDER_H_

#include <stdint.h> 

typedef enum{
	ALLSNSRREAD,
	ALLSNSRWAIT
}ExactoSystemStates;

ExactoSystemStates getExactoSysSt(uint8_t adr, uint8_t val);

int checkCmdType(char *cmd, int cmdlen);

int checkSrvType(char * cmd, int cmdlen);

uint8_t getDataCmd(char * cmd, uint8_t * trg, uint8_t * adr, uint8_t * value);
uint8_t charhex2int(char num);

#endif /* EXACTO_COMMANDER_H_ */
