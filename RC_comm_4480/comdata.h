

extern   int   SlaveTx(void);
extern   void  Req_Logic_Adr(unsigned char Absadr,unsigned char Logadr,unsigned char cmd);
//extern   void  MasterRetKeySave(void);
extern   void  KeyRetKeySave(void);
extern   void  ShiftData(void);
//extern   unsigned char UpKeyLoad(unsigned char id);
//extern   unsigned char DnKeyLoad(unsigned char id);
extern   int UpKeyLoad(unsigned char id);
extern   int DnKeyLoad(unsigned char id);
extern   unsigned char DnKeyOkCheck(unsigned char id);
extern   unsigned char KeyCancleCheck(unsigned char id);
