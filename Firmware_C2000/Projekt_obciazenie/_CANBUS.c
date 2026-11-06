#include "DSP28x_Project.h"

// 1. Funkcja konfigurująca magistralę CAN (wywoływana raz na start)
void Init_eCAN_Normal(void)
{
    struct ECAN_REGS ECanaShadow;
    
    InitECana();    
    InitECanGpio();

    EALLOW;
    // Wyłączenie skrzyni 25 na czas konfiguracji
    ECanaShadow.CANME.all = ECanaRegs.CANME.all;
    ECanaShadow.CANME.bit.ME25 = 0; 
    ECanaRegs.CANME.all = ECanaShadow.CANME.all;

    // ID wiadomości
    ECanaMboxes.MBOX25.MSGID.all = 0x95555555;
    
    // Konfiguracja jako nadajnik (Transmit = 0)
    ECanaShadow.CANMD.all = ECanaRegs.CANMD.all;
    ECanaShadow.CANMD.bit.MD25 = 0; 
    ECanaRegs.CANMD.all = ECanaShadow.CANMD.all;

    // długość ramki na 3 bajty (po jednym bajcie na każdy moduł)
    ECanaMboxes.MBOX25.MSGCTRL.bit.DLC = 3; 
    
    // Włączenie skrzynki 25
    ECanaShadow.CANME.all = ECanaRegs.CANME.all;
    ECanaShadow.CANME.bit.ME25 = 1; 
    ECanaRegs.CANME.all = ECanaShadow.CANME.all;

    // =======================================================
    // KONFIGURACJA ODBIORNIKA (MBOX26) - Komendy z komputera
    // =======================================================
    ECanaShadow.CANME.all = ECanaRegs.CANME.all;
    ECanaShadow.CANME.bit.ME26 = 0; // Wyłączenie przed konfiguracją
    ECanaRegs.CANME.all = ECanaShadow.CANME.all;

    // Identyfikator komend z PC (o 1 wyższy niż nadawczy)
    ECanaMboxes.MBOX26.MSGID.all = 0x95555556;
    
    // Ustawienie skrzynki na odbiór (Receive = 1)
    ECanaShadow.CANMD.all = ECanaRegs.CANMD.all;
    ECanaShadow.CANMD.bit.MD26 = 1; 
    ECanaRegs.CANMD.all = ECanaShadow.CANMD.all;
    
    // Włączenie skrzynki odbiorczej
    ECanaShadow.CANME.all = ECanaRegs.CANME.all;
    ECanaShadow.CANME.bit.ME26 = 1; 
    ECanaRegs.CANME.all = ECanaShadow.CANME.all;
    
    EDIS;
}

// 2. Funkcja wysyłająca dane (wywoływana przy każdej zmianie na obciążnicy)
void Send_State_eCAN(int m1, int m2, int m3)
{
    struct ECAN_REGS ECanaShadow;
    
    // Pakujemy stany modułów do rejestru danych MDL (Bajt 0, Bajt 1, Bajt 2)
    Uint32 payload = ((Uint32)m1 << 24) | ((Uint32)m2 << 16) | ((Uint32)m3 << 8);
    
    ECanaMboxes.MBOX25.MDL.all = payload;
    ECanaMboxes.MBOX25.MDH.all = 0x00000000;    
    
    // START TRANSMISJI
    ECanaShadow.CANTRS.all = 0;    
    ECanaShadow.CANTRS.bit.TRS25 = 1;      
    ECanaRegs.CANTRS.all = ECanaShadow.CANTRS.all; 

    // Oczekiwanie na potwierdzenie z TIMEOUTEM
    // (Dzięki temu, jeśli nie podepniesz kabla CAN, obciążnica się nie zawiesi!)
    Uint32 timeout = 0xFFFFF;
    do {
        ECanaShadow.CANTA.all = ECanaRegs.CANTA.all;
        timeout--;
    } while(ECanaShadow.CANTA.bit.TA25 == 0 && timeout > 0);
     
    // Jeśli wysłano poprawnie (nie było timeoutu), kasujemy flagę ACK
    if(timeout != 0) {
        ECanaShadow.CANTA.all = 0;     
        ECanaShadow.CANTA.bit.TA25 = 1;              
        ECanaRegs.CANTA.all = ECanaShadow.CANTA.all;
    }
}