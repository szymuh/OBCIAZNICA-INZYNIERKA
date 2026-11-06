#include "DSP28x_Project.h" 
#include <stdio.h> // dla funkcji sprintf()

// Deklaracje funkcji
void Init_ADC_Sensors(void);
void Read_Sensors(int *t1, int *t2, int *t3, 
                  int *curr_m1, int *volt_m1,
                  int *curr_m2, int *volt_m2,
                  int *curr_m3, int *volt_m3);

// ========================================================================= 
// PARAMETRY I2C I EKSPANDERA LCD (Adres 0x27) 
// ========================================================================= 
#define I2C_ADDR 0x27   

#define LCD_RS 0x01    // P0: RS 
#define LCD_RW 0x02    // P1: R/W 
#define LCD_EN 0x04    // P2: E 
#define LCD_BL 0x08    // P3: Backlight 

// ========================================================================= 
// DEFINICJE MAKRA DLA PRZEKAŹNIKÓW - MODUŁ 1
// ========================================================================= 
#define M1_SPDT1_LOW     GpioDataRegs.GPBCLEAR.bit.GPIO33 = 1 
#define M1_SPDT1_HIGH    GpioDataRegs.GPBSET.bit.GPIO33 = 1 
#define M1_SPDT2_LOW     GpioDataRegs.GPACLEAR.bit.GPIO13 = 1 
#define M1_SPDT2_HIGH    GpioDataRegs.GPASET.bit.GPIO13 = 1 
#define M1_SPST1_LOW     GpioDataRegs.GPBCLEAR.bit.GPIO32 = 1 
#define M1_SPST1_HIGH    GpioDataRegs.GPBSET.bit.GPIO32 = 1 
#define M1_SPST2_LOW     GpioDataRegs.GPACLEAR.bit.GPIO22 = 1 
#define M1_SPST2_HIGH    GpioDataRegs.GPASET.bit.GPIO22 = 1 
#define M1_SPST3_LOW     GpioDataRegs.GPACLEAR.bit.GPIO24 = 1 
#define M1_SPST3_HIGH    GpioDataRegs.GPASET.bit.GPIO24 = 1 
#define M1_GIGAVAC_LOW   GpioDataRegs.GPBCLEAR.bit.GPIO58 = 1 
#define M1_GIGAVAC_HIGH  GpioDataRegs.GPBSET.bit.GPIO58 = 1 

// ========================================================================= 
// DEFINICJE MAKRA DLA PRZEKAŹNIKÓW - MODUŁ 2
// ========================================================================= 
#define M2_SPDT1_LOW     GpioDataRegs.GPACLEAR.bit.GPIO15 = 1 
#define M2_SPDT1_HIGH    GpioDataRegs.GPASET.bit.GPIO15 = 1 
#define M2_SPDT2_LOW     GpioDataRegs.GPACLEAR.bit.GPIO3 = 1 
#define M2_SPDT2_HIGH    GpioDataRegs.GPASET.bit.GPIO3 = 1 
#define M2_SPST1_LOW     GpioDataRegs.GPACLEAR.bit.GPIO1 = 1 
#define M2_SPST1_HIGH    GpioDataRegs.GPASET.bit.GPIO1 = 1 
#define M2_SPST2_LOW     GpioDataRegs.GPBCLEAR.bit.GPIO56 = 1 
#define M2_SPST2_HIGH    GpioDataRegs.GPBSET.bit.GPIO56 = 1 
#define M2_SPST3_LOW     GpioDataRegs.GPACLEAR.bit.GPIO2 = 1 
#define M2_SPST3_HIGH    GpioDataRegs.GPASET.bit.GPIO2 = 1 
#define M2_GIGAVAC_LOW   GpioDataRegs.GPACLEAR.bit.GPIO0 = 1 
#define M2_GIGAVAC_HIGH  GpioDataRegs.GPASET.bit.GPIO0 = 1 

// ========================================================================= 
// DEFINICJE MAKRA DLA PRZEKAŹNIKÓW - MODUŁ 3
// ========================================================================= 
#define M3_SPDT1_LOW     GpioDataRegs.GPACLEAR.bit.GPIO27 = 1 
#define M3_SPDT1_HIGH    GpioDataRegs.GPASET.bit.GPIO27 = 1 
#define M3_SPDT2_LOW     GpioDataRegs.GPACLEAR.bit.GPIO11 = 1 
#define M3_SPDT2_HIGH    GpioDataRegs.GPASET.bit.GPIO11 = 1 
#define M3_SPST1_LOW     GpioDataRegs.GPBCLEAR.bit.GPIO41 = 1 
#define M3_SPST1_HIGH    GpioDataRegs.GPBSET.bit.GPIO41 = 1 
#define M3_SPST2_LOW     GpioDataRegs.GPBCLEAR.bit.GPIO55 = 1 
#define M3_SPST2_HIGH    GpioDataRegs.GPBSET.bit.GPIO55 = 1 
#define M3_SPST3_LOW     GpioDataRegs.GPACLEAR.bit.GPIO10 = 1 
#define M3_SPST3_HIGH    GpioDataRegs.GPASET.bit.GPIO10 = 1 
#define M3_GIGAVAC_LOW   GpioDataRegs.GPBCLEAR.bit.GPIO40 = 1 
#define M3_GIGAVAC_HIGH  GpioDataRegs.GPBSET.bit.GPIO40 = 1 

// ========================================================================= 
// DEFINICJE WEJŚĆ (Przełączniki 3-pozycyjne)
// ========================================================================= 
#define SW_M1_PIN_A  GpioDataRegs.GPBDAT.bit.GPIO51  
#define SW_M1_PIN_B  GpioDataRegs.GPADAT.bit.GPIO18   

#define SW_M2_PIN_A  GpioDataRegs.GPADAT.bit.GPIO16  
#define SW_M2_PIN_B  GpioDataRegs.GPADAT.bit.GPIO7   

#define SW_M3_PIN_A  GpioDataRegs.GPBDAT.bit.GPIO39  
#define SW_M3_PIN_B  GpioDataRegs.GPBDAT.bit.GPIO54   

// ========================================================================= 
// DEKLARACJE FUNKCJI I2C & LCD & CAN
// ========================================================================= 
void I2CA_Init(void); 
Uint16 I2C_WriteTCA(Uint16 reg, Uint16 data); 
void LCD_WriteNibble(Uint16 nibble, Uint16 rs_flag); 
void LCD_WriteByte(Uint16 data, Uint16 rs_flag); 
void LCD_Init(void); 
void LCD_Print(char *str); 

extern void Init_eCAN_Normal(void);
extern void Send_State_eCAN(int m1, int m2, int m3);
extern void InitAdc(void); 

void main(void) 
{ 
    // 1. Inicjalizacja zegarów i pinów GPIO 
    InitSysCtrl(); 
    EALLOW; 
     
    // --- KONFIGURACJA WEJŚĆ (DIR = 0) --- 
    GpioCtrlRegs.GPBDIR.bit.GPIO51 = 0; // M1_A
    GpioCtrlRegs.GPADIR.bit.GPIO18 = 0; // M1_B
    GpioCtrlRegs.GPADIR.bit.GPIO16 = 0; // M2_A
    GpioCtrlRegs.GPADIR.bit.GPIO7 = 0;  // M2_B
    GpioCtrlRegs.GPBDIR.bit.GPIO39 = 0; // M3_A
    GpioCtrlRegs.GPBDIR.bit.GPIO54 = 0; // M3_B
     
    // --- KONFIGURACJA PULL-UP (Włączone = 0) --- 
    GpioCtrlRegs.GPBPUD.bit.GPIO51 = 0; 
    GpioCtrlRegs.GPAPUD.bit.GPIO18 = 0; 
    GpioCtrlRegs.GPAPUD.bit.GPIO16 = 0; 
    GpioCtrlRegs.GPAPUD.bit.GPIO7 = 0; 
    GpioCtrlRegs.GPBPUD.bit.GPIO39 = 0; 
    GpioCtrlRegs.GPBPUD.bit.GPIO54 = 0; 

    // --- STAN BEZPIECZNY WYJŚĆ --- 
    M1_SPDT1_LOW; M1_SPST1_LOW; M1_SPST2_LOW; M1_SPST3_LOW; M1_SPDT2_LOW; M1_GIGAVAC_LOW; 
    M2_SPDT1_LOW; M2_SPST1_LOW; M2_SPST2_LOW; M2_SPST3_LOW; M2_SPDT2_LOW; M2_GIGAVAC_LOW; 
    M3_SPDT1_LOW; M3_SPST1_LOW; M3_SPST2_LOW; M3_SPST3_LOW; M3_SPDT2_LOW; M3_GIGAVAC_LOW; 
     
    // --- KONFIGURACJA WYJŚĆ (DIR = 1) --- 
    GpioCtrlRegs.GPBDIR.bit.GPIO33 = 1; GpioCtrlRegs.GPADIR.bit.GPIO13 = 1;   
    GpioCtrlRegs.GPBDIR.bit.GPIO32 = 1; GpioCtrlRegs.GPADIR.bit.GPIO22 = 1;   
    GpioCtrlRegs.GPADIR.bit.GPIO24 = 1; GpioCtrlRegs.GPBDIR.bit.GPIO58 = 1;   
    GpioCtrlRegs.GPADIR.bit.GPIO15 = 1; GpioCtrlRegs.GPADIR.bit.GPIO3 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO1 = 1;  GpioCtrlRegs.GPBDIR.bit.GPIO56 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO2 = 1;  GpioCtrlRegs.GPADIR.bit.GPIO0 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO27 = 1; GpioCtrlRegs.GPADIR.bit.GPIO11 = 1;
    GpioCtrlRegs.GPBDIR.bit.GPIO41 = 1; GpioCtrlRegs.GPBDIR.bit.GPIO55 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO10 = 1; GpioCtrlRegs.GPBDIR.bit.GPIO40 = 1;
    EDIS; 

    // 2. Inicjalizacja magistrali I2C, wyświetlacza, CAN oraz przetwornika ADC
    I2CA_Init(); 
    Init_eCAN_Normal(); 
    Init_ADC_Sensors(); // Ujednolicona nazwa
     
    I2C_WriteTCA(0x03, 0x00); 
    I2C_WriteTCA(0x01, 0x00); 
    DELAY_US(100000);  
    LCD_Init(); 
    LCD_WriteByte(0x01, 0); 
    DELAY_US(3000); 
    LCD_WriteByte(0x80, 0); 
    LCD_Print("M: R, T, I, U RECZNY"); 

    // 3. Zmienne stanu procesu
    int state_m1 = 0, state_m2 = 0, state_m3 = 0; 
    int target_m1 = 0, target_m2 = 0, target_m3 = 0; 
    
    int auto_mode = 0; 
    int can_update_needed = 0; 
    int rx_m1 = 0, rx_m2 = 0, rx_m3 = 0;

// Zmienne do temperatur, pomiarów i bufora ekranu
    int temp_m1 = 0, temp_m2 = 0, temp_m3 = 0;
    int curr_m1 = 0, volt_m1 = 0;
    int curr_m2 = 0, volt_m2 = 0;
    int curr_m3 = 0, volt_m3 = 0;
    char lcd_buf[21];

    // 4. Pętla główna automatu stanów 
    while(1) 
    { 
        can_update_needed = 0; 

        // ========================================================== 
        // ODBIÓR KOMEND CAN
        // ==========================================================
        if (ECanaRegs.CANRMP.bit.RMP26 == 1) 
        {
            Uint32 received_data = ECanaMboxes.MBOX26.MDL.all;
            rx_m1 = (received_data >> 24) & 0xFF; 
            rx_m2 = (received_data >> 16) & 0xFF; 
            rx_m3 = (received_data >> 8) & 0xFF;  
            ECanaRegs.CANRMP.bit.RMP26 = 1; 

            if (rx_m1 == 0xFF) {
                auto_mode = 1;
            } else if (rx_m1 == 0x00) {
                auto_mode = 0;
            } else if (auto_mode == 1) {
                if(rx_m1 >= 1 && rx_m1 <= 3) target_m1 = rx_m1;
                if(rx_m2 >= 1 && rx_m2 <= 3) target_m2 = rx_m2;
                if(rx_m3 >= 1 && rx_m3 <= 3) target_m3 = rx_m3;
            }
        }

        // ========================================================== 
        // ODCZYT HEBELKÓW (TRYB AUTO WYŁĄCZONY)
        // ==========================================================
        if (auto_mode == 0) 
        {
            if (SW_M1_PIN_A == 1 && SW_M1_PIN_B == 1) target_m1 = 1;
            else if (SW_M1_PIN_A == 0 && SW_M1_PIN_B == 1) target_m1 = 2;
            else if (SW_M1_PIN_A == 0 && SW_M1_PIN_B == 0) target_m1 = 3;

            if (SW_M2_PIN_A == 1 && SW_M2_PIN_B == 1) target_m2 = 1;
            else if (SW_M2_PIN_A == 0 && SW_M2_PIN_B == 1) target_m2 = 2;
            else if (SW_M2_PIN_A == 0 && SW_M2_PIN_B == 0) target_m2 = 3;

            if (SW_M3_PIN_A == 1 && SW_M3_PIN_B == 1) target_m3 = 1;
            else if (SW_M3_PIN_A == 0 && SW_M3_PIN_B == 1) target_m3 = 2;
            else if (SW_M3_PIN_A == 0 && SW_M3_PIN_B == 0) target_m3 = 3;
        }

        // ========================================================== 
        // REALIZACJA WYJŚĆ (Sterowanie przekaźnikami)
        // ==========================================================
        if (target_m1 != state_m1) {
            state_m1 = target_m1;
            can_update_needed = 1;
            if (state_m1 == 1) {
                M1_SPDT1_HIGH; M1_SPST1_LOW; M1_SPST2_LOW; M1_SPST3_LOW; M1_SPDT2_LOW; 
            } else if (state_m1 == 2) {
                M1_SPDT1_LOW; M1_SPST1_HIGH; M1_SPST2_LOW; M1_SPST3_LOW; M1_SPDT2_LOW; 
            } else if (state_m1 == 3) {
                M1_SPDT1_HIGH; M1_SPST1_HIGH; M1_SPST2_HIGH; M1_SPST3_HIGH; M1_SPDT2_HIGH; 
            }
        }

        if (target_m2 != state_m2) {
            state_m2 = target_m2;
            can_update_needed = 1;
            if (state_m2 == 1) {
                M2_SPDT1_HIGH; M2_SPST1_LOW; M2_SPST2_LOW; M2_SPST3_LOW; M2_SPDT2_LOW; 
            } else if (state_m2 == 2) {
                M2_SPDT1_LOW; M2_SPST1_HIGH; M2_SPST2_LOW; M2_SPST3_LOW; M2_SPDT2_LOW; 
            } else if (state_m2 == 3) {
                M2_SPDT1_HIGH; M2_SPST1_HIGH; M2_SPST2_HIGH; M2_SPST3_HIGH; M2_SPDT2_HIGH; 
            }
        }

        if (target_m3 != state_m3) {
            state_m3 = target_m3;
            can_update_needed = 1;
            if (state_m3 == 1) {
                M3_SPDT1_HIGH; M3_SPST1_LOW; M3_SPST2_LOW; M3_SPST3_LOW; M3_SPDT2_LOW; 
            } else if (state_m3 == 2) {
                M3_SPDT1_LOW; M3_SPST1_HIGH; M3_SPST2_LOW; M3_SPST3_LOW; M3_SPDT2_LOW; 
            } else if (state_m3 == 3) {
                M3_SPDT1_HIGH; M3_SPST1_HIGH; M3_SPST2_HIGH; M3_SPST3_HIGH; M3_SPDT2_HIGH; 
            }
        }

        // ========================================================== 
        // RAPORTOWANIE ZMIAN PO LINII CAN
        // ==========================================================
        if(can_update_needed == 1)
        {
            Send_State_eCAN(state_m1, state_m2, state_m3);
            DELAY_US(10000); 
        }

        // ========================================================== 
        // POMIAR SENSORÓW I AKTUALIZACJA WYŚWIETLACZA LCD
        // ==========================================================
Read_Sensors(&temp_m1, &temp_m2, &temp_m3, 
                 &curr_m1, &volt_m1, 
                 &curr_m2, &volt_m2, 
                 &curr_m3, &volt_m3);

        char *o1 = (state_m1==1) ? "5.5" : ((state_m1==2) ? "22 " : "88 ");
        char *o2 = (state_m2==1) ? "5.5" : ((state_m2==2) ? "22 " : "88 ");
        char *o3 = (state_m3==1) ? "5.5" : ((state_m3==2) ? "22 " : "88 ");

        // Linia 1: Tryb pracy
        LCD_WriteByte(0x80, 0); 
        if (auto_mode == 1) LCD_Print("M: R, T, I, U ZDALNY");
        else LCD_Print("M: R, T, I, U RECZNY");
        //Legenda wyświetlania: "Moduł: Rezystancja [Ohm], Temperatura [], Prąd [], Napięcie [mV]"

        // Linia 2: Moduł 1
        LCD_WriteByte(0xC0, 0); 
        sprintf(lcd_buf, "1:%-s %d %d %d", o1, temp_m1,curr_m1,volt_m1);
        LCD_Print(lcd_buf);

        // Linia 3: Moduł 2
        LCD_WriteByte(0x94, 0); 
        sprintf(lcd_buf, "2:%-s %d %d %d", o2, temp_m2,curr_m2,volt_m3);
        LCD_Print(lcd_buf);

        // Linia 4: Moduł 3
        LCD_WriteByte(0xD4, 0); 
        sprintf(lcd_buf, "3:%-s %d %d %d", o3, temp_m3,curr_m3,volt_m3);
        LCD_Print(lcd_buf);

        DELAY_US(250000);  
    } 
}

// ========================================================================= 
// IMPLEMENTACJA FUNKCJI I2C & LCD 
// ========================================================================= 

void I2CA_Init(void) 
{ 
    EALLOW; 
    SysCtrlRegs.PCLKCR0.bit.I2CAENCLK = 1;  

    GpioCtrlRegs.GPAPUD.bit.GPIO28 = 0;     
    GpioCtrlRegs.GPAPUD.bit.GPIO29 = 0;     
    GpioCtrlRegs.GPAQSEL2.bit.GPIO28 = 3;   
    GpioCtrlRegs.GPAQSEL2.bit.GPIO29 = 3;   
    GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 2;    
    GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 2;    
    EDIS; 

    I2caRegs.I2CMDR.all = 0x0000; 
    I2caRegs.I2CPSC.all = 8;         
    I2caRegs.I2CCLKL = 45;           
    I2caRegs.I2CCLKH = 45; 
    I2caRegs.I2CMDR.all = 0x0020;   
} 

Uint16 I2C_WriteTCA(Uint16 reg, Uint16 data) 
{ 
    Uint32 timeout = 100000; 

    while(I2caRegs.I2CMDR.bit.STP == 1 && timeout > 0) timeout--; 
    if (timeout == 0) return 0; 

    I2caRegs.I2CSAR = I2C_ADDR;             
    I2caRegs.I2CCNT = 2;                    
    I2caRegs.I2CDXR = reg;                  
    I2caRegs.I2CMDR.all = 0x6E20;           

    timeout = 100000; 
    while(I2caRegs.I2CSTR.bit.XRDY == 0 && timeout > 0) timeout--; 
    if (timeout == 0) return 0; 

    I2caRegs.I2CDXR = data;                 
    DELAY_US(200); 
    return 1; 
} 

void LCD_WriteNibble(Uint16 nibble, Uint16 rs_flag) 
{ 
    Uint16 payload = (nibble & 0xF0) | LCD_BL | (rs_flag ? LCD_RS : 0); 

    I2C_WriteTCA(0x01, payload);          
    I2C_WriteTCA(0x01, payload | LCD_EN);  
    DELAY_US(500); 
    I2C_WriteTCA(0x01, payload);          
    DELAY_US(500); 
} 

void LCD_WriteByte(Uint16 data, Uint16 rs_flag) 
{ 
    LCD_WriteNibble(data & 0xF0, rs_flag);           
    LCD_WriteNibble((data << 4) & 0xF0, rs_flag);    
} 

void LCD_Init(void) 
{ 
    DELAY_US(50000);  

    LCD_WriteNibble(0x30, 0); 
    DELAY_US(10000); 
    LCD_WriteNibble(0x30, 0); 
    DELAY_US(2000); 
    LCD_WriteNibble(0x30, 0); 
    DELAY_US(2000); 
    LCD_WriteNibble(0x20, 0);  
    DELAY_US(2000); 

    LCD_WriteByte(0x28, 0);  
    DELAY_US(2000); 
    LCD_WriteByte(0x0C, 0);  
    DELAY_US(2000); 
    LCD_WriteByte(0x01, 0);  
    DELAY_US(10000); 
    LCD_WriteByte(0x06, 0);  
    DELAY_US(2000); 
} 

void LCD_Print(char *str) 
{ 
    while(*str) 
    { 
        LCD_WriteByte(*str++, 1); 
    } 
}

void Init_ADC_Sensors(void)
{
    InitAdc(); 

    EALLOW;
    // --- MODUŁ 1 ---
    AdcRegs.ADCSOC0CTL.bit.CHSEL = 2;   // NTC M1 (ADCINA2)
    AdcRegs.ADCSOC0CTL.bit.ACQPS = 6;  
    AdcRegs.ADCSOC0CTL.bit.TRIGSEL = 0; 
    AdcRegs.ADCSOC3CTL.bit.CHSEL = 0;   // Prąd M1 (ADCINA0)
    AdcRegs.ADCSOC3CTL.bit.ACQPS = 6;
    AdcRegs.ADCSOC3CTL.bit.TRIGSEL = 0;
    AdcRegs.ADCSOC4CTL.bit.CHSEL = 1;   // Napięcie M1 (ADCINA1)
    AdcRegs.ADCSOC4CTL.bit.ACQPS = 6;
    AdcRegs.ADCSOC4CTL.bit.TRIGSEL = 0;

    // --- MODUŁ 2 ---
    AdcRegs.ADCSOC1CTL.bit.CHSEL = 8;   // NTC M2 (ADCINB0)
    AdcRegs.ADCSOC1CTL.bit.ACQPS = 6;
    AdcRegs.ADCSOC1CTL.bit.TRIGSEL = 0;
    AdcRegs.ADCSOC5CTL.bit.CHSEL = 10;  // Prąd M2 (ADCINB2)
    AdcRegs.ADCSOC5CTL.bit.ACQPS = 6;
    AdcRegs.ADCSOC5CTL.bit.TRIGSEL = 0;
    AdcRegs.ADCSOC6CTL.bit.CHSEL = 9;   // Napięcie M2 (ADCINB1)
    AdcRegs.ADCSOC6CTL.bit.ACQPS = 6;
    AdcRegs.ADCSOC6CTL.bit.TRIGSEL = 0;

    // --- MODUŁ 3 ---
    AdcRegs.ADCSOC2CTL.bit.CHSEL = 11;  // NTC M3 (ADCINB3)
    AdcRegs.ADCSOC2CTL.bit.ACQPS = 6;
    AdcRegs.ADCSOC2CTL.bit.TRIGSEL = 0;
    AdcRegs.ADCSOC7CTL.bit.CHSEL = 13;  // Prąd M3 (ADCINB5)
    AdcRegs.ADCSOC7CTL.bit.ACQPS = 6;
    AdcRegs.ADCSOC7CTL.bit.TRIGSEL = 0;
    AdcRegs.ADCSOC8CTL.bit.CHSEL = 12;  // Napięcie M3 (ADCINB4)
    AdcRegs.ADCSOC8CTL.bit.ACQPS = 6;
    AdcRegs.ADCSOC8CTL.bit.TRIGSEL = 0;
    EDIS;
}

void Read_Sensors(int *t1, int *t2, int *t3, 
                  int *curr_m1, int *volt_m1,
                  int *curr_m2, int *volt_m2,
                  int *curr_m3, int *volt_m3)
{
    // Wyzwolenie SOC 0 do 8
    AdcRegs.ADCSOCFRC1.all = 0x01FF; 
    DELAY_US(30);




    *t1 = AdcResult.ADCRESULT0;    
    *curr_m1 = AdcResult.ADCRESULT3; 
    *volt_m1 = AdcResult.ADCRESULT4;
    //*volt_m1 = (int)(((long)AdcResult.ADCRESULT4 * 3300L) / 4095L) - 1500;

    *t2 = AdcResult.ADCRESULT1;
    *curr_m2 = AdcResult.ADCRESULT5; 
    *volt_m2 = AdcResult.ADCRESULT6;
    //*volt_m2 = (int)(((long)AdcResult.ADCRESULT6 * 3300L) / 4095L) - 1500;

    *t3 = AdcResult.ADCRESULT2;
    *curr_m3 = AdcResult.ADCRESULT7; 
    *volt_m3 = AdcResult.ADCRESULT8;
    //*volt_m3 = (int)(((long)AdcResult.ADCRESULT8 * 3300L) / 4095L) - 1500;

}