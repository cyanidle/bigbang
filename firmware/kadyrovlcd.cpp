#include "kadyrovlcd.h"
#include "global.h"
void reverseArray(uint8_t a[], uint8_t n)
{
    int temp;
    for (int i = 0; i < n / 2; i++)
    {
        temp = a[n - i - 1];
        a[n - i - 1] = a[i];
        a[i] = temp;
    }
}

KadyrovLcd::KadyrovLcd(Address address) :
    lcd(address, 16, 2)
{
}

void KadyrovLcd::setup() {
    lcd.init();
    lcd.backlight();
    lcd.home();
    lcd.clear();
    lcd.print("Prodam garazh");
    lcd.createChar(0,LT);
    lcd.createChar(1,UB);
    lcd.createChar(2,RT);
    lcd.createChar(3,LL);
    lcd.createChar(4,LB);
    lcd.createChar(5,LR);
    lcd.createChar(6,MB);
    lcd.createChar(7,block);
}

void KadyrovLcd::print(int number){
    lcd.clear(); /////входное число
    uint16_t numba = number; //распускается на цифры в ходе подсчета
    uint8_t count = 0; //количество цифр
    uint8_t score[4];
    //////////////подсчет цифр в числе
    while(numba){
        number/=10;
        score[count] = numba - number*10;
        numba = number;
        count++;
    }
    reverseArray(score, count);
    for(uint8_t i = 0; i<count; i++) {
        printDigit(score[i],i*4);
    }
}

void KadyrovLcd::printDigit(int digit, int position){
    switch(digit) {
        case 0: custom0(position); break;
        case 1: custom1(position); break;
        case 2: custom2(position); break;
        case 3: custom3(position); break;
        case 4: custom4(position); break;
        case 5: custom5(position); break;
        case 6: custom6(position); break;
        case 7: custom7(position); break;
        case 8: custom8(position); break;
        case 9: custom9(position); break;
        default: nh.logerror("[Arduino:LCD] Digit out of range!");
    }
}

void KadyrovLcd::custom0(int x)
{ // uses segments to build the number 0
    lcd.setCursor(x,0); // set cursor to column 0, line 0 (first row)
    lcd.write(0);  // call each segment to create
    lcd.write(1);  // top half of the number
    lcd.write(2);
    lcd.setCursor(x, 1); // set cursor to colum 0, line 1 (second row)
    lcd.write(3);  // call each segment to create
    lcd.write(4);  // bottom half of the number
    lcd.write(5);
}
void KadyrovLcd::custom1(int x)
{
    lcd.setCursor(x,0);
    lcd.write(1);
    lcd.write(2);
    lcd.print(" ");
    lcd.setCursor(x,1);
    lcd.write(4);
    lcd.write(7);
    lcd.write(4);
}
void KadyrovLcd::custom2(int x)
{
    lcd.setCursor(x,0);
    lcd.write(6);
    lcd.write(6);
    lcd.write(2);
    lcd.setCursor(x, 1);
    lcd.write(3);
    lcd.write(4);
    lcd.write(4);
}
void KadyrovLcd::custom3(int x)
{
    lcd.setCursor(x,0);
    lcd.write(6);
    lcd.write(6);
    lcd.write(2);
    lcd.setCursor(x, 1);
    lcd.write(4);
    lcd.write(4);
    lcd.write(5);
}
void KadyrovLcd::custom4(int x)
{
    lcd.setCursor(x,0);
    lcd.write(3);
    lcd.write(4);
    lcd.write(7);
    lcd.setCursor(x, 1);
    lcd.print(" ");
    lcd.print(" ");
    lcd.write(7);
}
void KadyrovLcd::custom5(int x)
{
    lcd.setCursor(x,0);
    lcd.write(3);
    lcd.write(6);
    lcd.write(6);
    lcd.setCursor(x, 1);
    lcd.write(4);
    lcd.write(4);
    lcd.write(5);
}
void KadyrovLcd::custom6(int x)
{
    lcd.setCursor(x,0);
    lcd.write(0);
    lcd.write(6);
    lcd.write(6);
    lcd.setCursor(x, 1);
    lcd.write(3);
    lcd.write(4);
    lcd.write(5);
}
void KadyrovLcd::custom7(int x)
{
    lcd.setCursor(x,0);
    lcd.write(1);
    lcd.write(1);
    lcd.write(2);
    lcd.setCursor(x, 1);
    lcd.print(" ");
    lcd.print(" ");
    lcd.write(7);
}
void KadyrovLcd::custom8(int x)
{
    lcd.setCursor(x,0);
    lcd.write(0);
    lcd.write(6);
    lcd.write(2);
    lcd.setCursor(x, 1);
    lcd.write(3);
    lcd.write(4);
    lcd.write(5);
}
void KadyrovLcd::custom9(int x)
{
    lcd.setCursor(x,0);
    lcd.write(0);
    lcd.write(6);
    lcd.write(2);
    lcd.setCursor(x, 1);
    lcd.print(" ");
    lcd.print(" ");
    lcd.write(7);
}
