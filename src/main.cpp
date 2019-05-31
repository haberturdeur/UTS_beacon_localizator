#include <Arduino.h>
#include <Wire.h>

#include <util/delay.h>

#define TRIG_PIN (1<<2)
#define SYNC_PIN (1<<3)

#define ECHO_D_MASK 0xFC
#define ECHO_B_MASK 0x03

#define ECHOS ((PIND & ECHO_D_MASK) | (PINB & ECHO_B_MASK))

#define ECHO_TIMEOUT   500 // us
#define MEAS_TIMEOUT 50000 // us

#define SLAVE_ADDRESS 0x01

uint32_t time() { return micros(); }

//char buf[8];
// volatile uint8_t cs[8] = {0x80,0,0,0x81,0x82,0x83,0x84,0x85};
volatile uint8_t cs[8] = {1,2,3,4,5,6,7,8};


//void receiveData(int byteCount);
void sendData();

void setup() {
    Serial.begin(115200);
    DDRB  |= TRIG_PIN;
    PORTB |= SYNC_PIN;
    PORTD |= ECHO_D_MASK;
    PORTB |= ECHO_B_MASK;
    Serial.println("Locator");
    Wire.begin(SLAVE_ADDRESS);
    // Wire.onReceive(receiveData);
    Wire.onRequest(sendData);
    Serial.println("Ready!");
    pinMode(A4, INPUT_PULLUP);
    pinMode(A5, INPUT_PULLUP);
    delay(1000);
}

const char* to_hex_str(uint8_t v) {
    static char res[3] = { 0, 0, 0 };
    static const char c[] = "0123456789ABCDEF";
    res[0] = c[v >> 4]; 
    res[1] = c[v & 0x0F];
    return res;
}

uint8_t zeropos(uint8_t v) {
    for(uint8_t i = 0; i != 8; ++i)
        if ((v & (1<<i)) == 0)
            return i;
    return 8;
}

void loop() {
    while ((PINB & SYNC_PIN) == 0) {}
    Serial.println("waiting...");
    while ((PINB & SYNC_PIN) != 0) {}
    PORTB |= TRIG_PIN;
    _delay_us(10);
    PORTB &= ~TRIG_PIN;
    const uint32_t t_start = time();
    uint8_t echos = ECHOS;
    while (ECHOS != 0xFF) {
        if ((time() - t_start) > ECHO_TIMEOUT) {
            Serial.print("Echo start timeout, read 0x");
            Serial.println(to_hex_str(echos));
            return;
        }
        echos = ECHOS;
    }
    uint32_t rect[8];
    uint8_t rece[8];
    int8_t i = 0;
    for (; i != 8; ++i) {
        uint8_t last_echos = echos;
        if (echos == 0)
            break;
        uint32_t t = t_start;
        while(echos == last_echos) {
            if ((t - t_start) > MEAS_TIMEOUT) {
                Serial.print("Echo rec timeout, read 0x");
                Serial.println(to_hex_str(echos));
                i = -i;
                break;
            }
            t = time();
            echos = ECHOS;
        }
        if (i < 0) {
            i = -i;
            break;
        }
        rect[i] = t - t_start;
        rece[i] = echos;
    }

    uint8_t old_mask = 0;
    // cs[2]=zeropos(rece[0] | old_mask);

    Serial.print("Received ");
    Serial.print(i);
    Serial.println(" changes:");
    for (int8_t j = 0; j != i; ++j) {
        uint8_t reci = zeropos(rece[j] | old_mask);
        old_mask |= (1<<reci);
        rece[j] = reci;
        Serial.print('\t');
        Serial.print(reci);
        Serial.print(": ");
        Serial.print(340*(rect[j]/1e6));
        Serial.println(" m");
    }
    return;
    float dist = (340*(rect[0]/1e6))*100;
    if(dist>=128){
        cs[1]=127;
    }else{
        cs[1] = dist;
    }
    cs[2]=rece[0];

}

// void receiveData(int byteCount ){
//     while(Wire.available()>0) 
//     {
//       val=Wire.read();
//       Serial.print((char)val);
//       flag=1;
//     }

// }
void sendData(){
    Wire.write(const_cast<uint8_t*>(cs),8);

}