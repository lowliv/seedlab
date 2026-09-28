// Name: Vlad Kazakin
// Title: Assignment 2 1a
// Purpose: print received string from pi and convert to ascii

#include <Wire.h>

// Define address for i2c
#define MY_ADDR 8

char receivedMessage[32];
volatile uint8_t messageLength = 0;
volatile bool newMessage = false;


void setup()
{
    Serial.begin(115200);

    // Start i2c
    Wire.begin(MY_ADDR);

    // Call function when receiving data
    Wire.onReceive(receiveEvent);

    Serial.println("Arduino ready.");
}


void loop()
{
    if (newMessage)
    {
        char message[32];
        uint8_t length;

        noInterrupts();

        length = messageLength;

        for (int i = 0; i < length; i++)
        {
            message[i] = receivedMessage[i];
        }

        newMessage = false;

        interrupts();


        // Prints original string
        Serial.print("Characters: ");

        for (int i = 0; i < length; i++)
        {
            Serial.print(message[i]);
        }

        Serial.println();


        // Prints ascii of string
        Serial.print("ASCII: ");

        for (int i = 0; i < length; i++)
        {
            Serial.print((int)message[i]);

            if (i < length - 1)
            {
                Serial.print(" ");
            }
        }

        Serial.println();
        Serial.println();
    }
}


// Called automatically when data comes 
void receiveEvent(int numberOfBytes)
{
    messageLength = 0;

    while (Wire.available() && messageLength < 32)
    {
        receivedMessage[messageLength] = Wire.read();
        messageLength++;
    }

    newMessage = true;
}
