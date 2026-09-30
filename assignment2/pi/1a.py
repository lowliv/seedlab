# Name: Vlad Kazakin
# Title: Assignment 2 1a CV
# Purpose: send a string of characters to arduino to print them and ascii out

# Import necessary modules
from smbus2 import SMBus
from time import sleep

i2c = SMBus(1)

while True:
    string = input("Enter a string to send: ")

    # Converts string to ascii of each character
    command = [ord(character) for character in string]
    
    # Tries writing to arduino
    try:
        i2c.write_i2c_block_data(
            8,
            0,
            command
        )

        print("Sent:", string)
