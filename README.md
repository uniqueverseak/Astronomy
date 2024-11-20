# Astronomy

dancing light -------------------------------------------------
Import Rpi.GPIO as GPIO
Import time
L1 = 2
L2 = 3
L3 = 4
L4 = 17
GPIO.setmode(GPIO.BCM)
GPIO.setup(L1, GPIO.OUT)
GPIO.setup(L2, GPIO.OUT)
GPIO.setup(L3, GPIO.OUT)
GPIO.setup(L4, GPIO.OUT)
GPIO.output(L1, GPIO.HIGH)
GPIO.output(L2, GPIO.HIGH)
GPIO.output(L3, GPIO.HIGH)
GPIO.output(L4, GPIO.HIGH)
while True :
GPIO.output(L1, GPIO.LOW)
time.sleep(1)
GPIO.output(L1, GPIO.HIGH)
GPIO.output(L2, GPIO.LOW)
time.sleep(1)
GPIO.output(L2, GPIO.HIGH)
GPIO.output(L3, GPIO.LOW)
time.sleep(1)
GPIO.output(L3, GPIO.HIGH)
GPIO.output(L4, GPIO.LOW)
time.sleep(1)
GPIO.output(L4, GPIO.HIGH)
GPIO.output(L1, GPIO.LOW)
time.sleep(1)
GPIO.output(L1, GPIO.HIGH)
GPIO.output(L2, GPIO.LOW)
time.sleep(1)
GPIO.output(L2, GPIO.HIGH)
for i in range(4):
GPIO.output(L1, GPIO.LOW)
time.sleep(0.1)
GPIO.output(L1, GPIO.HIGH)
GPIO.output(L2, GPIO.LOW)
time.sleep(0.1)
GPIO.output(L2, GPIO.HIGH)
GPIO.output(L3, GPIO.LOW)
time.sleep(0.1)
GPIO.output(L3, GPIO.HIGH)
GPIO.output(L4, GPIO.LOW)
time.sleep(0.1)
GPIO.output(L4, GPIO.HIGH)
for i in range(4):
GPIO.output(L4, GPIO.LOW)
time.sleep(0.1)
GPIO.output(L4, GPIO.HIGH)
GPIO.output(L3, GPIO.LOW)
time.sleep(0.1)
GPIO.output(L3, GPIO.HIGH)
GPIO.output(L2, GPIO.LOW)
time.sleep(0.1)
GPIO.output(L2, GPIO.HIGH)
GPIO.output(L1, GPIO. LOW)
time.sleep(0.1)
GPIO.output(L1, GPIO.HIGH)
for i in range(4):
GPIO.output(L4, GPIO.LOW
GPIO.output(L2, GPIO.LOW)
time.sleep(0.25)
GPIO.output(L4, GPIO.HIGH)
GPIO.output(L2, GPIO.HIGH)
GPIO.output(L1, GPIO.LOW
GPIO.output(L3, GPIO.LOW)
Time.sleep(0.25)
GPIO.output(L1, GPIO.HIGH)
GPIO.output(L3, GPIO.HIGH)
for i in range(4):
GPIO.output(L4, GPIO.LOW
GPIO.output(L2, GPIO.LOW)
GPIO.output(L1, GPIO.LOW)
GPIO.output(L3, GPIO.LOW)
time.sleep(0.25)
GPIO.output(L4, GPIO.HIGH)
GPIO.output(L2, GPIO.HIGH)
GPIO.output(L1, GPIO.HIGH)
GPIO.output(L3, GPIO.HIGH)
time.sleep(0.25)
GPIO.cleanup()

# Import necessary libraries
import RPi.GPIO as GPIO  # Library to control GPIO pins on Raspberry Pi
import time  # Library to handle delays

# Define GPIO pins for LEDs
L1 = 2
L2 = 3
L3 = 4
L4 = 17

# GPIO setup
GPIO.setmode(GPIO.BCM)  # Use Broadcom pin numbering
GPIO.setup(L1, GPIO.OUT)
GPIO.setup(L2, GPIO.OUT)
GPIO.setup(L3, GPIO.OUT)
GPIO.setup(L4, GPIO.OUT)

# Turn off all LEDs initially
GPIO.output(L1, GPIO.HIGH)
GPIO.output(L2, GPIO.HIGH)
GPIO.output(L3, GPIO.HIGH)
GPIO.output(L4, GPIO.HIGH)

try:
    while True:
        # Sequence 1: Light up one LED at a time in a loop
        GPIO.output(L1, GPIO.LOW)
        time.sleep(1)
        GPIO.output(L1, GPIO.HIGH)

        GPIO.output(L2, GPIO.LOW)
        time.sleep(1)
        GPIO.output(L2, GPIO.HIGH)

        GPIO.output(L3, GPIO.LOW)
        time.sleep(1)
        GPIO.output(L3, GPIO.HIGH)

        GPIO.output(L4, GPIO.LOW)
        time.sleep(1)
        GPIO.output(L4, GPIO.HIGH)

        # Sequence 2: Rapid blinking of LEDs one by one
        for i in range(4):
            GPIO.output(L1, GPIO.LOW)
            time.sleep(0.1)
            GPIO.output(L1, GPIO.HIGH)

            GPIO.output(L2, GPIO.LOW)
            time.sleep(0.1)
            GPIO.output(L2, GPIO.HIGH)

            GPIO.output(L3, GPIO.LOW)
            time.sleep(0.1)
            GPIO.output(L3, GPIO.HIGH)

            GPIO.output(L4, GPIO.LOW)
            time.sleep(0.1)
            GPIO.output(L4, GPIO.HIGH)

        # Sequence 3: Reverse rapid blinking
        for i in range(4):
            GPIO.output(L4, GPIO.LOW)
            time.sleep(0.1)
            GPIO.output(L4, GPIO.HIGH)

            GPIO.output(L3, GPIO.LOW)
            time.sleep(0.1)
            GPIO.output(L3, GPIO.HIGH)

            GPIO.output(L2, GPIO.LOW)
            time.sleep(0.1)
            GPIO.output(L2, GPIO.HIGH)

            GPIO.output(L1, GPIO.LOW)
            time.sleep(0.1)
            GPIO.output(L1, GPIO.HIGH)

        # Sequence 4: Alternating LEDs
        for i in range(4):
            GPIO.output(L4, GPIO.LOW)
            GPIO.output(L2, GPIO.LOW)
            time.sleep(0.25)
            GPIO.output(L4, GPIO.HIGH)
            GPIO.output(L2, GPIO.HIGH)

            GPIO.output(L1, GPIO.LOW)
            GPIO.output(L3, GPIO.LOW)
            time.sleep(0.25)
            GPIO.output(L1, GPIO.HIGH)
            GPIO.output(L3, GPIO.HIGH)

        # Sequence 5: All LEDs blinking together
        for i in range(4):
            GPIO.output(L4, GPIO.LOW)
            GPIO.output(L2, GPIO.LOW)
            GPIO.output(L1, GPIO.LOW)
            GPIO.output(L3, GPIO.LOW)
            time.sleep(0.25)
            GPIO.output(L4, GPIO.HIGH)
            GPIO.output(L2, GPIO.HIGH)
            GPIO.output(L1, GPIO.HIGH)
            GPIO.output(L3, GPIO.HIGH)
            time.sleep(0.25)

except KeyboardInterrupt:
    # Cleanup GPIO on exit
    print("\nExiting and cleaning up GPIO...")
    GPIO.cleanup()

Decimal Binary---------------------------------------------------------------------
import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BOARD)
GPIO.setup(3,GPIO.OUT)
GPIO.setup(5,GPIO.OUT)
GPIO.setup(7,GPIO.OUT)
GPIO.setup(11,GPIO.OUT)
GPIO.setup(13,GPIO.OUT)
GPIO.setup(15,GPIO.OUT)
GPIO.setup(19,GPIO.OUT)
GPIO.setup(21,GPIO.OUT)
#256GPIO.setup(23,GPIO.OUT)
def func(n):
if n==1 or n==0:
l.append(n)
length = len(l)
for i in range (8-length):
l.append(0)
l.reverse()
else:
r = n%2
l.append(r)
n = n//2
func(n)

g = [3,5,7,11,13,15,19,21]
for i in range(len(g)):
GPIO.output(g[i],GPIO.LOW)

while True:
n=int(input("Enter your number :"))
l=[]
func(n)
print(l)
j = len(l)
for i in range(j):
if l[i] ==1:
GPIO.output(g[i],GPIO.LOW)
else:
GPIO.output(g[i],GPIO.HIGH)

PWM-----------------------------------------------------------------
import RPi.GPIO as GPIO import time
GPIO.setmode(GPIO.BOARD) GPIO.setup(3,GPIO.OUT) # 1
GPIO.setup(5,GPIO.OUT) # 5
GPIO.setup(7,GPIO.OUT) # 6
a = float(input("TIME LENGTH")) e = 100000
n = ((a*e)**0.5) i=1
while True:
GPIO.output(7,GPIO.HIGH)

while i<n:

GPIO.output(5,GPIO.HIGH) GPIO.output(3,GPIO.LOW)

time.sleep(i/e)

GPIO.output(5,GPIO.LOW) GPIO.output(3,GPIO.HIGH)
time.sleep((n/e)-(i/e)) 
i=i+1
i=1

DHT11-------------------------------------------------------------------------
#include "DHT.h"  (library DHT and adafruit unified sensor and G0 PIN CONNECTED TO SENSOR)
#deflne DHTPIN 0 // Digital pin connected to the DHT sensor
#deflne DHTTYPE DHT11 // DHT 11
DHT dht(DHTPIN, DHTTYPE);
void setup() {
Serial.begin(9600); Serial.println(F("DHTxx
test!"));
dht.begin();
}
void loop() {
// Wait a few seconds between measurements. delay(2000);
// Reading temperature or humidity takes about 250 milliseconds!
// Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor) float h = dht.readHumidity();
// Read temperature as Celsius (the default) float t =
dht.readTemperature();
// Read temperature as Fahrenheit (isFahrenheit = true) float f =
dht.readTemperature(true);
// Check if any reads failed and exit early (to try again). if (isnan(h) ||
isnan(t) || isnan(f)) { Serial.println(F("Failed to read from DHT
sensor!")); return;
}
// Compute heat index in Fahrenheit (the default) float hif =
dht.computeHeatIndex(f, h);
// Compute heat index in Celsius (isFahreheit = false) float hic =
dht.computeHeatIndex(t, h, false);
Serial.print(F("Humidity: ")); Serial.print(h);
Serial.print(F("% Temperature: "));
Serial.print(t); Serial.print(F("°C "));
Serial.print(f);
Serial.print(F("°F Heat index: "));
Serial.print(hic); Serial.print(F("°C "));
Serial.print(hif); Serial.println(F("°F"));
}


BMP280 -----------------------------------------------------------------------------------------------
Connect the GND (ground) pin of the BMP280 sensor to the GND (ground) pin of the ESP32.
Connect the VCC pin of the BMP280 sensor to the 3.3V (3V3) pin of the ESP32. The SCL pin of the
BMP280 sensor should be connected to the G22 pin of the ESP32, and the SDA pin to the G21


This is a library for the BMP280 humidity, temperature & pressure sensor
Designed speciflcally to work with the Adafruit BMP280 Breakout
----> http://www.adafruit.com/products/2651
These sensors use I2C or SPI to communicate, 2 or 4 pins are required
to interface.
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#deflne BMP_SCK (13)
#deflne BMP_MISO (12)
#deflne BMP_MOSI (11)
#deflne BMP_CS (10)
Adafruit_BMP280 bmp; // I2C
//Adafruit_BMP280 bmp(BMP_CS); // hardware SPI
//Adafruit_BMP280 bmp(BMP_CS, BMP_MOSI, BMP_MISO, BMP_SCK);
void setup() {
Serial.begin(9600);
while ( !Serial ) delay(100); // wait for native usb
Serial.println(F("BMP280 test"));
unsigned status;
//status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
status = bmp.begin(0x76);
if (!status) {
Serial.println(F("Could not flnd a valid BMP280 sensor, check wiring or "
"try a different address!"));
Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
Serial.print(" ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
Serial.print(" ID of 0x56-0x58 represents a BMP 280,\n");
Serial.print(" ID of 0x60 represents a BME 280.\n");
Serial.print(" ID of 0x61 represents a BME 680.\n");
while (1) delay(10);
}
/* Default settings from datasheet. */
bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, /* Operating Mode. */
Adafruit_BMP280::SAMPLING_X2, /* Temp. oversampling */
Adafruit_BMP280::SAMPLING_X16, /* Pressure oversampling */
Adafruit_BMP280::FILTER_X16, /* Filtering. */
Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
}
void loop() {
Serial.print(F("Temperature = "));
Serial.print(bmp.readTemperature());
Serial.println(" *C");
Serial.print(F("Pressure = "));
Serial.print(bmp.readPressure());
Serial.println(" Pa");
Serial.print(F("Approx altitude = "));
Serial.print(bmp.readAltitude(1011.9)); /* Adjusted to local forecast! */
Serial.println(" m");
Serial.println();
delay(2000);
}

Communication code----------------------------------------------------------------
  Used UDP(USER DATAGRAM PROTOCOL)

  
Server code:-
import socket
host = '10.205.2.238'
port = 8000
storedValue = "server client comm group 1"
def setupServer():
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print("Socket created.")
try:
s.bind((host, port))
except socket.error as msg:
print(msg)
print("Socket bind comlete.")
return s
def setupConnection():
s.listen(1) # Allows one connection at a time.
conn, address = s.accept()
print("Connected to: " + address[0] + ":" + str(address[1]))
return conn
def GET():
reply = storedValue
return reply
def REPEAT(dataMessage):
reply = dataMessage[1]
return reply
def dataTransfer(conn):
# A big loop that sends/receives data until told not to.
while True:
# Receive the data
data = conn.recv(1024) # receive the data
data = data.decode('utf-8')
# Split the data such that you separate the command
# from the rest of the data.
dataMessage = data.split(' ', 1)
command = dataMessage[0]
if command == 'GET':
reply = GET()
elif command == 'REPEAT':
reply = REPEAT(dataMessage)
elif command == 'EXIT':
print("Our client has left us :(")
break
elif command == 'KILL':
print("Our server is shutting down.")
s.close()
break
else:
reply = 'Unknown Command'
# Send the reply back to the client
conn.sendall(str.encode(reply))
print("Data has been sent!")
conn.close()
s = setupServer()
while True:
try:
conn = setupConnection()
dataTransfer(conn)
except:
break
Client code:-
import socket
from time import sleep
host = '10.205.2.238'
port = 8000
def setupSocket():
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((host, port))
return s
def sendReceive(s, message):
s.send(str.encode(message))
reply = s.recv(1024)
print("We have received a reply")
print("Send closing message.")
s.send(str.encode("EXIT"))
s.close()
reply = reply.decode('utf-8')
return reply
def transmit(message):
s = setupSocket()
response = sendReceive(s, "GET")
return response
response = transmit("GET")
print(response)

BMP280 DATA TRANSMIT------------------------------------------------------------------------------------

Circuits connections are shown below. Inbelow connection VCC of bmp-280 is connected to 3.3V pin of raspberry pie board and both ground pins are connected together, SCL pin and SDA pins of bmp-280 are connected to GPIO-2 and GPIO-3 pins of raspberry pie 4 board.


Server code:
import socket
import json
import csv
host = '10.205.2.245'
port = 8000
def setupServer():
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print("Socket created.")
try:
s.bind((host, port))
except socket.error as msg:
print(msg)
print("Socket bind comlete.")
return s
def setupConnection():
s.listen(1) # Allows one connection at a time.
conn, address = s.accept()
print("Connected to: " + address[0] + ":" + str(address[1]))
return conn
def dataTransfer(conn):
# A big loop that sends/receives data until told not to.
while True:
# Receive the data
data = conn.recv(1024) # receive the data
data = data.decode('utf-8')
print(data)
# Split the data such that you separate the command
# from the rest of the data.
dataMessage = data.split(' ', 1)
command = dataMessage[0]
dataext = json.loads(data)
fi eldname = dataext.keys()
with open('/home/redpitaya/Documents/meow.csv', mode = 'a', newline='') as fi le:
writer = csv.DictWriter(fi le, fi eldnames = fi eldname)
fi le.seek(0,2)
if fi le.tell()==0:
writer.writeheader()
writer.writerow(dataext)
print()
# conn.close()
s = setupServer()
while True:
conn = setupConnection()
while(True):
dataTransfer(conn)



Client code:-
#!/usr/bin/env python
import time
from bmp280 import BMP280
try:
from smbus2 import SMBus
except ImportError:
from smbus import SMBus
import socket
host = '10.205.2.245'
port = 8000
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((host, port))
print("""temperature-and-pressure.py - Displays the temperature and pressure.
Press Ctrl+C to exit!
""")
# Initialise the BMP280
bus = SMBus(1)
bmp280 = BMP280(i2c_dev=bus)
while True:
temperature = bmp280.get_temperature()
pressure = bmp280.get_pressure()
degree_sign = u"\N{DEGREE SIGN}"
format_temp = "{:.2f}".format(temperature)
print('Temperature = ' + format_temp + degree_sign + 'C')
format_press = "{:.2f}".format(pressure)
print('Pressure = ' + format_press + ' hPa \n')
jsonData = "{ \"temperature\":" + "{:.2f}".format(temperature) + ", \"pressure\":" + "{:.2f}".format(pressure) + "}"
s.send(str.encode(jsonData))
time.sleep(4)

MPU9250(rpi)---------------------------------------------------------------------------------------
Circuits connections are shown below. In below connection VCC of MPU-9250 is connected to 3.3V pin of raspberry pie board and both ground pins are connected together, SDA pin and SCL pins of MPU-9250 are connected to GPIO-2 and GPIO-3 pins of raspberry pie 4 board.

import time
from mpu9250_jmdev.registers import *
from mpu9250_jmdev.mpu_9250 import MPU9250
# Create an MPU9250 instance
mpu = MPU9250(
address_ak=AK8963_ADDRESS,
address_mpu_master=MPU9050_ADDRESS_68, # In case the MPU9250 is connected to another I2C device
address_mpu_slave=None,
bus=1,
gfs=GFS_1000,
afs=AFS_8G,
mfs=AK8963_BIT_16,
mode=AK8963_MODE_C100HZ)
# Confi gure the MPU9250
mpu.confi gure()
while True:
# Read the accelerometer, gyroscope, and magnetometer values
accel_data = mpu.readAccelerometerMaster()
gyro_data = mpu.readGyroscopeMaster()
mag_data = mpu.readMagnetometerMaster()
# Print the sensor values
print("Accelerometer:", accel_data)
print("Gyroscope:", gyro_data)
print("Magnetometer:", mag_data)
# Wait for 1 second before the next reading
time.sleep(1)


DHT COMMUNICATION-----------------------------------------------------------------------
pip install Adafruit_DHT.
GPIO 4 FOR DHT

Server code:
import socket
import json
import csv
host = '10.205.2.245'
port = 8000
def setupServer():
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print("Socket created.")
try:
s.bind((host, port))
except socket.error as msg:
print(msg)
print("Socket bind comlete.")
return s
def setupConnection():
s.listen(1) # Allows one connection at a time.
conn, address = s.accept()
print("Connected to: " + address[0] + ":" + str(address[1]))
return conn
def dataTransfer(conn):
# A big loop that sends/receives data until told not to.
while True:
# Receive the data
data = conn.recv(1024) # receive the data
data = data.decode('utf-8')
print(data)
# Split the data such that you separate the command
# from the rest of the data.
dataMessage = data.split(' ', 1)
command = dataMessage[0]
dataext = json.loads(data)
fi eldname = dataext.keys()
with open('/home/redpitaya/Documents/meow.csv', mode = 'a', newline='') as fi le:
writer = csv.DictWriter(fi le, fi eldnames = fi eldname)
fi le.seek(0,2)
if fi le.tell()==0:
writer.writeheader()
writer.writerow(dataext)
print()
# conn.close()
s = setupServer()
while True:
conn = setupConnection()
while(True):
dataTransfer(conn)

Client code-

#!/usr/bin/env python
import time
import Adafruit_DHT  # Library for DHT11
import socket

# Sensor setup
DHT_SENSOR = Adafruit_DHT.DHT11
DHT_PIN = 4  # Replace with the GPIO pin connected to the DHT11 data pin

# Server connection setup
host = '10.205.2.245'  # Replace with your server's IP
port = 8000
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((host, port))

print("""temperature-and-humidity.py - Displays the temperature and humidity.
Press Ctrl+C to exit!
""")

while True:
    # Read temperature and humidity from the DHT11 sensor
    humidity, temperature = Adafruit_DHT.read(DHT_SENSOR, DHT_PIN)

    if humidity is not None and temperature is not None:
        # Format the data
        degree_sign = u"\N{DEGREE SIGN}"
        format_temp = "{:.2f}".format(temperature)
        format_hum = "{:.2f}".format(humidity)

        print(f'Temperature = {format_temp}{degree_sign}C')
        print(f'Humidity = {format_hum}%\n')

        # Prepare JSON data
        jsonData = f'{{ "temperature": {format_temp}, "humidity": {format_hum} }}'

        # Send data to the server
        s.send(str.encode(jsonData))
    else:
        print("Failed to retrieve data from DHT11 sensor")

    # Delay for a few seconds before the next read
    time.sleep(4)
